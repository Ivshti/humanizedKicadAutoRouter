"""Router core: search, commit with exact verification, rip-up with dependency tracking.

A connection joins two copper components of one net (or a component and a plane).
The search runs on a cost raster built for that connection; the found path becomes
real geometry (tracks + vias), is pulled tight, and is verified exactly before it is
committed. Negotiation mode lets a search cross other routes at a price and rips the
routes it crossed; they go back on the queue.
"""
import ctypes
import math
import time
from dataclasses import dataclass, field

import numpy as np
from shapely.geometry import LineString, Point, box

import native
from boardModel import B, F, IN1, IN2
from native import edt
from space import HARD, NLAYERS, Item, Space

@dataclass
class Knobs:
    grid: float = 0.025
    margin: float = 0.02
    signalWidth: float = 0.2
    powerWidth: dict = field(default_factory=lambda: {"Power": 0.5, "Rail": 0.3})
    turn45Mm: float = 0.1
    turn90Mm: float = 0.5
    viaMm: float = 3.0
    otherLayerMult: float = 2.5
    neckMm: float = 1.0          # power tracks may neck down this close to their own pads
    escapeMm: float = 1.5        # body keep-outs do not apply this close to the route's own pads
    attractMult: float = 0.55    # bundle members pay this much less one pitch from a sibling
    thermalPenalty: float = 2.0  # cost added on GND pads' thermal spoke corridors
    bodyMinAreaMm2: float = 40.0
    stitchPitchMm: float = 4.0
    ampRiseC: float = 20.0
    outerUm: float = 35.0
    ampMargin: float = 1.08
    hweight: float = 1.15
    negotiatePenalty: float = 6.0
    corridorTileMm: float = 2.0
    corridorOut: float = 3.0     # bundle members pay this much more outside their corridor
    corridorViaMm: float = 4.0   # what a layer change costs a corridor, per member
    corridorShare: float = 0.75  # a corridor tile another group runs through, while it has room
    viaCleanupMult: float = 4.0  # the last cleanup pass prices vias this much higher
    joinMm: float = 6.0          # bundle members whose ends are this close share a corridor
    maxGroup: int = 6
    maxExpand: int = 30_000_000


def padMinDim(pad):
    minx, miny, maxx, maxy = pad.poly.bounds
    return min(maxx - minx, maxy - miny)


def ipcWidth(amps, k):
    """IPC-2221 external conductor width (mm) for `amps` at k.ampRiseC rise, k.outerUm copper."""
    areaMil2 = (amps / (0.048 * k.ampRiseC ** 0.44)) ** (1 / 0.725)
    return areaMil2 / (k.outerUm / 25.4) * 0.0254 * k.ampMargin


@dataclass
class NetRule:
    width: float
    narrow: float
    clearance: float
    viaDia: float
    viaDrill: float
    cls: str


@dataclass
class Conn:
    cid: int
    net: str
    a: object                 # Pad
    b: object                 # Pad, or None for "to the plane"
    kind: str                 # rf | gnd | pwr | sig
    block: str = ""
    bundle: str = ""
    prefLayer: int = -1       # -1: none
    strict: bool = False      # no vias, pad layer only
    routed: bool = False
    fails: int = 0
    rid: int = 0
    order: float = 0.0
    amps: float = 0.0         # current this connection carries (power flows); 0 = unknown
    corridor: object = None   # (2, tiles, tiles) bool: where its bundle group runs, F and B


@dataclass
class Route:
    rid: int
    conn: Conn
    items: list
    nodes: list               # union-find nodes this route joins
    parents: set


class Router:
    def __init__(self, model, knobs=None, log=print):
        self.m = model
        self.k = knobs or Knobs()
        self.log = log
        self.kernel = native.kernel
        dc = model.classes["Default"].clearance
        self.sp = Space(model.outline, self.k.grid, self.k.margin, model.edgeClearance, dc)
        self.sp.defaultClearance = dc
        self.netId = {n: i + 1 for i, n in enumerate(sorted(model.netClass))}
        self.netName = {i: n for n, i in self.netId.items()}
        self.rules = {n: self.ruleFor(n) for n in model.netClass}
        self.routes = {}
        self.nextRid = 1
        self.children = {}
        self.netItems = {}
        self.planeNets = {"GND"}
        self.flowNets = set()      # power nets whose current is planned from flows
        self.hist = np.zeros((NLAYERS, self.sp.H, self.sp.W), np.float32)
        self.stats = {"searches": 0, "searchS": 0.0, "costS": 0.0, "commitS": 0.0}
        self.corridors = None      # Corridors planner, once the bundle phase starts
        self.loadFixed()

    # ---- setup ------------------------------------------------------------------------
    def ruleFor(self, net):
        cls = self.m.netClass.get(net, "Default")
        nc = self.m.classes[cls]
        if cls in self.k.powerWidth:
            w, narrow = self.k.powerWidth[cls], nc.width
        elif cls == "Default":
            w, narrow = self.k.signalWidth, nc.width
        else:
            w = narrow = nc.width
        return NetRule(w, min(narrow, w), nc.clearance, nc.viaDia, nc.viaDrill, cls)

    def ruleForConn(self, conn):
        """Net rule, widened to carry the connection's current (IPC-2221, outer copper) and
        allowed to neck to the size of the pads it lands on."""
        base = self.rules[conn.net]
        if conn.amps <= 0:
            if conn.net in self.flowNets:  # a tap on a planned power net carries nothing
                return NetRule(base.narrow, base.narrow, base.clearance, base.viaDia,
                               base.viaDrill, base.cls)
            return base
        need = ipcWidth(conn.amps, self.k)
        width = max(base.width if conn.kind != "pwr" else base.narrow, need)
        pads = [p for p in (conn.a, conn.b) if p is not None]
        padMin = min(padMinDim(p) for p in pads)
        narrow = min(width, max(base.narrow, padMin))
        return NetRule(width, narrow, base.clearance, base.viaDia, base.viaDrill, base.cls)

    def padClearance(self, pad):
        base = self.rules[pad.net].clearance if pad.net else self.sp.defaultClearance
        return max(base, pad.ownClearance)

    def loadFixed(self):
        sp = self.sp
        for pad in self.m.pads:
            nid = self.netId.get(pad.net, HARD) if pad.net else HARD
            # one item per distinct copper shape (padstacks may differ per layer)
            groups = {}
            for l in pad.layers:
                geom = pad.polys.get(l, pad.poly)
                groups.setdefault(geom.wkb, (geom, []))[1].append(l)
            for geom, layers in groups.values():
                it = Item(0, nid, "pad", tuple(layers), geom, self.padClearance(pad),
                          pts=(pad.center,), smd=pad.smd, tag=f"{pad.ref}.{pad.num}")
                it.pad = pad
                sp.add(it)
                self.netItems.setdefault(nid, set()).add(it.iid)
            if pad.smd:
                win, mask = sp.rasterMask(pad.poly)
                i0, j0, i1, j1 = win
                for l in pad.layers:
                    if l in (F, B):
                        sub = sp.smdPad[i0:i1, j0:j1]
                        sub[mask] = nid
        hc = self.m.minHoleClearance - self.sp.defaultClearance
        for (x, y, r, hnet) in self.m.holes:
            geom = Point(x, y).buffer(r)
            hid = self.netId[hnet] if hnet else HARD
            it = Item(0, hid, "hole", tuple(range(NLAYERS)), geom, 0.0, pts=((x, y),), drill=2 * r)
            sp.add(it)
            win, mask = sp.rasterMask(geom)
            if mask.size:
                i0, j0, i1, j1 = win
                sp.holeMask[i0:i1, j0:j1] |= mask
            if hc > 0 and hid == HARD:
                win, mask = sp.rasterMask(geom, hc)
                i0, j0, i1, j1 = win
                for l in range(NLAYERS):
                    sp.occ[l, i0:i1, j0:j1][mask & (sp.occ[l, i0:i1, j0:j1] == 0)] = HARD
                    sp.fixed[l, i0:i1, j0:j1][mask & (sp.fixed[l, i0:i1, j0:j1] == 0)] = HARD
        edge = Item(0, HARD, "edge", tuple(range(NLAYERS)), self.m.outline.boundary, 0.0)
        sp.add(edge)
        for ko in self.m.keepouts:
            it = Item(0, HARD, "keepout", tuple(ko.layers), ko.poly, 0.0,
                      noTracks=ko.noTracks, noVias=ko.noVias)
            # keepouts are not copper: index them for exact checks but keep them off occ
            it.iid = sp.nextId
            sp.nextId += 1
            sp.items[it.iid] = it
            sp.index(it, True)
            if ko.noTracks:
                win, mask = sp.rasterMask(ko.poly, 0.25 + self.k.margin)
                i0, j0, i1, j1 = win
                for l in ko.layers:
                    sp.trackKeep[l, i0:i1, j0:j1] |= mask
            if ko.noVias:
                win, mask = sp.rasterMask(ko.poly, 0.45 + self.k.margin)
                i0, j0, i1, j1 = win
                sp.viaKeep[i0:i1, j0:j1] |= mask
        self.loadStrategyRasters()

    def loadStrategyRasters(self):
        """Human conventions as rasters: no foreign copper under module/antenna bodies,
        keep GND thermal spokes free, and a static distance-to-hole field."""
        sp, k = self.sp, self.k
        self.bodyKeep = np.zeros((NLAYERS, sp.H, sp.W), bool)
        self.bodyViaKeep = np.zeros((sp.H, sp.W), bool)
        self.bodies = []
        for fp in self.m.footprints.values():
            crt = fp.courtyard
            if crt is None or crt.is_empty:
                continue
            if fp.ref.startswith("ANT"):
                body = crt                      # the antenna owns its whole courtyard
            elif crt.area >= k.bodyMinAreaMm2:
                body = crt.buffer(-1.0)         # under a module/connector body
            else:
                continue
            if body.is_empty:
                continue
            self.bodies.append((fp.ref, fp.side, body))
            win, mask = sp.rasterMask(body)
            i0, j0, i1, j1 = win
            self.bodyKeep[fp.side, i0:i1, j0:j1] |= mask
            self.bodyViaKeep[i0:i1, j0:j1] |= mask
        self.thermalCost = np.zeros((NLAYERS, sp.H, sp.W), np.float32)
        for pad in self.m.nets.get("GND", []):
            if not pad.smd:
                continue
            minx, miny, maxx, maxy = pad.poly.bounds
            cx, cy = pad.center
            reach = 0.7
            for (ax, ay, bx, by) in ((minx - reach, cy - 0.2, maxx + reach, cy + 0.2),
                                     (cx - 0.2, miny - reach, cx + 0.2, maxy + reach)):
                win, mask = sp.rasterMask(box(ax, ay, bx, by).difference(pad.poly))
                if mask.size == 0:
                    continue
                i0, j0, i1, j1 = win
                for l in pad.layers:
                    if l in (F, B):
                        self.thermalCost[l, i0:i1, j0:j1][mask] = k.thermalPenalty
        self.holeDist = (edt(sp.holeMask) * sp.g).astype(np.float32)

    # ---- connectivity ------------------------------------------------------------------
    def nodeOf(self, item):
        if item.kind == "pad":
            return ("pad", item.pad.pid)
        return ("route", item.route)

    def components(self, net):
        """Union-find over pads, routes and the plane of one net."""
        parent = {}

        def find(x):
            parent.setdefault(x, x)
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        def union(a, b):
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[ra] = rb

        for pad in self.m.nets.get(net, []):
            find(("pad", pad.pid))
            if net in self.planeNets and not pad.smd:
                union(("pad", pad.pid), ("plane", net))
        for r in self.routes.values():
            if r.conn.net != net:
                continue
            for n in r.nodes:
                union(("route", r.rid), n)
        return find

    def connected(self, conn):
        find = self.components(conn.net)
        tgt = ("plane", conn.net) if conn.b is None else ("pad", conn.b.pid)
        return find(("pad", conn.a.pid)) == find(tgt)

    def componentItems(self, net, root, find):
        nid = self.netId[net]
        out = []
        for iid in self.netItems.get(nid, ()):
            it = self.sp.items[iid]
            if find(self.nodeOf(it)) == root:
                out.append(it)
        return out

    # ---- cost raster -------------------------------------------------------------------
    def buildCost(self, conn, rule, win, layers, srcItems, dstItems, negotiate, extraMargin):
        sp, k, g = self.sp, self.k, self.k.grid
        nid = self.netId[conn.net]
        plane = conn.net in self.planeNets
        i0, j0, i1, j1 = win
        h, w = i1 - i0, j1 - j0
        m = k.margin + extraMargin
        rW = rule.width / 2 + rule.clearance + m
        rN = rule.narrow / 2 + rule.clearance + m
        endPads = [it for it in srcItems + dstItems if it.kind == "pad"]
        ownPads = [it for it in endPads if it.pad is conn.a or it.pad is conn.b]
        # escape zone: close to the pads this connection starts/ends on, body keep-outs
        # do not apply (a pin under a module must get out) and power may neck down
        own = np.zeros((h, w), bool)
        for it in ownPads:
            self.paint(own, win, it)
        padDist = edt(own) * g if own.any() else np.full((h, w), 1e3)
        escape = padDist < k.escapeMm
        neckZone = padDist < k.neckMm if rule.narrow < rule.width else np.zeros((h, w), bool)
        attract = self.bundleAttraction(conn, rule, win, layers) if conn.bundle else None
        cost = np.empty((len(layers), h, w), np.float32)
        wideOk = np.empty((len(layers), h, w), bool)
        for li, l in enumerate(layers):
            dAll = sp.foreignDist(nid, l, win)
            wideOk[li] = dAll >= rW
            ok = wideOk[li] | (neckZone & (dAll >= rN))
            soft = None
            if negotiate:
                # routes in the way will be ripped: judge width against fixed copper only
                dFix = sp.foreignDist(nid, l, win, fixedOnly=True)
                okFix = (dFix >= rW) | (neckZone & (dFix >= rN))
                soft = okFix & ~ok
                ok = okFix
                wideOk[li] = dFix >= rW
            ok &= ~sp.trackKeep[l, i0:i1, j0:j1]
            if not plane:
                ok &= ~(self.bodyKeep[l, i0:i1, j0:j1] & ~escape)
            mult = 1.0
            if conn.prefLayer >= 0 and l != conn.prefLayer:
                mult = k.otherLayerMult
            c = np.full((h, w), mult, np.float32)
            c += self.hist[l, i0:i1, j0:j1]
            if not plane:
                c += self.thermalCost[l, i0:i1, j0:j1]
            if attract is not None:
                c[attract[li]] *= k.attractMult
            if conn.corridor is not None and l in (F, B):
                inside = self.corridors.fine(conn.corridor, 0 if l == F else 1, win)
                c[~inside & ~escape] *= k.corridorOut
            if soft is not None:
                c[soft] *= (1.0 + k.negotiatePenalty)
            c[~ok] = -1
            cost[li] = c
        # own pad interiors: always enterable by a track that stays inside the pad
        for it in endPads:
            wwin, mask = self.cachedMask(it, f"inner:{rule.narrow}")
            if mask.size == 0:
                continue
            a0, b0, a1, b1 = wwin
            c0, d0 = max(a0, i0), max(b0, j0)
            c1, d1 = min(a1, i1), min(b1, j1)
            if c1 <= c0 or d1 <= d0:
                continue
            sub = mask[c0 - a0:c1 - a0, d0 - b0:d1 - b0]
            for li, l in enumerate(layers):
                if l in it.layers:
                    view = cost[li, c0 - i0:c1 - i0, d0 - j0:d1 - j0]
                    view[sub & (view < 0)] = 1.0
        # vias
        rv = rule.viaDia / 2
        dAny = sp.foreignDist(nid, None, win, fixedOnly=negotiate)
        viaok = dAny >= rv + rule.clearance + m
        viaok &= ~sp.viaKeep[i0:i1, j0:j1]
        if not plane:
            viaok &= ~(self.bodyViaKeep[i0:i1, j0:j1] & ~escape)
        viaok &= sp.maskDist(sp.smdPad, win, pred=lambda a: a == nid) >= rv + 0.03
        viaok &= self.holeDist[i0:i1, j0:j1] >= rule.viaDrill / 2 + self.m.minHoleToHole + m
        viaok &= sp.maskDist(sp.viaAt, win, pred=lambda a: a == nid) >= \
            rule.viaDrill + self.m.minHoleToHole + m
        for (p, r) in getattr(self, "forbidVias", ()):
            ys, xs = sp.cellCenters(win)
            viaok &= (xs - p[0]) ** 2 + (ys - p[1]) ** 2 >= r * r
        return cost, viaok.astype(np.uint8), wideOk, None

    def bundleAttraction(self, conn, rule, win, layers):
        """Cells one track pitch away from tracks already laid by the same bundle."""
        sp, g = self.sp, self.k.grid
        i0, j0, i1, j1 = win
        h, w = i1 - i0, j1 - j0
        x0, y0 = sp.toMm(i0, j0)
        x1, y1 = sp.toMm(i1, j1)
        out = np.zeros((len(layers), h, w), bool)
        pitch = rule.width + rule.clearance + self.k.margin
        for li, l in enumerate(layers):
            mask = np.zeros((h, w), bool)
            for it in sp.query([l], (x0, y0, x1, y1)):
                if it.kind != "track" or it.net == self.netId[conn.net]:
                    continue
                r = self.routes.get(it.route)
                if r is None or r.conn.bundle != conn.bundle:
                    continue
                self.paint(mask, win, it, "land")
            if mask.any():
                d = edt(mask) * g
                out[li] = (d >= pitch - g) & (d <= pitch + 2 * g)
        return out

    def cachedMask(self, item, key="copper"):
        """(win, mask) raster of an item, computed once: copper shape, or the centreline
        cells a route may land on ("land")."""
        cache = item.__dict__.setdefault("_masks", {})
        if key not in cache:
            sp = self.sp
            if key == "copper" and item.kind == "pad":
                cache[key] = sp.rasterMask(item.geom)
            elif item.kind == "track":
                r = item.width / 2 if key == "copper" else sp.g * 0.75
                cache[key] = sp.segMask(item.pts[0], item.pts[1], r)
            elif item.kind == "via":
                r = item.width / 2 if key == "copper" else sp.g * 0.75
                cache[key] = sp.segMask(item.pts[0], item.pts[0], r)
            elif key.startswith("inner:"):
                inner = item.geom.buffer(-float(key[6:]) / 2 + 1e-3)
                if inner.is_empty:
                    inner = Point(item.pad.center).buffer(sp.g * 0.8)
                cache[key] = sp.rasterMask(inner)
            else:  # land on a pad: anywhere on its copper
                cache[key] = self.cachedMask(item, "copper")
        return cache[key]

    @staticmethod
    def blit(arr, win, wwin, mask):
        i0, j0, i1, j1 = win
        if mask.size == 0:
            return
        a0, b0, a1, b1 = wwin
        c0, d0 = max(a0, i0), max(b0, j0)
        c1, d1 = min(a1, i1), min(b1, j1)
        if c1 <= c0 or d1 <= d0:
            return
        arr[c0 - i0:c1 - i0, d0 - j0:d1 - j0] |= mask[c0 - a0:c1 - a0, d0 - b0:d1 - b0]

    def paint(self, arr, win, item, key="copper"):
        wwin, mask = self.cachedMask(item, key)
        self.blit(arr, win, wwin, mask)

    def endpointCells(self, items, layers, win, cost):
        """Window cells where a route may start/end on these items."""
        i0, j0, i1, j1 = win
        h, w = i1 - i0, j1 - j0
        out = np.zeros((len(layers), h, w), bool)
        for it in items:
            m2 = np.zeros((h, w), bool)
            self.paint(m2, win, it, "land")
            for li, l in enumerate(layers):
                if l in it.layers:
                    out[li] |= m2
        out &= cost >= 0
        return out

    # ---- search ------------------------------------------------------------------------
    def search(self, conn, layers, winPad, negotiate=False, extraMargin=0.0, viaMm=None):
        sp, k = self.sp, self.k
        rule = self.ruleForConn(conn)
        find = self.components(conn.net)
        rootA = find(("pad", conn.a.pid))
        srcItems = self.componentItems(conn.net, rootA, find)
        toPlane = conn.b is None
        if toPlane:
            rootB = find(("plane", conn.net))
            dstItems = [it for it in self.componentItems(conn.net, rootB, find)]
            ctr = conn.a.center
            bounds = (ctr[0], ctr[1], ctr[0], ctr[1])
        else:
            rootB = find(("pad", conn.b.pid))
            dstItems = self.componentItems(conn.net, rootB, find)
            ax, ay = conn.a.center
            bx, by = conn.b.center
            bounds = (min(ax, bx), min(ay, by), max(ax, bx), max(ay, by))
        if conn.amps > 0:
            # current must not be routed through a thinner tap: land on pads, vias or
            # copper at least as wide as this connection
            ends = {conn.a.ref, conn.b.ref if conn.b is not None else None}

            def okLand(it):
                if it.kind == "pad":
                    return it.pad.ref in ends
                r = self.routes.get(it.route)
                return r is not None and r.conn.amps >= conn.amps - 1e-6
            srcItems = [it for it in srcItems if okLand(it)] or srcItems
            dstItems = [it for it in dstItems if okLand(it)] or dstItems
        win = sp.winFor(bounds, winPad)
        t0 = time.perf_counter()
        cost, viaok, wideOk, _ = self.buildCost(conn, rule, win, layers, srcItems, dstItems,
                                                      negotiate, extraMargin)
        if conn.strict:
            viaok[:] = 0
        src = self.endpointCells(srcItems, layers, win, cost)
        dst = self.endpointCells(dstItems, layers, win, cost)
        self.stats["costS"] += time.perf_counter() - t0
        if not src.any() or (not dst.any() and not toPlane):
            return None
        Ls, h, w = cost.shape
        srcIdx = np.flatnonzero(src).astype(np.int32)
        tgt = dst.astype(np.uint8)
        tvia = viaok.copy() if toPlane else None
        vCost = (viaMm if viaMm is not None else k.viaMm) / k.grid
        ys, xs = np.nonzero(dst.any(axis=0)) if dst.any() else (np.array([0]), np.array([0]))
        tx0, tx1, ty0, ty1 = int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())
        maxPath = h * w * 2 + 16
        outPath = np.empty(min(maxPath, 4_000_000), np.int32)
        expanded = ctypes.c_int64(0)
        cost = np.ascontiguousarray(cost)
        t0 = time.perf_counter()
        n = self.kernel.astar(Ls, h, w, cost.ctypes.data, viaok.ctypes.data, ctypes.c_float(vCost),
                              tgt.ctypes.data, tvia.ctypes.data if tvia is not None else None,
                              srcIdx.ctypes.data, len(srcIdx), ctypes.c_float(k.turn45Mm / k.grid),
                              ctypes.c_float(k.turn90Mm / k.grid),
                              ctypes.c_float(0.0 if toPlane else k.hweight),
                              tx0, ty0, tx1, ty1, outPath.ctypes.data, len(outPath),
                              ctypes.c_int64(k.maxExpand), ctypes.byref(expanded), None)
        self.stats["searches"] += 1
        self.stats["searchS"] += time.perf_counter() - t0
        if n <= 0:
            return None
        path = outPath[:n].tolist()
        terminalVia = path[-1] == -1
        if terminalVia:
            path = path[:-1]
        cells = []
        for f in path:
            li, rem = divmod(f, h * w)
            i, j = divmod(rem, w)
            cells.append((layers[li], i + win[0], j + win[1], bool(wideOk[li, i, j])))
        return dict(cells=cells, terminalVia=terminalVia, srcItems=srcItems, dstItems=dstItems,
                    rule=rule, win=win)

    # ---- geometry ----------------------------------------------------------------------
    def pathToGeometry(self, conn, res):
        """Cells -> runs of vertices per layer + via positions."""
        sp = self.sp
        cells = res["cells"]
        runs, vias = [], []
        cur = [cells[0]]
        for prev, c in zip(cells, cells[1:]):
            if c[0] != prev[0]:
                runs.append(cur)
                vias.append(sp.toMm(c[1], c[2]))
                cur = [c]
            else:
                cur.append(c)
        runs.append(cur)
        out = []
        for run in runs:
            pts, wide = [], []
            for idx, c in enumerate(run):
                p = sp.toMm(c[1], c[2])
                if idx == 0 or idx == len(run) - 1:
                    pts.append(p)
                    wide.append(c[3])
                    continue
                pa, pb = run[idx - 1], run[idx + 1]
                d1 = (c[1] - pa[1], c[2] - pa[2])
                d2 = (pb[1] - c[1], pb[2] - c[2])
                if d1 != d2 or pa[3] != c[3] or pb[3] != c[3]:
                    pts.append(p)
                    wide.append(c[3])
            out.append((run[0][0], pts, wide))
        return out, vias

    def trackItem(self, net, layer, p1, p2, width, clearance, rid):
        geom = LineString([p1, p2]).buffer(width / 2, 16) if p1 != p2 else Point(p1).buffer(width / 2, 16)
        return Item(0, self.netId[net], "track", (layer,), geom, clearance, width=width,
                    pts=(p1, p2), route=rid)

    def viaItem(self, net, p, rule, rid):
        geom = Point(p).buffer(rule.viaDia / 2, 16)
        return Item(0, self.netId[net], "via", tuple(range(NLAYERS)), geom, rule.clearance,
                    width=rule.viaDia, pts=(p,), drill=rule.viaDrill, route=rid)

    def check(self, it, ignore=()):
        return self.sp.violations(it.net, it.layers, it.geom, it.clearance, isVia=it.kind == "via",
                                  drill=it.drill, holeToHole=self.m.minHoleToHole,
                                  holeClearance=self.m.minHoleClearance, ignore=ignore)

    @staticmethod
    def octilinear(p, q):
        dx, dy = q[0] - p[0], q[1] - p[1]
        if abs(dx) < 1e-6 or abs(dy) < 1e-6:
            return True
        return abs(abs(dx) - abs(dy)) < 1e-6

    def segOk(self, conn, layer, p, q, width, rule):
        it = self.trackItem(conn.net, layer, p, q, width, rule.clearance, 0)
        return not self.check(it)

    def pullTight(self, conn, layer, pts, wide, rule):
        """Greedy string pulling with octilinear shortcuts that pass the exact check at the
        width the segment will have. Only stretches the raster planned as necks may narrow."""
        def width(k0, k1):
            if all(wide[k0:k1 + 1]):
                return rule.width
            return self.pickWidth(conn, layer, pts[k0], pts[k1], rule)
        if len(pts) <= 2:
            return pts, [width(0, len(pts) - 1)]
        out, widths = [pts[0]], []
        kidx = 0
        while kidx < len(pts) - 1:
            nxt = kidx + 1
            for j in range(len(pts) - 1, kidx + 1, -1):
                if not self.octilinear(pts[kidx], pts[j]):
                    continue
                wj = rule.width if all(wide[kidx:j + 1]) else rule.narrow
                if self.segOk(conn, layer, pts[kidx], pts[j], wj, rule):
                    nxt = j
                    break
            widths.append(width(kidx, nxt))
            out.append(pts[nxt])
            kidx = nxt
        return out, widths

    def pickWidth(self, conn, layer, p, q, rule):
        """Widest that fits: full width, else neck to the nearer end pad's own width."""
        cands = [rule.width]
        if conn.amps > 0 and conn.b is not None:
            mid = ((p[0] + q[0]) / 2, (p[1] + q[1]) / 2)
            pad = min((conn.a, conn.b), key=lambda pd: math.dist(pd.center, mid))
            cands.append(min(rule.width, max(rule.narrow, padMinDim(pad))))
            cands.append(min(rule.width, max(rule.narrow, ipcWidth(conn.amps, self.k) / self.k.ampMargin)))
        for wdt in sorted(set(cands), reverse=True):
            if wdt > rule.narrow and self.segOk(conn, layer, p, q, wdt, rule):
                return wdt
        return rule.narrow

    def entryPoint(self, conn, layer, p, items, rule):
        """Extend a route end to the centre of the pad (or onto the centreline of a track)."""
        for it in items:
            if layer not in it.layers or not it.geom.buffer(1e-3).contains(Point(p)):
                continue
            if it.kind == "pad":
                c = it.pad.center
                if c != p and self.octilinear(p, c) and self.segOk(conn, layer, c, p, rule.narrow, rule):
                    return c
                return None
            if it.kind == "track":
                ls = LineString(it.pts)
                q = ls.interpolate(ls.project(Point(p)))
                q = (q.x, q.y)
                if q != p and self.segOk(conn, layer, q, p, rule.narrow, rule):
                    return q
                return None
            if it.kind == "via":
                c = it.pts[0]
                if c != p and self.segOk(conn, layer, c, p, rule.narrow, rule):
                    return c
                return None
        return None

    # ---- commit ------------------------------------------------------------------------
    def buildItems(self, conn, res, rid):
        rule = res["rule"]
        runs, vias = self.pathToGeometry(conn, res)
        items = []
        for ri, (layer, pts, wide) in enumerate(runs):
            if ri == 0:
                e = self.entryPoint(conn, layer, pts[0], res["srcItems"], rule)
                if e is not None:
                    pts = [e] + pts
                    wide = [False] + wide
            if ri == len(runs) - 1 and not res["terminalVia"]:
                e = self.entryPoint(conn, layer, pts[-1], res["dstItems"], rule)
                if e is not None:
                    pts = pts + [e]
                    wide = wide + [False]
            pts, widths = self.pullTight(conn, layer, pts, wide, rule)
            for p, q, wdt in zip(pts, pts[1:], widths):
                if p != q:
                    items.append(self.trackItem(conn.net, layer, p, q, wdt, rule.clearance, rid))
        for v in vias:
            items.append(self.viaItem(conn.net, v, rule, rid))
        if res["terminalVia"]:
            last = runs[-1][1][-1]
            items.append(self.viaItem(conn.net, last, rule, rid))
        return items

    def endpointOwners(self, conn, res):
        """Route ids / pad nodes the path starts and ends on."""
        sp = self.sp
        nodes, parents = [], set()
        for cell, pool in ((res["cells"][0], res["srcItems"]), (res["cells"][-1], res["dstItems"])):
            p = Point(sp.toMm(cell[1], cell[2]))
            best = None
            for it in pool:
                if cell[0] in it.layers and it.geom.buffer(sp.g).contains(p):
                    best = it
                    if it.kind == "pad":
                        break
            if best is None:
                continue
            nodes.append(self.nodeOf(best))
            if best.kind != "pad":
                parents.add(best.route)
        if res["terminalVia"]:
            nodes.append(("plane", conn.net))
        return nodes, parents

    def commit(self, conn, res, items):
        rid = self.nextRid
        self.nextRid += 1
        for it in items:
            it.route = rid
            self.sp.add(it)
            self.netItems.setdefault(it.net, set()).add(it.iid)
        nodes, parents = self.endpointOwners(conn, res)
        r = Route(rid, conn, [it.iid for it in items], nodes, parents)
        self.routes[rid] = r
        for p in parents:
            self.children.setdefault(p, set()).add(rid)
        conn.routed = True
        conn.rid = rid
        return r

    def rip(self, rid, ripped=None):
        """Remove a route and everything that hangs off it; returns the connections freed."""
        ripped = [] if ripped is None else ripped
        r = self.routes.pop(rid, None)
        if r is None:
            return ripped
        for iid in r.items:
            it = self.sp.remove(iid)
            self.netItems.get(it.net, set()).discard(iid)
        r.conn.routed = False
        r.conn.rid = 0
        ripped.append(r.conn)
        for c in list(self.children.pop(rid, ())):
            self.rip(c, ripped)
        for p in r.parents:
            self.children.get(p, set()).discard(rid)
        return ripped

    # ---- cleanup -----------------------------------------------------------------------
    def routeCost(self, rid):
        r = self.routes[rid]
        items = [self.sp.items[i] for i in r.items]
        length = sum(math.dist(*it.pts) for it in items if it.kind == "track")
        nVia = sum(1 for it in items if it.kind == "via")
        return length + nVia * self.k.viaMm, nVia

    def restore(self, old):
        """Put a ripped leaf route back exactly as it was."""
        ids = []
        for it in old.oldItems:
            it.stamped = []
            self.sp.add(it)
            self.netItems.setdefault(it.net, set()).add(it.iid)
            ids.append(it.iid)
        old.items = ids
        self.routes[old.rid] = old
        for p in old.parents:
            self.children.setdefault(p, set()).add(old.rid)
        old.conn.routed = True
        old.conn.rid = old.rid

    def improve(self, rid, layers, winPads, viaMm):
        """Rip a leaf route and search again with everything else in place; keep the new one
        only if it is shorter or uses fewer vias."""
        r = self.routes.get(rid)
        if r is None or self.children.get(rid):
            return False
        before, viasBefore = self.routeCost(rid)
        r.oldItems = [self.sp.items[i] for i in r.items]
        self.rip(rid)
        st, new = self.route(r.conn, layers, winPads, viaMm=viaMm)
        if st == "ok" and new is not None:
            after, viasAfter = self.routeCost(new.rid)
            if after < before - 0.3 or viasAfter < viasBefore:
                return True
            self.rip(new.rid)
        elif st == "ok":   # already connected some other way: nothing to put back
            return True
        self.restore(r)
        return False

    # ---- power copper ------------------------------------------------------------------
    def padUnder(self, net, layer, p):
        nid = self.netId[net]
        for it in self.sp.query([layer], (p[0], p[1], p[0], p[1])):
            if it.kind == "pad" and it.net == nid and it.geom.buffer(1e-3).contains(Point(p)):
                return it.pad
        return None

    def fatten(self, nets, widths=(2.0, 1.6, 1.3, 1.0, 0.8, 0.6)):
        """Power copper the way a person pours it with tracks: every power track widened to
        the widest that fits, so neighbouring tracks of the net overlap into one area. A
        track ending on a pad grows no wider than that pad, and none covers a GND pad's
        thermal spokes."""
        sp = self.sp
        grown = 0
        for r in list(self.routes.values()):
            if r.conn.net not in nets:
                continue
            for idx, iid in enumerate(r.items):
                it = sp.items[iid]
                if it.kind != "track":
                    continue
                layer = it.layers[0]
                cap = max(widths)
                for p in it.pts:
                    pad = self.padUnder(r.conn.net, layer, p)
                    if pad is not None:
                        minx, miny, maxx, maxy = pad.poly.bounds
                        cap = min(cap, max(it.width, maxx - minx, maxy - miny))
                for w in widths:
                    if w <= it.width + 1e-6 or w > cap + 1e-6:
                        continue
                    new = self.trackItem(r.conn.net, layer, it.pts[0], it.pts[1], w, it.clearance, r.rid)
                    if self.check(new):
                        continue
                    wwin, mask = sp.rasterMask(new.geom)
                    i0, j0, i1, j1 = wwin
                    if mask.size and (self.thermalCost[layer, i0:i1, j0:j1][mask] > 0).any():
                        continue
                    sp.remove(iid)
                    self.netItems[it.net].discard(iid)
                    sp.add(new)
                    self.netItems[new.net].add(new.iid)
                    r.items[idx] = new.iid
                    grown += 1
                    break
        return grown

    # ---- plane stitching ----------------------------------------------------------------
    def stitch(self, net="GND"):
        """GND via field on a square grid (snapped to the nearest legal spot near each node)."""
        sp, k = self.sp, self.k
        rule = self.rules[net]
        nid = self.netId[net]
        win = (0, 0, sp.H, sp.W)
        rv = rule.viaDia / 2
        m = k.margin
        ok = sp.foreignDist(nid, None, win, reach=0) >= rv + max(rule.clearance, 0.25) + m
        ok &= ~sp.viaKeep
        ok &= (edt(sp.smdPad != 0) * sp.g) >= rv + 0.1
        ok &= self.holeDist >= rule.viaDrill / 2 + self.m.minHoleToHole + m
        ok &= sp.maskDist(sp.viaAt, win, reach=0, pred=lambda a: a != 0) >= rule.viaDrill + 0.6
        conn = Conn(0, net, None, None, "stitch")
        p = k.stitchPitchMm
        minx, miny, maxx, maxy = self.m.outline.bounds
        added = 0
        snap = int(p / 3 / sp.g)
        y = miny + p / 2
        while y < maxy:
            x = minx + p / 2
            while x < maxx:
                i, j = sp.toCell(x, y)
                a0, b0 = max(i - snap, 0), max(j - snap, 0)
                sub = ok[a0:i + snap + 1, b0:j + snap + 1]
                if sub.any():
                    ii, jj = np.nonzero(sub)
                    d = (ii + a0 - i) ** 2 + (jj + b0 - j) ** 2
                    for idx in np.argsort(d)[:6]:
                        pt = sp.toMm(ii[idx] + a0, jj[idx] + b0)
                        rid = self.nextRid
                        via = self.viaItem(net, pt, rule, rid)
                        if not self.check(via):
                            self.nextRid += 1
                            via.route = rid
                            sp.add(via)
                            self.netItems.setdefault(nid, set()).add(via.iid)
                            self.routes[rid] = Route(rid, conn, [via.iid], [("plane", net)], set())
                            added += 1
                            break
                x += p
            y += p
        return added

    def viaSpacing(self, items):
        """Hole-to-hole between the vias of one new route (the search cannot see its own)."""
        vias = [it for it in items if it.kind == "via"]
        out = []
        for i, a in enumerate(vias):
            for b in vias[i + 1:]:
                d = math.dist(a.pts[0], b.pts[0]) - a.drill / 2 - b.drill / 2
                if d < self.m.minHoleToHole + 1e-4:
                    out.append((b.pts[0], a.drill / 2 + b.drill / 2 + self.m.minHoleToHole + 0.05))
        return out

    # ---- one connection ----------------------------------------------------------------
    def route(self, conn, layers, winPads=(2.0, 6.0, 25.0), negotiate=False, viaMm=None):
        """Route conn; returns ('ok', route) | ('fail', None) | ('conflict', victims)."""
        if self.connected(conn):
            conn.routed = True
            return "ok", None
        self.forbidVias = []
        for extra in (0.0, 0.03, 0.03):
            for wp in winPads:
                res = self.search(conn, layers, wp, negotiate=negotiate, extraMargin=extra, viaMm=viaMm)
                if res is not None:
                    break
            if res is None:
                return "fail", None
            t0 = time.perf_counter()
            rid = self.nextRid
            items = self.buildItems(conn, res, rid)
            bad = set()
            for it in items:
                for o in self.check(it):
                    bad.add(o)
            selfBad = self.viaSpacing(items)
            self.stats["commitS"] += time.perf_counter() - t0
            if selfBad:
                self.forbidVias += selfBad
                continue
            if not bad:
                return "ok", self.commit(conn, res, items)
            victims = {o.route for o in bad if o.kind in ("track", "via") and o.route}
            fixedHit = [o for o in bad if o.kind not in ("track", "via")]
            if negotiate and victims and not fixedHit:
                return "conflict", (res, items, victims)
            # raster error against fixed copper: retry with a wider margin
        return "fail", None
