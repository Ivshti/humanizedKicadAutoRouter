"""Router core: search, commit with exact verification, rip-up with dependency tracking.

A connection joins two copper components of one net (or a component and a plane).
The search runs on a cost raster built for that connection; the found path becomes
real geometry (tracks + vias), is pulled tight, and is verified exactly before it is
committed. Negotiation mode lets a search cross other routes at a price and rips the
routes it crossed; they go back on the queue.
"""
import ctypes
import math
import re
import time
from dataclasses import dataclass, field

import numpy as np
from shapely.geometry import LineString, Point, box

import native
from boardModel import B, F, IN1, IN2
from native import edt
from space import HARD, NLAYERS, Item, Space

# shapely's round shapes are polygons inside the true circle; scale them so the polygon
# lies outside it and every clearance it passes holds for the real round copper
CIRC = 1 / math.cos(math.pi / 64)


@dataclass
class Knobs:
    grid: float = 0.025
    margin: float = 0.02
    signalWidth: float = 0.2
    powerWidth: dict = field(default_factory=lambda: {"Power": 0.5, "Rail": 0.3})
    turn45Mm: float = 0.1
    turn90Mm: float = 0.5
    viaMm: float = 5.0
    otherLayerMult: float = 2.5
    neckMm: float = 1.0          # power tracks may neck down this close to their own pads
    escapeMm: float = 1.5        # body keep-outs do not apply this close to the route's own pads
    escapeViaMm: float = 4.0     # a bundle member's vias belong this close to its pads (or at
                                 # a planned layer change)
    farViaMult: float = 5.0      # a via anywhere else costs this many vias
    attractMult: float = 0.55    # bundle members pay this much less one pitch from a sibling
    hugMult: float = 0.8         # signals pay this much less one clearance from other wiring
    pairMult: float = 0.3        # a pair's second net pays this much less at the pair gap
    thermalPenalty: float = 2.0  # cost added on GND pads' thermal spoke corridors
    bodyMinAreaMm2: float = 40.0
    stitchPitchMm: float = 4.0
    ampRiseC: float = 20.0
    outerUm: float = 35.0
    ampMargin: float = 1.08
    hweight: float = 1.15
    minHeuristic: float = 0.3    # floor on the estimate's scale (search speed vs. optimality)
    negotiatePenalty: float = 6.0
    corridorTileMm: float = 2.0
    corridorOut: float = 3.0     # bundle members pay this much more outside their corridor
    corridorViaMm: float = 4.0   # what a layer change costs a corridor, per member
    againstCost: float = 0.6     # running along the diagonal that is the other layer's main
                                 # direction costs this much more per mm
    pairSkewMm: float = 0.5      # length mismatch allowed in a differential pair that the
                                 # design rules (.kicad_dru skew) say nothing about
    topDiagonal: str = "auto"    # the top layer's main diagonal: "/", "\\" or "auto" (planned)
    corridorShare: float = 0.75  # a corridor tile another group runs through, while it has room
    viaCleanupMult: float = 4.0  # the last cleanup pass prices vias this much higher
    joinMm: float = 6.0          # bundle members whose ends are this close share a corridor
    maxGroup: int = 6
    maxExpand: int = 30_000_000


# a power rail by name, for boards whose net classes do not say: a voltage (5V, 3V3,
# +3.3v, V1_8, V12) or a rail word (VCC, VDD, VBAT, VBUS, VIN, VSYS), but not the
# control lines named after a rail (VBUS_EN, PWR_FAIL, BAT_DET)
RAIL_NAME = re.compile(r"(^|[_+])(\d+(\.\d+)?V\d*|V\d+(_\d+)?|VCC\w*|VDD\w*|AVDD|DVDD|VBAT|VBUS\w*|"
                       r"VIN|VSYS|VREG|BAT_?P)($|_)", re.I)
RAIL_CONTROL = re.compile(r"(^|_)(EN|DET|SENSE|SNS|FB|PG|GOOD|OK|FAIL|BUT|KEY|ARM|OFF|ON|INT|CTRL|"
                          r"CC\d?|ID|ADC|MON|SEL)($|_)", re.I)


def railByName(net):
    leaf = net.rsplit("/", 1)[-1]
    return bool(RAIL_NAME.search(leaf)) and not RAIL_CONTROL.search(leaf)


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
    corridorVia: object = None  # (tiles, tiles) bool: where its group's plan changes layer
    groupRank: int = 0        # corridor group, in planning order
    groupPos: int = 0         # position across the group's width
    pairLeader: object = None # set on a pair's second net: the first net's connection
    pairMate: object = None   # both nets of a pair: the other one's connection


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
        self.planeNets = set(model.planes) or {"GND"}
        self.hasPowerClasses = any(c in self.k.powerWidth for c in model.netClass.values())
        self.rules = {n: self.ruleFor(n) for n in model.netClass}
        self.routes = {}
        self.nextRid = 1
        self.children = {}
        self.netItems = {}
        self.flowNets = set()      # power nets whose current is planned from flows
        self.hist = np.zeros((NLAYERS, self.sp.H, self.sp.W), np.float32)
        self.stats = {"searches": 0, "searchS": 0.0, "costS": 0.0, "commitS": 0.0}
        self.corridors = None      # Corridors planner, once the bundle phase starts
        # each outer layer's main diagonal (corridors.py axis numbering); the other diagonal
        # costs extra on it, so the two layers run their long wiring crosswise
        self.mainAxis = {F: 3, B: 1}
        self.loadFixed()

    # ---- setup ------------------------------------------------------------------------
    def powerClass(self, net):
        """The power width class of a net (a key of Knobs.powerWidth), or None for a signal.
        A net class named Power or Rail says so; on a board without them, a net in the
        default class whose name is a rail's is routed as a Rail."""
        cls = self.m.netClass.get(net, "Default")
        if cls in self.k.powerWidth:
            return cls
        if cls == "Default" and not self.hasPowerClasses and net not in self.planeNets \
                and railByName(net):
            return "Rail"
        return None

    def ruleFor(self, net):
        cls = self.m.netClass.get(net, "Default")
        nc = self.m.classes[cls]
        pc = self.powerClass(net)
        if pc:
            # power may neck to signal width at its pins (fine-pitch regulators)
            w, narrow = max(self.k.powerWidth[pc], nc.width), min(nc.width, self.k.signalWidth)
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
            geom = Point(x, y).buffer(r * CIRC, 16)
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
        for pad in self.m.nets.get("GND", []) if "GND" in self.planeNets else []:
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
        # a plane that does not cover the board: cells where a via reaches it
        self.planeCells = {}
        for net, fill in self.m.planes.items():
            if fill is None or net not in self.netId:
                continue
            cells = np.zeros((sp.H, sp.W), bool)
            rule = self.rules[net]
            win, mask = sp.rasterMask(fill.buffer(-(rule.viaDia / 2 + rule.clearance)))
            if mask.size:
                i0, j0, i1, j1 = win
                cells[i0:i1, j0:j1] |= mask
            self.planeCells[net] = cells

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
            if net in self.planeNets and not pad.smd and self.onPlane(net, pad.center):
                union(("pad", pad.pid), ("plane", net))
        for r in self.routes.values():
            if r.conn.net != net:
                continue
            for n in r.nodes:
                union(("route", r.rid), n)
        return find

    def onPlane(self, net, p):
        """Whether the net's plane has copper under point p."""
        fill = self.m.planes.get(net)
        return fill is None or fill.contains(Point(p))

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
        hug = self.hugMasks(conn, rule, win, layers) if conn.kind == "sig" else None
        pair = self.pairBand(conn, rule, win, layers)
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
            if hug is not None:
                own, other = hug
                c[own[li]] *= k.attractMult
                c[other[li] & ~own[li]] *= k.hugMult
            if pair is not None:
                c[pair[0][li]] *= k.pairMult
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
        # one cell of raster slack: a pad corner can rasterise one cell short
        viaok &= sp.maskDist(sp.smdPad, win, pred=lambda a: a == nid) >= rv + 0.03 + g * math.sqrt(2) + extraMargin
        viaok &= self.holeDist[i0:i1, j0:j1] >= rule.viaDrill / 2 + self.m.minHoleToHole + m
        viaok &= sp.maskDist(sp.viaAt, win, pred=lambda a: a == nid) >= \
            rule.viaDrill + self.m.minHoleToHole + m
        viaok = viaok.astype(np.uint8)
        # a bundle member or pair wants its vias at its escapes and planned layer changes
        # (a pair: beside the first net's vias); anywhere else a via costs more (marked 2)
        want = None
        if pair is not None:
            want = (padDist < k.escapeViaMm) | pair[1]
        elif conn.corridorVia is not None:
            want = (padDist < k.escapeViaMm) | self.corridors.fine(conn.corridorVia, None, win)
        if want is not None:
            viaok[(viaok == 1) & ~want] = 2
        for (p, r) in getattr(self, "forbidVias", ()):
            ys, xs = sp.cellCenters(win)
            viaok[(xs - p[0]) ** 2 + (ys - p[1]) ** 2 < r * r] = 0
        return cost, viaok, wideOk, None

    def pairBand(self, conn, rule, win, layers):
        """For a differential pair net whose partner is already routed: cells at the pair
        gap from the partner's tracks (per layer), and cells within reach of its vias.
        Either net may be the one routed second (push and shove can rip either)."""
        lead = conn.pairMate
        if lead is None or not lead.routed or lead.rid not in self.routes:
            return None
        sp, g, k = self.sp, self.k.grid, self.k
        i0, j0, i1, j1 = win
        h, w = i1 - i0, j1 - j0
        items = [sp.items[i] for i in self.routes[lead.rid].items]
        band = np.zeros((len(layers), h, w), bool)
        gap = rule.width / 2 + self.m.classes[rule.cls].pairGap + k.margin
        for li, l in enumerate(layers):
            m = np.zeros((h, w), bool)
            for it in items:
                if it.kind == "track" and l in it.layers:
                    self.paint(m, win, it)
            if m.any():
                d = edt(m) * g
                band[li] = (d >= gap) & (d <= gap + 2 * g)
        nearVia = np.zeros((h, w), bool)
        vias = [it for it in items if it.kind == "via"]
        if vias:
            ys, xs = sp.cellCenters(win)
            for v in vias:
                nearVia |= (xs - v.pts[0][0]) ** 2 + (ys - v.pts[0][1]) ** 2 <= 1.5 ** 2
        return band, nearVia

    def hugMasks(self, conn, rule, win, layers):
        """Cells where this track would run exactly one clearance beside existing signal or
        power copper on the same layer: beside its own bundle (own) and beside anything
        else (other). Packing tracks together keeps the space left over in one piece, so
        the ground pour can fill it."""
        sp, g, k = self.sp, self.k.grid, self.k
        i0, j0, i1, j1 = win
        h, w = i1 - i0, j1 - j0
        x0, y0 = sp.toMm(i0, j0)
        x1, y1 = sp.toMm(i1, j1)
        own = np.zeros((len(layers), h, w), bool)
        other = np.zeros((len(layers), h, w), bool)
        nid = self.netId[conn.net]
        plane = {self.netId[n] for n in self.planeNets}
        band = rule.width / 2 + rule.clearance + k.margin
        for li, l in enumerate(layers):
            mOwn = np.zeros((h, w), bool)
            mAll = np.zeros((h, w), bool)
            for it in sp.query([l], (x0, y0, x1, y1)):
                if it.kind != "track" or it.net == nid or it.net in plane:
                    continue
                self.paint(mAll, win, it)
                r = self.routes.get(it.route)
                if conn.bundle and r is not None and r.conn.bundle == conn.bundle:
                    self.paint(mOwn, win, it)
            for mask, out in ((mOwn, own), (mAll, other)):
                if mask.any():
                    d = edt(mask) * g
                    out[li] = (d >= band) & (d <= band + 2 * g)
        return own, other

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
        if tvia is not None and conn.net in self.planeCells:
            i0, j0, i1, j1 = win
            tvia &= self.planeCells[conn.net][i0:i1, j0:j1]
        vCost = (viaMm if viaMm is not None else k.viaMm) / k.grid
        ys, xs = np.nonzero(dst.any(axis=0)) if dst.any() else (np.array([0]), np.array([0]))
        tx0, tx1, ty0, ty1 = int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())
        maxPath = h * w * 2 + 16
        outPath = np.empty(min(maxPath, 4_000_000), np.int32)
        expanded = ctypes.c_int64(0)
        cost = np.ascontiguousarray(cost)
        # the distance estimate must not exceed what the cheapest cells cost, or the
        # search runs straight past discounted (hugging, pair-following) paths
        free = cost[cost > 0]
        hweight = k.hweight * max(min(1.0, float(free.min()) if free.size else 1.0), k.minHeuristic)
        layerDir = self.layerDir(layers) if conn.kind in ("sig", "pwr") else None
        t0 = time.perf_counter()
        n = self.kernel.astar(Ls, h, w, cost.ctypes.data, viaok.ctypes.data, ctypes.c_float(vCost),
                              tgt.ctypes.data, tvia.ctypes.data if tvia is not None else None,
                              srcIdx.ctypes.data, len(srcIdx), ctypes.c_float(k.turn45Mm / k.grid),
                              ctypes.c_float(k.turn90Mm / k.grid),
                              ctypes.c_float(0.0 if toPlane else hweight),
                              tx0, ty0, tx1, ty1, outPath.ctypes.data, len(outPath),
                              ctypes.c_int64(k.maxExpand), ctypes.byref(expanded), None,
                              ctypes.c_float(vCost * k.farViaMult),
                              layerDir.ctypes.data if layerDir is not None else None)
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

    def layerDir(self, layers):
        """Extra cost per unit length along each axis, per search layer: the diagonal that
        is the other outer layer's main direction."""
        out = np.zeros((len(layers), 4), np.float32)
        for li, l in enumerate(layers):
            if l in self.mainAxis:
                out[li, 4 - self.mainAxis[l]] = self.k.againstCost
        return out

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
        r = width / 2 * CIRC
        geom = LineString([p1, p2]).buffer(r, 16) if p1 != p2 else Point(p1).buffer(r, 16)
        return Item(0, self.netId[net], "track", (layer,), geom, clearance, width=width,
                    pts=(p1, p2), route=rid)

    def viaItem(self, net, p, rule, rid):
        geom = Point(p).buffer(rule.viaDia / 2 * CIRC, 16)
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

    # ---- differential pairs -------------------------------------------------------------
    def coupling(self, c1, c2, reach=0.35):
        """Share of c1's track length that runs beside c2's tracks (same layer, edge gap
        within reach)."""
        if c1.rid not in self.routes or c2.rid not in self.routes:
            return 0.0
        t1 = [self.sp.items[i] for i in self.routes[c1.rid].items if self.sp.items[i].kind == "track"]
        t2 = [self.sp.items[i] for i in self.routes[c2.rid].items if self.sp.items[i].kind == "track"]
        total = near = 0.0
        for a in t1:
            seg = LineString(a.pts)
            n = max(2, int(seg.length / 0.2) + 1)
            for i in range(n):
                p = seg.interpolate(i / (n - 1), normalized=True)
                total += seg.length / n
                if any(b.layers == a.layers and LineString(b.pts).distance(p) <= (a.width + b.width) / 2 + reach
                       for b in t2):
                    near += seg.length / n
        return near / total if total else 0.0

    def repairPair(self, lead, follow, layers, winPads, viaMm):
        """Rip both nets of a loosely coupled pair and route them again together; keep the
        new pair only if it couples better."""
        old = [self.routes.get(c.rid) for c in (lead, follow)]
        if any(r is None or self.children.get(r.rid) for r in old):
            return False
        before = self.coupling(lead, follow)
        for r in old:
            r.oldItems = [self.sp.items[i] for i in r.items]
            self.rip(r.rid)
        new = []
        for c in (lead, follow):
            st, rt = self.route(c, layers, winPads, viaMm=viaMm)
            if st != "ok" or rt is None:
                break
            new.append(rt)
        if len(new) == 2 and self.coupling(lead, follow) > before + 0.1:
            return True
        for rt in new:
            self.rip(rt.rid)
        for r in old:
            self.restore(r)
        return False

    # ---- length matching ----------------------------------------------------------------
    def netLength(self, net):
        nid = self.netId[net]
        return sum(math.dist(*self.sp.items[i].pts) for i in self.netItems.get(nid, ())
                   if self.sp.items[i].kind == "track")

    @staticmethod
    def serpentine(p, q, w, side, extra, amax, pitch):
        """The run p->q with rectangular bumps (45 deg corners) to one side that add up to
        `extra` mm of length, at most `amax` high. Returns (points, mm added), or None when
        the run is too short or the extra too small for one bump."""
        L = math.dist(p, q)
        if L < 1e-6:
            return None
        u = ((q[0] - p[0]) / L, (q[1] - p[1]) / L)
        n = (-u[1] * side, u[0] * side)
        c0 = w                                  # corner chamfer
        lost = (8 - 4 * math.sqrt(2)) * c0      # what the chamfers take off 2 * amplitude
        adv = 4 * c0 + pitch                    # run length one bump spans
        margin = max(2 * w, 0.3)
        fit = int((L - 2 * margin + pitch) // (adv + pitch))
        if fit < 1 or extra < 4 * c0 - lost or amax < 2 * c0:
            return None
        nb = min(fit, math.ceil(extra / (2 * amax - lost)))
        A = max(2 * c0, min(amax, (extra / nb + lost) / 2))
        gain = nb * (2 * A - lost)
        span = nb * adv + (nb - 1) * pitch
        x = (L - span) / 2
        at = lambda a, b: (p[0] + u[0] * a + n[0] * b, p[1] + u[1] * a + n[1] * b)
        pts = [p]
        for _ in range(nb):
            pts += [at(x, 0), at(x + c0, c0), at(x + c0, A - c0), at(x + 2 * c0, A),
                    at(x + 2 * c0 + pitch, A), at(x + 3 * c0 + pitch, A - c0),
                    at(x + 3 * c0 + pitch, c0), at(x + adv, 0)]
            x += adv + pitch
        pts.append(q)
        return pts, gain

    def lengthen(self, conn, extra, amax):
        """Add about `extra` mm to a routed connection with serpentines on its longest
        straight runs, each one verified exactly before it replaces the run. Returns the
        length added."""
        r = self.routes.get(conn.rid)
        if r is None:
            return 0.0
        added = 0.0
        tracks = sorted((self.sp.items[i] for i in r.items if self.sp.items[i].kind == "track"),
                        key=lambda it: -math.dist(*it.pts))
        for it in tracks:
            want = extra - added
            if want < 0.05:
                break
            pitch = max(3 * it.width, it.width + it.clearance)
            done = False
            for amp in (amax, amax * 0.6, amax * 0.35):
                for side in (1, -1):
                    res = self.serpentine(*it.pts, it.width, side, want, amp, pitch)
                    if res is None:
                        continue
                    pts, gain = res
                    layer = it.layers[0]
                    new = [self.trackItem(conn.net, layer, a, b, it.width, it.clearance, r.rid)
                           for a, b in zip(pts, pts[1:]) if a != b]
                    if any(self.check(t, ignore={it.iid}) or self.underBody(t, layer) for t in new):
                        continue
                    self.sp.remove(it.iid)
                    self.netItems[it.net].discard(it.iid)
                    r.items.remove(it.iid)
                    for t in new:
                        self.sp.add(t)
                        self.netItems[t.net].add(t.iid)
                        r.items.append(t.iid)
                    added += gain
                    done = True
                    break
                if done:
                    break
        return added

    def underBody(self, item, layer):
        """Copper of a route that would sit under a module or antenna body."""
        win, mask = self.sp.rasterMask(item.geom)
        i0, j0, i1, j1 = win
        return bool((self.bodyKeep[layer, i0:i1, j0:j1] & mask).any()) if mask.size else False

    def matchPairs(self, partners, budgetFor, log):
        """Differential pairs whose two nets differ in length by more than their skew
        budget: the shorter net is lengthened with serpentines to match."""
        for a, b in sorted(partners.items()):
            if a > b or a not in self.netId or b not in self.netId:
                continue
            budget = budgetFor(a)
            la, lb = self.netLength(a), self.netLength(b)
            skew = abs(la - lb)
            if skew <= budget * 0.5:
                continue
            short = a if la < lb else b
            conns = sorted({id(rt.conn): rt.conn for rt in self.routes.values()
                            if rt.conn.net == short and rt.conn.b is not None}.values(),
                           key=lambda c: -self.routeCost(c.rid)[0])
            added = 0.0
            for c in conns:
                if skew - added <= budget * 0.25:
                    break
                w = self.ruleForConn(c).width
                added += self.lengthen(c, skew - added, max(1.0, 5 * w))
            after = abs(self.netLength(a) - self.netLength(b))
            log(f"length match {a}/{b}: skew {skew:.2f} -> {after:.2f} mm (budget {budget:.2f})")

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
    def stitchMask(self, net="GND"):
        """Cells where a plane via may go, judged on the current board."""
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
        return ok

    def placePlaneVia(self, pts, net="GND"):
        """Put a plane via at the first of pts that passes the exact check."""
        sp = self.sp
        rule = self.rules[net]
        nid = self.netId[net]
        for pt in pts:
            rid = self.nextRid
            via = self.viaItem(net, pt, rule, rid)
            if self.check(via):
                continue
            self.nextRid += 1
            via.route = rid
            sp.add(via)
            self.netItems.setdefault(nid, set()).add(via.iid)
            self.routes[rid] = Route(rid, Conn(0, net, None, None, "stitch"), [via.iid],
                                     [("plane", net)], set())
            return pt
        return None

    def nearestOk(self, ok, x, y, radiusMm, n=6, minMm=0.0):
        """Up to n legal cells nearest to (x, y), within radiusMm (and at least minMm)."""
        sp = self.sp
        i, j = sp.toCell(x, y)
        r = int(radiusMm / sp.g)
        a0, b0 = max(i - r, 0), max(j - r, 0)
        sub = ok[a0:i + r + 1, b0:j + r + 1]
        if not sub.any():
            return []
        ii, jj = np.nonzero(sub)
        d = (ii + a0 - i) ** 2 + (jj + b0 - j) ** 2
        keep = (d <= r * r) & (d >= (minMm / sp.g) ** 2)
        ii, jj, d = ii[keep], jj[keep], d[keep]
        return [sp.toMm(ii[x] + a0, jj[x] + b0) for x in np.argsort(d)[:n]]

    def returnVias(self, net="GND", reach=1.5):
        """A plane via next to every signal via that has none within reach: the return
        current of a track that changes layers changes planes there too."""
        sp = self.sp
        ok = self.stitchMask(net)
        nid = self.netId[net]
        gnd = [it.pts[0] for it in sp.items.values() if it.kind == "via" and it.net == nid]
        added = 0
        for it in list(sp.items.values()):
            if it.kind != "via" or it.net == nid:
                continue
            x, y = it.pts[0]
            if any(math.dist(g, (x, y)) <= reach for g in gnd):
                continue
            pt = self.placePlaneVia(self.nearestOk(ok, x, y, reach, minMm=0.0), net)
            if pt is not None:
                gnd.append(pt)
                i, j = sp.toCell(*pt)
                r = int(0.9 / sp.g)
                ok[max(i - r, 0):i + r + 1, max(j - r, 0):j + r + 1] = False
                added += 1
        return added

    def stitch(self, net="GND"):
        """GND via field on a square grid (snapped to the nearest legal spot near each node)."""
        sp, k = self.sp, self.k
        ok = self.stitchMask(net)
        p = k.stitchPitchMm
        minx, miny, maxx, maxy = self.m.outline.bounds
        added = 0
        y = miny + p / 2
        while y < maxy:
            x = minx + p / 2
            while x < maxx:
                if self.placePlaneVia(self.nearestOk(ok, x, y, p / 3), net) is not None:
                    added += 1
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
