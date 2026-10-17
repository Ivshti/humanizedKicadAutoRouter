"""Router core: search, commit with exact verification, rip-up with dependency tracking.

A connection joins two copper components of one net (or a component and a plane).
The search runs on a cost raster built for that connection; the found path becomes
real geometry (tracks + vias), is pulled tight, and is verified exactly before it is
committed. Negotiation mode lets a search cross other routes at a price and rips the
routes it crossed; they go back on the queue.
"""
import ctypes
import math
import os
import re
import time
from dataclasses import dataclass, field

import numpy as np
from shapely.geometry import LineString, Point, box
from shapely.ops import unary_union

import native
from boardModel import B, F, KICAD_LAYERS, LAYER_NAMES, PHYSICAL, defaultPlaneLayers
from native import edt
from space import HARD, Item, Space

# shapely's round shapes are polygons inside the true circle; scale them so the polygon
# lies outside it and every clearance it passes holds for the real round copper
CIRC = 1 / math.cos(math.pi / 64)


@dataclass
class Knobs:
    grid: float = 0.025
    margin: float = 0.02
    signalWidth: float = 0.0     # signal track width; 0 = the board's Default net class width
    seed: int = 0                # 0: the routing order as planned; another: each connection's
                                 # place in it shaken a little (reproducibly), to try other orders
    powerWidth: dict = field(default_factory=lambda: {"Power": 0.5, "Rail": 0.3})
    turn45Mm: float = 0.1
    turn90Mm: float = 0.5
    viaMm: float = 5.0
    otherLayerMult: float = 2.5
    neckMm: float = 0.5          # a power track may neck down only this close to a pad of its
                                 # own that is narrower than the track; elsewhere it keeps its
                                 # width or the route fails
    escapeMm: float = 1.5        # body keep-outs do not apply this close to the route's own pads
    escapeViaMm: float = 4.0     # a bundle member's vias belong this close to its pads (or at
                                 # a planned layer change)
    farViaMult: float = 5.0      # a via anywhere else costs this many vias
    attractMult: float = 0.55    # bundle members pay this much less one pitch from a sibling
    hugMult: float = 0.8         # signals pay this much less one clearance from other wiring
    pairMult: float = 0.3        # a pair's second net pays this much less at the pair gap
    thermalPenalty: float = 2.0  # cost added on GND pads' thermal spoke corridors
    thermalHard: bool = True     # other nets may not cross those corridors at all (except
                                 # within 0.3 mm of their own pads, to get out of a pin row)
    fanoutViaGapMm: float = 0.25 # bare board between a GND fanout via's ring and any pad edge
    fanoutInsideMult: float = 3.0  # a fanout pays this much more where its via does not belong:
                                 # inside a small part's outline (via on the far side), or just
                                 # outside a chip's pin ring (via inside the ring, under the body)
    fanoutParts: str = "caps-ics"  # which pads of a net poured on the outer layers (GND) get
                                 # their own via: "all"; "caps-ics" = capacitors and multi-pin
                                 # parts other than connectors, the rest take the pour and get a
                                 # via only if the pour misses them; "none" = every pad takes
                                 # the pour, stitching feeds it and a pad it misses is reported.
                                 # Pads of plane nets with no outer pour always get their via.
    blockLayer: str = "free"     # "own": wiring inside a block stays on its parts' layer, the
                                 # other outer layer is left to the bundles (relaxed only for a
                                 # connection that cannot be routed that way)
    bodyMinAreaMm2: float = 40.0
    stitchPitchMm: float = 4.0
    stitch: bool = True          # stitch the outer ground pours to the plane after routing
    stitchPadGapMm: float = 0.5  # bare board between a stitching or return via and any pad
    stitchPerIsland: int = 2     # vias (or through-hole pads) every piece of outer pour gets
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
    pairSkewMm: float = 1.5      # length mismatch allowed in a differential pair that the
                                 # design rules (.kicad_dru skew) say nothing about
    topDiagonal: str = "auto"    # the top layer's main diagonal: "/", "\\" or "auto" (planned)
    corridorShare: float = 0.75  # a corridor tile another group runs through, while it has room
    viaCleanupMult: float = 4.0  # the last cleanup pass prices vias this much higher
    straightenMult: float = 4.0  # the straightening pass prices corners this much higher
    pairStartMm: float = 1.2     # a pair may start this far from the midpoint of its two pads
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
    pairTried: bool = False   # the pair was already tried together (see Router.routePair)
    deferred: bool = False    # a GND fanout left to the pour unless the pour misses the pad
    layerLock: int = -1       # the only layer this connection may use (-1: any)
    viaGapMm: float = -1.0    # a fanout's via-to-pad gap when not fanoutViaGapMm (-1)
    thin: bool = False        # a plane fanout that did not fit at its pad's width: signal width


@dataclass
class Route:
    rid: int
    conn: Conn
    items: list
    nodes: list               # union-find nodes this route joins
    parents: set
    paired: bool = False      # laid together with its differential partner


class Router:
    def __init__(self, model, knobs=None, log=print):
        self.m = model
        self.k = knobs or Knobs()
        self.log = log
        self.kernel = native.kernel
        dc = model.classes["Default"].clearance
        self.NL = len(KICAD_LAYERS)
        self.allLayers = tuple(range(self.NL))
        self.sp = Space(model.outline, self.k.grid, self.k.margin, model.edgeClearance, dc, self.NL)
        self.sp.defaultClearance = dc
        self.netId = {n: i + 1 for i, n in enumerate(sorted(model.netClass))}
        self.netName = {i: n for n, i in self.netId.items()}
        self.planeNets = (set(model.planes) or {"GND"}) & set(self.netId)   # nets the board has
        self.hasPowerClasses = any(c in self.k.powerWidth for c in model.netClass.values())
        self.rules = {n: self.ruleFor(n) for n in model.netClass}
        self.routes = {}
        self.nextRid = 1
        self.children = {}
        self.netItems = {}
        self.flowNets = set()      # power nets whose current is planned from flows
        self.hist = np.zeros((self.NL, self.sp.H, self.sp.W), np.float32)
        self.stats = {"searches": 0, "searchS": 0.0, "costS": 0.0, "commitS": 0.0}
        self.corridors = None      # Corridors planner, once the bundle phase starts
        # the stack: inner layers that hold a plane (the board's own zones, else the default
        # stack) stay planes, every other layer carries signals
        planeLayers = getattr(model, "planeLayers", None)
        if planeLayers is None:
            planeLayers = {l: {"GND"} for l in defaultPlaneLayers(self.NL)}
        self.planeLayers = planeLayers
        self.signalLayers = [l for l in PHYSICAL if l not in planeLayers]   # stack order
        # each signal layer's main direction (corridors.py axis numbering): the outer layers
        # run opposite diagonals, the inner signal layers alternate horizontal and vertical
        # starting along the board's long side; running along the opposite axis costs extra
        self.mainAxis = {F: 3, B: 1}
        minx, miny, maxx, maxy = model.outline.bounds
        axis = 0 if maxx - minx >= maxy - miny else 2
        for l in self.signalLayers:
            if l not in (F, B):
                self.mainAxis[l] = axis
                axis = 2 - axis
        self.asForeign = False     # while True, new track items belong to no net (see trackItem)
        self.entryWide = False     # set by entryLegs: its legs are checked at the full width
        self.pairCrossed = False   # set by layPair: the last centreline would cross the pair
        self.pairLegFail = None    # set by layPair: the centreline end no pad's legs reached
        self.existing = {}         # rid (< 0) -> (net, nodes): copper that was on the board
        self.existingItems = {}    # rid (< 0) -> its items
        self.softPieces = set()    # rids of existing pieces an earlier router run drew
        self.droppedUuids = set()  # board items of soft pieces ripped this run
        self.allConns = []         # every planned connection (a ripped soft piece's are re-queued)
        self.padsOfRef = {}
        for pad in model.pads:
            self.padsOfRef.setdefault(pad.ref, []).append(pad)
        self.hulls = {}
        self.ruleAreas = np.zeros((self.sp.H, self.sp.W), bool)   # cells within any rule area
        # nets the finished board pours on both outer layers (finishBoard): their pads
        # can be left to the pour
        self.outerPourNets = {"GND"} & self.planeNets
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

    def signalWidth(self):
        return self.k.signalWidth or self.m.classes["Default"].width

    def ruleFor(self, net):
        cls = self.m.netClass.get(net, "Default")
        nc = self.m.classes[cls]
        pc = self.powerClass(net)
        sig = self.signalWidth()
        if pc:
            # power may neck to signal width at its pins (fine-pitch regulators)
            w, narrow = max(self.k.powerWidth[pc], nc.width), min(nc.width, sig)
        elif cls == "Default":
            w, narrow = sig, nc.width
        else:
            w = narrow = nc.width
        return NetRule(w, min(narrow, w), nc.clearance, nc.viaDia, nc.viaDrill, cls)

    def ruleForConn(self, conn):
        """Net rule, widened to carry the connection's current (IPC-2221, outer copper) and
        allowed to neck to the size of the pads it lands on."""
        base = self.rules[conn.net]
        if conn.b is None and conn.kind == "gnd" and not conn.thin \
                and conn.net not in self.outerPourNets:
            # a fanout to a supply plane is a short power link: the net's power width, as
            # wide as its pad's narrow side allows (a 0.3 mm pin gets 0.3, not the signal
            # width). Ground fanouts stay thin: there are many of them, in the densest
            # places, and the pour around them carries the current anyway
            pw = base.width if self.powerClass(conn.net) else self.k.powerWidth["Rail"]
            w = max(base.narrow, min(pw, padMinDim(conn.a)))
            return NetRule(w, w, base.clearance, base.viaDia, base.viaDrill, base.cls)
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
            it = Item(0, hid, "hole", self.allLayers, geom, 0.0, pts=((x, y),), drill=2 * r)
            sp.add(it)
            win, mask = sp.rasterMask(geom)
            if mask.size:
                i0, j0, i1, j1 = win
                sp.holeMask[i0:i1, j0:j1] |= mask
            if hc > 0 and hid == HARD:
                win, mask = sp.rasterMask(geom, hc)
                i0, j0, i1, j1 = win
                for l in range(self.NL):
                    sp.occ[l, i0:i1, j0:j1][mask & (sp.occ[l, i0:i1, j0:j1] == 0)] = HARD
                    sp.fixed[l, i0:i1, j0:j1][mask & (sp.fixed[l, i0:i1, j0:j1] == 0)] = HARD
        edge = Item(0, HARD, "edge", self.allLayers, self.m.outline.boundary, 0.0)
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
            # stitching stays out of every rule area, whatever it forbids (a "no parts"
            # area over printed artwork is no place for a via either)
            win, mask = sp.rasterMask(ko.poly, 0.45 + self.k.margin)
            if mask.size:
                i0, j0, i1, j1 = win
                self.ruleAreas[i0:i1, j0:j1] |= mask
        self.loadStrategyRasters()

    def loadStrategyRasters(self):
        """Human conventions as rasters: no foreign copper under module/antenna bodies,
        keep GND thermal spokes free, and a static distance-to-hole field."""
        sp, k = self.sp, self.k
        self.bodyKeep = np.zeros((self.NL, sp.H, sp.W), bool)
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
        self.thermalCost = np.zeros((self.NL, sp.H, sp.W), np.float32)
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
        padAll = np.zeros((sp.H, sp.W), bool)
        for pad in self.m.pads:
            if F in pad.layers or B in pad.layers:
                win, mask = sp.rasterMask(pad.poly)
                if mask.size:
                    i0, j0, i1, j1 = win
                    padAll[i0:i1, j0:j1] |= mask
        self.padEdgeDist = (edt(padAll) * sp.g).astype(np.float32)
        # a plane that does not cover the board: cells where a via reaches it
        self.setPlaneFills(self.m.planes)

    def setPlaneFills(self, fills):
        """{net: inner fill of its plane, None for a whole-board plane}: where a via of the
        net reaches its plane (inside the fill, by the via's radius plus clearance)."""
        sp = self.sp
        self.planeCells = {}
        for net, fill in fills.items():
            if fill is None or net not in self.netId:
                continue
            cells = np.zeros((sp.H, sp.W), bool)
            rule = self.rules[net]
            room = fill.buffer(-(rule.viaDia / 2 + rule.clearance))
            win, mask = sp.rasterMask(room) if not room.is_empty else (None, np.zeros(0))
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
        for rid, (n, nodes) in self.existing.items():
            if n == net:
                find(("route", rid))
                for nd in nodes:
                    union(("route", rid), nd)
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
    def buildCost(self, conn, rule, win, layers, srcItems, dstItems, negotiate, extraMargin,
                  ownNets=None, ownPads=None):
        sp, k, g = self.sp, self.k, self.k.grid
        nid = self.netId[conn.net]
        nets = tuple(ownNets) if ownNets else (nid,)
        plane = conn.net in self.planeNets
        i0, j0, i1, j1 = win
        h, w = i1 - i0, j1 - j0
        m = k.margin + extraMargin
        rW = rule.width / 2 + rule.clearance + m
        rN = rule.narrow / 2 + rule.clearance + m
        endPads = [it for it in srcItems + dstItems if it.kind == "pad"]
        if ownPads is None:
            ownPads = [it for it in endPads if it.pad is conn.a or it.pad is conn.b]
        # escape zone: close to the pads this connection starts/ends on, body keep-outs
        # do not apply (a pin under a module must get out) and power may neck down
        own = np.zeros((h, w), bool)
        for it in ownPads:
            self.paint(own, win, it)
        padDist = edt(own) * g if own.any() else np.full((h, w), 1e3)
        escape = padDist < k.escapeMm
        # a power track necks only to land on a pad narrower than itself
        narrowPads = [it for it in ownPads if it.kind == "pad" and padMinDim(it.pad) < rule.width - 1e-6]
        neckZone = np.zeros((h, w), bool)
        if rule.narrow < rule.width and narrowPads and k.neckMm > 0:
            m2 = np.zeros((h, w), bool)
            for it in narrowPads:
                self.paint(m2, win, it)
            neckZone = edt(m2) * g < k.neckMm
        hug = self.hugMasks(conn, rule, win, layers) if conn.kind == "sig" else None
        pair = self.pairBand(conn, rule, win, layers)
        cost = np.empty((len(layers), h, w), np.float32)
        wideOk = np.empty((len(layers), h, w), bool)
        for li, l in enumerate(layers):
            dAll = sp.foreignDist(nid, l, win, nets=nets)
            wideOk[li] = dAll >= rW
            ok = wideOk[li] | (neckZone & (dAll >= rN))
            soft = None
            if negotiate:
                # routes in the way will be ripped: judge width against fixed copper only
                dFix = sp.foreignDist(nid, l, win, fixedOnly=True, nets=nets)
                okFix = (dFix >= rW) | (neckZone & (dFix >= rN))
                soft = okFix & ~ok
                ok = okFix
                wideOk[li] = dFix >= rW
            ok &= ~sp.trackKeep[l, i0:i1, j0:j1]
            if not plane:
                ok &= ~(self.bodyKeep[l, i0:i1, j0:j1] & ~escape)
                if k.thermalHard:
                    ok &= ~(self.thermalCost[l, i0:i1, j0:j1] > 0) | (padDist < 0.3)
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
            if conn.corridor is not None and l in self.corridors.index:
                inside = self.corridors.fine(conn.corridor, self.corridors.index[l], win)
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
        dAny = sp.foreignDist(nid, None, win, fixedOnly=negotiate, nets=nets)
        viaok = dAny >= rv + rule.clearance + m
        viaok &= ~sp.viaKeep[i0:i1, j0:j1]
        if not plane:
            viaok &= ~(self.bodyViaKeep[i0:i1, j0:j1] & ~escape)
        # one cell of raster slack: a pad corner can rasterise one cell short
        viaok &= sp.maskDist(sp.smdPad, win, pred=lambda a: np.isin(a, nets)) >= rv + 0.03 + g * math.sqrt(2) + extraMargin
        viaok &= self.holeDist[i0:i1, j0:j1] >= rule.viaDrill / 2 + self.m.minHoleToHole + m
        viaok &= sp.maskDist(sp.viaAt, win, pred=lambda a: np.isin(a, nets)) >= \
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
            else:  # land on a pad: anywhere on its copper, a little in from the edge, so an
                # end on the pad's rim is inside KiCad's outline too (ours is rounded outward)
                land = item.geom.buffer(-0.03)
                win, mask = sp.rasterMask(land) if not land.is_empty else (None, np.zeros(0, bool))
                cache[key] = (win, mask) if mask.size and mask.any() else self.cachedMask(item, "copper")
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
        srcIdx = np.flatnonzero(src).astype(np.int32)
        tgt = dst.astype(np.uint8)
        tvia = viaok.copy() if toPlane else None
        i0, j0, i1, j1 = win
        if tvia is not None and conn.net in self.planeCells:
            tvia &= self.planeCells[conn.net][i0:i1, j0:j1]
        if tvia is not None and conn.kind == "gnd":
            # a fanout via keeps fanoutViaGapMm of bare board to every pad, its own too,
            # and lands on the side away from the rest of its part
            gap = conn.viaGapMm if conn.viaGapMm >= 0 else k.fanoutViaGapMm
            tvia &= self.padEdgeDist[i0:i1, j0:j1] >= rule.viaDia / 2 + gap + k.grid
            avoid = self.fanoutAvoid(conn.a, win)
            if avoid is not None:
                sub = cost[:, avoid]
                cost[:, avoid] = np.where(sub > 0, sub * k.fanoutInsideMult, sub)
        cells, terminalVia = self.runKernel(conn, cost, viaok, wideOk, srcIdx, tgt, tvia, win, layers,
                                            viaMm, toPlane)
        if cells is None:
            return None
        return dict(cells=cells, terminalVia=terminalVia, srcItems=srcItems, dstItems=dstItems,
                    rule=rule, win=win)

    RING_PADS = 8   # a part with more pads than this is a chip with a ring of pins

    def fanoutAvoid(self, pad, win):
        """Window cells a ground fanout from this pad should stay out of, so its via lands
        where a person puts it: for a small part (passive, transistor) the part's own
        outline, so the via goes on the far side from its other pads; for a chip with a
        ring of pins, the band just outside the ring, where the wiring to its other pins
        arrives, so the via goes inside the ring, under the body. The pad itself is never
        avoided. None for a one-pad part."""
        pads = self.padsOfRef.get(pad.ref, [])
        if len(pads) < 2:
            return None
        if pad.ref not in self.hulls:
            self.hulls[pad.ref] = unary_union([q.poly for q in pads]).convex_hull
        hull = self.hulls[pad.ref]
        area = hull.buffer(self.k.escapeMm).difference(hull) if self.isRingChip(pad) else hull
        geom = area.difference(pad.poly.buffer(self.k.grid))
        i0, j0, i1, j1 = win
        out = np.zeros((i1 - i0, j1 - j0), bool)
        wwin, mask = self.sp.rasterMask(geom)
        self.blit(out, win, wwin, mask)
        return out

    def runKernel(self, conn, cost, viaok, wideOk, srcIdx, tgt, tvia, win, layers, viaMm, toPlane):
        """The A* call; returns (cells, terminalVia) or (None, False)."""
        k = self.k
        Ls, h, w = cost.shape
        vCost = (viaMm if viaMm is not None else k.viaMm) / k.grid
        ys, xs = np.nonzero(tgt.any(axis=0)) if tgt.any() else (np.array([0]), np.array([0]))
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
            return None, False
        path = outPath[:n].tolist()
        terminalVia = path[-1] == -1
        if terminalVia:
            path = path[:-1]
        cells = []
        for f in path:
            li, rem = divmod(f, h * w)
            i, j = divmod(rem, w)
            cells.append((layers[li], i + win[0], j + win[1], bool(wideOk[li, i, j])))
        return cells, terminalVia

    @staticmethod
    def layerName(l):
        return LAYER_NAMES[l]

    def layerDir(self, layers):
        """Extra cost per unit length along each axis, per search layer: the diagonal that
        is the other outer layer's main direction."""
        out = np.zeros((len(layers), 4), np.float32)
        for li, l in enumerate(layers):
            if l in self.mainAxis:
                out[li, (self.mainAxis[l] + 2) % 4] = self.k.againstCost
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
        # a pair's centreline is checked as nobody's copper: clear of both nets' pads
        nid = -1 if self.asForeign else self.netId[net]
        return Item(0, nid, "track", (layer,), geom, clearance, width=width, pts=(p1, p2), route=rid)

    def viaItem(self, net, p, rule, rid):
        geom = Point(p).buffer(rule.viaDia / 2 * CIRC, 16)
        return Item(0, self.netId[net], "via", self.allLayers, geom, rule.clearance,
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

    @staticmethod
    def legs(p, q):
        """Ways to join p and q octilinearly: a direct segment when p->q is octilinear, else
        the two L shapes (straight then diagonal, diagonal then straight), each given as the
        corner point."""
        dx, dy = q[0] - p[0], q[1] - p[1]
        adx, ady = abs(dx), abs(dy)
        if adx < 1e-6 or ady < 1e-6 or abs(adx - ady) < 1e-6:
            return [None]
        sx, sy = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
        if adx > ady:
            run = adx - ady
            return [(p[0] + sx * run, p[1]), (q[0] - sx * run, q[1])]
        run = ady - adx
        return [(p[0], p[1] + sy * run), (q[0], q[1] - sy * run)]

    JOG_MM = 0.06   # a piece this short between two bends is a jog, not a deliberate run

    @staticmethod
    def foldJogs(pts):
        """A tiny piece between two bends (where two offset runs of a pair meet) folded into
        one corner where its neighbours' lines cross, when that turn is no sharper than a
        right angle. The ends stay where they are."""
        pts = list(pts)
        i = 1
        while i + 2 < len(pts):
            a, b, c, d = pts[i - 1], pts[i], pts[i + 1], pts[i + 2]
            if math.dist(b, c) >= Router.JOG_MM:
                i += 1
                continue
            u = (b[0] - a[0], b[1] - a[1])
            v = (d[0] - c[0], d[1] - c[1])
            den = u[0] * v[1] - u[1] * v[0]
            if abs(den) < 1e-12 or (u[0] * v[0] + u[1] * v[1]) / (math.hypot(*u) * math.hypot(*v)) < -0.01:
                i += 1
                continue
            t = ((c[0] - a[0]) * v[1] - (c[1] - a[1]) * v[0]) / den
            x = (a[0] + t * u[0], a[1] + t * u[1])
            if t <= 0 or math.dist(x, b) > 2 * Router.JOG_MM or math.dist(x, c) > 2 * Router.JOG_MM \
                    or (x[0] - c[0]) * v[0] + (x[1] - c[1]) * v[1] >= (d[0] - c[0]) * v[0] + (d[1] - c[1]) * v[1]:
                i += 1
                continue
            pts[i:i + 2] = [x]
        return pts

    @staticmethod
    def heading(p, q):
        dx, dy = q[0] - p[0], q[1] - p[1]
        return (0 if abs(dx) < 1e-6 else (1 if dx > 0 else -1),
                0 if abs(dy) < 1e-6 else (1 if dy > 0 else -1))

    def segOk(self, conn, layer, p, q, width, rule):
        it = self.trackItem(conn.net, layer, p, q, width, rule.clearance, 0)
        return not self.check(it)

    def pathOk(self, conn, layer, pts, width, rule):
        return all(a == b or self.segOk(conn, layer, a, b, width, rule) for a, b in zip(pts, pts[1:]))

    def pullTight(self, conn, layer, pts, wide, rule, before=None, after=None, onTrack=(False, False)):
        """Greedy string pulling with octilinear shortcuts, straight or L-shaped, that pass
        the exact check at the width the segment will have. Only stretches the raster
        planned as necks may narrow. `before`/`after`: the headings the run arrives with
        and leaves in, so a shortcut's corner lands where a bend already is. `onTrack`:
        whether the run's first / last point is a T junction on another track, where the
        piece that meets it must not be shorter than MIN_LEG."""
        def width(a, b, wideHere):
            return rule.width if wideHere else self.pickWidth(conn, layer, a, b, rule)
        if len(pts) <= 2:
            return pts, [width(pts[0], pts[-1], all(wide))]
        out, widths = [pts[0]], []
        kidx = 0
        while kidx < len(pts) - 1:
            nxt, chosen = kidx + 1, None
            incoming = self.heading(out[-2], out[-1]) if len(out) > 1 else before
            for j in range(len(pts) - 1, kidx + 1, -1):
                wideHere = all(wide[kidx:j + 1])
                wj = rule.width if wideHere else rule.narrow
                outgoing = self.heading(pts[j], pts[j + 1]) if j + 1 < len(pts) else after
                cands = []
                for corner in self.legs(pts[kidx], pts[j]):
                    path = [pts[kidx], pts[j]] if corner is None else [pts[kidx], corner, pts[j]]
                    # prefer the L that continues the incoming heading or meets the outgoing one
                    score = (0 if corner is None else 2)
                    if corner is not None and self.heading(path[0], path[1]) == incoming:
                        score -= 1
                    if corner is not None and self.heading(path[1], path[2]) == outgoing:
                        score -= 1
                    cands.append((score, path))
                for _, path in sorted(cands, key=lambda c: c[0]):
                    if kidx == 0 and onTrack[0] and math.dist(path[0], path[1]) < self.MIN_LEG:
                        continue
                    if j == len(pts) - 1 and onTrack[1] and math.dist(path[-2], path[-1]) < self.MIN_LEG:
                        continue
                    if self.pathOk(conn, layer, path, wj, rule):
                        chosen, nxt = (path, wideHere), j
                        break
                if chosen:
                    break
            path, wideHere = chosen if chosen else ([pts[kidx], pts[nxt]], all(wide[kidx:nxt + 1]))
            for a, b in zip(path, path[1:]):
                widths.append(width(a, b, wideHere))
                out.append(b)
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
        if self.nearNarrowPad(conn, p, q, rule):
            return rule.narrow
        # no reason to neck here: the width stays, and the route's exact check turns the
        # route down if it does not fit
        return rule.width

    def nearNarrowPad(self, conn, p, q, rule):
        """Whether p-q comes within neckMm of an end pad narrower than the track."""
        seg = LineString([p, q]) if p != q else Point(p)
        return any(pad is not None and padMinDim(pad) < rule.width - 1e-6
                   and seg.distance(pad.poly) < self.k.neckMm + 1e-6 for pad in (conn.a, conn.b))

    MIN_LEG = 0.1   # mm; KiCad's connectivity ignores a shorter piece at a T junction

    @staticmethod
    def octilinearHits(p, a, b):
        """Points of segment a-b that p reaches along one octilinear leg, nearest first."""
        out = []
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        for hx, hy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            det = hx * (-dy) - hy * (-dx)
            if abs(det) < 1e-9:
                continue
            # p + t*h = a + u*(b-a)
            rx, ry = ax - p[0], ay - p[1]
            t = (rx * (-dy) - ry * (-dx)) / det
            u = (hx * ry - hy * rx) / det
            if t >= 0 and -1e-9 <= u <= 1 + 1e-9:
                out.append((t * math.hypot(hx, hy), (p[0] + t * hx, p[1] + t * hy)))
        out.sort(key=lambda e: e[0])
        return out

    def viaOntoTrack(self, conn, res, runs, rule):
        """A route that ends (or starts) with a via landing on another track of its net,
        off that track's centreline: the via and the piece before it slide onto the
        centreline (KiCad warns about a track that is not centred on a via)."""
        runs = list(runs)
        for end in (-1, 0):
            if len(runs) < 2 or len(runs[end][1]) != 1 or (end == -1 and res["terminalVia"]):
                continue
            lyr, pts1, w1 = runs[end]
            items = res["dstItems"] if end == -1 else res["srcItems"]
            p = pts1[0]
            owner = next((it for it in items if lyr in it.layers and it.kind == "track"
                          and it.geom.buffer(1e-3).contains(Point(p))), None)
            if owner is None:
                continue
            k = -2 if end == -1 else 1
            lyr0, pts0, w0 = runs[k]
            seq = pts0 if end == -1 else pts0[::-1]
            moved = self.slideOnto(conn, lyr0, seq, owner, rule)
            q = moved[-1]
            if q == seq[-1] or self.check(self.viaItem(conn.net, q, rule, 0)):
                continue
            runs[k] = (lyr0, moved if end == -1 else moved[::-1], w0)
            runs[end] = (lyr, [q], w1)
        return runs

    def slideOnto(self, conn, layer, pts, track, rule):
        """A run that ends inside another track's copper but off its centreline (no leg
        long enough fitted) is slid along its own last piece onto that centreline: KiCad
        joins a track end to another track only there. Returns the points (unchanged when
        the slid piece does not fit)."""
        if len(pts) < 2:
            return pts
        p, prev = pts[-1], pts[-2]
        if LineString(track.pts).distance(Point(p)) < 1e-4:
            return pts
        h = self.heading(prev, p)
        best = None
        for d, q in self.octilinearHits(prev, *track.pts):
            if self.heading(prev, q) == h and (best is None or abs(math.dist(prev, q) - math.dist(prev, p))
                                               < abs(math.dist(prev, best) - math.dist(prev, p))):
                best = q
        if best is None or math.dist(prev, best) < 1e-6 or not self.segOk(conn, layer, prev, best, rule.narrow, rule):
            return pts
        return pts[:-1] + [best]

    def entryLegs(self, conn, layer, p, items, rule):
        """Points that take a route end on to the centre of its pad (or the centreline of
        the track / the via it lands on), straight or with one bend, and the item it lands
        on: (points, item). ([], None) when it lands on nothing."""
        for it in items:
            if layer not in it.layers or not it.geom.buffer(1e-3).contains(Point(p)):
                continue
            if it.kind == "pad":
                c = it.pad.center
                if c == p or not it.geom.contains(Point(c)):
                    # already there, or a pad with no copper at its centre (a ring around a
                    # microphone's sound port): the end stays where it landed, on the copper
                    return [], it
                # the last leg along the pad's long axis, as a person draws it, at full
                # width unless the pad is narrower than the track (then at the pad's width,
                # or the narrow width if that does not fit)
                minx, miny, maxx, maxy = it.pad.poly.bounds
                axis = (1, 0) if maxx - minx >= maxy - miny else (0, 1)
                pw = padMinDim(it.pad)
                ew = rule.width if pw >= rule.width - 1e-6 else max(rule.narrow, pw)
                wide = ew >= rule.width - 1e-6
                best = []
                for corner in self.legs(p, c):
                    path = [p, c] if corner is None else [p, corner, c]
                    if not self.pathOk(conn, layer, path, ew, rule) and \
                            (wide or not self.pathOk(conn, layer, path, rule.narrow, rule)):
                        continue
                    h = self.heading(path[-2], path[-1])
                    along = (abs(h[0]), abs(h[1])) == axis
                    if along or not best:
                        best = path[1:]
                    if along:
                        break
                self.entryWide = wide
                return best, it
            if it.kind == "track":
                # on to the centreline with one leg no shorter than KiCad needs; a point
                # already inside the copper may stay where it is
                inside = LineString(it.pts).distance(Point(p)) <= it.width / 2 - 1e-3
                self.entryWide = True
                for d, q in self.octilinearHits(p, *it.pts):
                    if d < self.MIN_LEG:
                        continue
                    if self.segOk(conn, layer, p, q, rule.width, rule):
                        return [q], it
                return [], it if inside else None
            if it.kind == "via":
                q = it.pts[0]
                self.entryWide = True
                if q == p:
                    return [], it
                for corner in self.legs(p, q):
                    path = [p, q] if corner is None else [p, corner, q]
                    if self.pathOk(conn, layer, path, rule.width, rule):
                        return path[1:], it
                return [], it
        return [], None

    # ---- commit ------------------------------------------------------------------------
    def buildItems(self, conn, res, rid):
        rule = res["rule"]
        runs, _ = self.pathToGeometry(conn, res)
        runs = self.viaOntoTrack(conn, res, runs, rule)
        items = []
        tight = []
        for ri, (layer, pts, wide) in enumerate(runs):
            onTrack = [False, False]
            if ri == 0:
                self.entryWide = False
                e, owner = self.entryLegs(conn, layer, pts[0], res["srcItems"], rule)
                res["srcOwner"] = owner
                onTrack[0] = owner is not None and owner.kind == "track"
                if onTrack[0] and not e:
                    pts = self.slideOnto(conn, layer, pts[::-1], owner, rule)[::-1]
                pts = e[::-1] + pts
                wide = [self.entryWide] * len(e) + wide
            if ri == len(runs) - 1 and not res["terminalVia"]:
                self.entryWide = False
                e, owner = self.entryLegs(conn, layer, pts[-1], res["dstItems"], rule)
                res["dstOwner"] = owner
                onTrack[1] = owner is not None and owner.kind == "track"
                if onTrack[1] and not e:
                    pts = self.slideOnto(conn, layer, pts, owner, rule)
                pts = pts + e
                wide = wide + [self.entryWide] * len(e)
            raw = list(pts)
            pts, widths = self.pullTight(conn, layer, pts, wide, rule, onTrack=onTrack)
            if os.environ.get("HR_TRACE_NET") == conn.net:
                self.log(f"trace {conn.net} {conn.a.ref}.{conn.a.num}->{conn.b.ref + '.' + conn.b.num if conn.b else 'plane'} "
                         f"layer {layer} run {ri}: raw {raw} -> {pts} widths {widths}")
            tight.append([layer, list(pts), list(widths)])
        self.snapVias(conn, tight, rule, res["terminalVia"])
        for layer, pts, widths in tight:
            for p, q, wdt in zip(pts, pts[1:], widths):
                if p != q:
                    items.append(self.trackItem(conn.net, layer, p, q, wdt, rule.clearance, rid))
        for k in range(len(tight) - 1):
            items.append(self.viaItem(conn.net, tight[k][1][-1], rule, rid))
        if res["terminalVia"]:
            items.append(self.viaItem(conn.net, tight[-1][1][-1], rule, rid))
        return items

    STUB_MM = 0.3   # a track piece this short between a bend and a via is a stub

    @staticmethod
    def lineCross(p1, p2, q1, q2):
        """Where the line through p1, p2 meets the line through q1, q2 (None if parallel)."""
        d1 = (p2[0] - p1[0], p2[1] - p1[1])
        d2 = (q2[0] - q1[0], q2[1] - q1[1])
        det = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(det) < 1e-9:
            return None
        t = ((q1[0] - p1[0]) * d2[1] - (q1[1] - p1[1]) * d2[0]) / det
        return (p1[0] + d1[0] * t, p1[1] + d1[1] * t)

    def snapVias(self, conn, tight, rule, terminalVia):
        """A via a stub away from where its tracks bend is moved onto the bend: to where
        the two layers' track lines meet, or to the stub's far end, whichever keeps every
        piece octilinear and passes the exact checks. tight: [[layer, pts, widths]] per
        layer run, in order; changed in place."""
        STUB = self.STUB_MM
        rv = rule.viaDia / 2

        def viaOk(x):
            return not self.check(self.viaItem(conn.net, x, rule, 0))

        def segsOk(layer, pts, widths):
            return all(a == b or self.segOk(conn, layer, a, b, w, rule) for a, b, w in zip(pts, pts[1:], widths))

        def clean(pts):
            return all(self.octilinear(a, b) and math.dist(a, b) > 1e-6 for a, b in zip(pts, pts[1:]))
        # a hop: down a layer and back up within a millimetre, where staying would do
        k = 0
        while k + 2 < len(tight):
            la, A, wa = tight[k]
            lm, Mp, wm = tight[k + 1]
            lc, C, wc = tight[k + 2]
            hop = sum(math.dist(a, b) for a, b in zip(Mp, Mp[1:]))
            if la == lc and A and C and hop < 1.0:
                P, Q = A[-1], C[0]
                link = None
                for corner in self.legs(P, Q):
                    path = [P, Q] if corner is None else [P, corner, Q]
                    w = min(wa[-1] if wa else rule.width, wc[0] if wc else rule.width)
                    if P == Q or segsOk(la, path, [w] * (len(path) - 1)):
                        link = (path, w)
                        break
                if link is not None:
                    path, w = link
                    merged = A + path[1:] + C[1:]
                    widths = list(wa) + [w] * (len(path) - 1) + list(wc)
                    segs = [(a, b, w) for a, b, w in zip(merged, merged[1:], widths) if a != b]
                    if segs:
                        merged = [segs[0][0]] + [b for _, b, _ in segs]
                        widths = [w for _, _, w in segs]
                    else:
                        merged, widths = merged[:1], []
                    tight[k:k + 3] = [[la, merged, widths]]
                    continue
            k += 1
        for k in range(len(tight) - 1):
            la, A, wa = tight[k]
            lb, Bp, wb = tight[k + 1]
            if not A or not Bp:
                continue
            V = A[-1]
            cands = []
            if len(A) >= 3 and math.dist(A[-2], V) < STUB:          # stub ends run k
                X = self.lineCross(A[-3], A[-2], V, Bp[1]) if len(Bp) >= 2 else None
                for x in (X, A[-2]):
                    if x is not None:
                        cands.append((x, A[:-2] + [x], wa[:len(A) - 2], [x] + Bp[1:], list(wb)))
            if len(Bp) >= 3 and math.dist(V, Bp[1]) < STUB:         # stub starts run k+1
                X = self.lineCross(A[-2], V, Bp[1], Bp[2]) if len(A) >= 2 else None
                for x in (X, Bp[1]):
                    if x is not None:
                        cands.append((x, A[:-1] + [x], list(wa), [x] + Bp[2:], wb[1:]))
            for x, nA, nwa, nB, nwb in cands:
                # a run that would be left as a bare point was the route's landing on
                # copper there (a track, a pad): the via stays where it lands
                if len(nA) < 2 or len(nB) < 2:
                    continue
                if math.dist(x, V) > 2 * STUB or not clean(nA) or not clean(nB):
                    continue
                if viaOk(x) and segsOk(la, nA, nwa) and segsOk(lb, nB, nwb):
                    tight[k][1:] = [nA, nwa]
                    tight[k + 1][1:] = [nB, nwb]
                    break
        if terminalVia and tight:
            la, A, wa = tight[-1]
            if len(A) >= 3 and math.dist(A[-2], A[-1]) < STUB:
                x = A[-2]
                i, j = self.sp.toCell(*x)
                gap = conn.viaGapMm if conn.viaGapMm >= 0 else self.k.fanoutViaGapMm
                onPlaneOk = conn.net not in self.planeCells or self.planeCells[conn.net][i, j]
                padOk = conn.kind != "gnd" or self.padEdgeDist[i, j] >= rv + gap + self.k.grid
                if onPlaneOk and padOk and viaOk(x):
                    tight[-1][1:] = [A[:-1], wa[:len(A) - 2]]

    def endpointOwners(self, conn, res):
        """Route ids / pad nodes the path starts and ends on."""
        sp = self.sp
        nodes, parents = [], set()
        for cell, pool, owner in ((res["cells"][0], res["srcItems"], res.get("srcOwner")),
                                  (res["cells"][-1], res["dstItems"], res.get("dstOwner"))):
            best = owner   # what the entry legs physically attached to
            if best is None:
                p = Point(sp.toMm(cell[1], cell[2]))
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
        """Remove a route and everything that hangs off it; returns the connections freed.
        A soft piece of existing copper (drawn by an earlier run) can be ripped too: the
        planned connections of its net that it served come back."""
        ripped = [] if ripped is None else ripped
        if rid < 0:
            return self.ripSoft(rid, ripped)
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

    def routeBends(self, rid):
        """Corners in a route: points where two of its track pieces meet at an angle."""
        ends = {}
        for i in self.routes[rid].items:
            it = self.sp.items[i]
            if it.kind != "track":
                continue
            p, q = it.pts
            ends.setdefault((it.layers[0], p), []).append(self.heading(p, q))
            ends.setdefault((it.layers[0], q), []).append(self.heading(q, p))
        return sum(1 for hs in ends.values() if len(hs) == 2 and hs[0] != (-hs[1][0], -hs[1][1]))

    def ripSoft(self, rid, ripped):
        if rid not in self.softPieces:
            return ripped
        net, nodes = self.existing.pop(rid)
        self.softPieces.discard(rid)
        for it in self.existingItems.pop(rid):
            self.sp.remove(it.iid)
            self.netItems.get(it.net, set()).discard(it.iid)
            self.droppedUuids.add(it.uuid)
        pads = {n for n in nodes if n[0] == "pad"}
        for c in self.allConns:
            if c.net == net and c not in ripped and (("pad", c.a.pid) in pads or
                                                     (c.b is not None and ("pad", c.b.pid) in pads)):
                c.routed = False
                ripped.append(c)
        for child in list(self.children.pop(rid, ())):
            self.rip(child, ripped)
        return ripped

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

    def improve(self, rid, layers, winPads, viaMm, straighten=False):
        """Rip a leaf route and search again with everything else in place; keep the new one
        only if it is shorter or uses fewer vias. `straighten`: search with bends priced
        high, and also keep a route with fewer corners that is not longer."""
        r = self.routes.get(rid)
        if r is None or self.children.get(rid) or r.paired:
            return False
        before, viasBefore = self.routeCost(rid)
        bendsBefore = self.routeBends(rid)
        r.oldItems = [self.sp.items[i] for i in r.items]
        self.rip(rid)
        turn = self.k.turn45Mm
        if straighten:
            self.k.turn45Mm = turn * self.k.straightenMult
        try:
            st, new = self.route(r.conn, layers, winPads, viaMm=viaMm)
        finally:
            self.k.turn45Mm = turn
        if st == "ok" and new is not None:
            after, viasAfter = self.routeCost(new.rid)
            if after < before - 0.3 or viasAfter < viasBefore:
                return True
            if straighten and viasAfter <= viasBefore and after <= before + 0.3 \
                    and self.routeBends(new.rid) < bendsBefore:
                return True
            self.rip(new.rid)
        elif st == "ok" and self.nodesJoined(r):
            # its two pads are joined some other way, and so is everything the route
            # itself was holding together: nothing to put back
            return True
        self.restore(r)
        return False

    def nodesJoined(self, r):
        """Whether the nodes a (ripped) route joined are still in one component."""
        find = self.components(r.conn.net)
        roots = {find(n) for n in r.nodes}
        return len(roots) <= 1

    def audit(self, log, phase):
        """Track ends that touch nothing of their net, or only routes this route does not
        record as parents (so a rip could strand it): bookkeeping errors."""
        bad = 0
        for rid, r in self.routes.items():
            own = set(r.items)
            for i in r.items:
                it = self.sp.items[i]
                if it.kind != "track":
                    continue
                for p in it.pts:
                    touching = [o for o in self.sp.query(it.layers, (p[0], p[1], p[0], p[1]))
                                if o.iid != it.iid and o.net == it.net and o.kind in ("pad", "track", "via")
                                and o.geom.buffer(1e-3).contains(Point(p))]
                    where = f"{r.conn.net} route {rid} ({r.conn.a.ref}.{r.conn.a.num}->" \
                            f"{r.conn.b.ref + '.' + r.conn.b.num if r.conn.b else 'plane'}) end {p} layer {it.layers[0]}"
                    if not touching:
                        bad += 1
                        log(f"audit {phase}: {where} touches nothing")
                    elif not any((o.kind == "pad" and self.nodeOf(o) in r.nodes) or o.iid in own
                                 or o.route in r.parents for o in touching):
                        bad += 1
                        log(f"audit {phase}: {where} touches only {sorted({(o.kind, o.route, o.tag) for o in touching})}; "
                            f"nodes {r.nodes} parents {sorted(r.parents)}")
        return bad

    # ---- copper already on the board ----------------------------------------------------------
    def loadExisting(self, tracks, vias, soft=frozenset()):
        """Tracks [(net, layer, p1, p2, width, uuid)] and vias [(net, p, dia, drill, uuid)]
        already on the board, as fixed copper: nothing may cross them, negotiation never
        rips them, and routes of their net may join them. Each connected piece of it is a
        pseudo-route (negative id) that joins the pads it touches, and the plane when it
        holds a via. Items whose uuid is in `soft` were drawn by an earlier run of the
        router: they form pieces of their own that negotiation may rip like a route (and
        the connections they served are routed again)."""
        items = []
        for t in tracks:
            net, layer, p1, p2, w = t[:5]
            nid = self.netId.get(net, HARD)
            r = w / 2 * CIRC
            geom = LineString([p1, p2]).buffer(r, 16) if p1 != p2 else Point(p1).buffer(r, 16)
            cl = self.rules[net].clearance if net in self.rules else self.sp.defaultClearance
            uid = t[5] if len(t) > 5 else None
            items.append(Item(0, nid, "track", (layer,), geom, cl, width=w, pts=(p1, p2),
                              tag="soft" if uid in soft else "existing"))
            items[-1].uuid = uid
        for v in vias:
            net, p, dia, drill = v[:4]
            nid = self.netId.get(net, HARD)
            cl = self.rules[net].clearance if net in self.rules else self.sp.defaultClearance
            uid = v[4] if len(v) > 4 else None
            items.append(Item(0, nid, "via", self.allLayers, Point(p).buffer(dia / 2 * CIRC, 16), cl,
                              width=dia, pts=(p,), drill=drill, tag="soft" if uid in soft else "existing"))
            items[-1].uuid = uid
        for it in items:
            self.sp.add(it)
            self.netItems.setdefault(it.net, set()).add(it.iid)
        parent = {}

        def find(x):
            parent.setdefault(x, x)
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x
        contacts = []
        for it in items:
            find(it.iid)
            if it.net == HARD:
                continue
            anchors = [(it.layers[0], q) for q in it.pts] if it.kind == "track" else \
                [(l, it.pts[0]) for l in it.layers]
            for l, q in anchors:
                pt = Point(q)
                for o in self.sp.query([l], (q[0], q[1], q[0], q[1])):
                    if o.iid == it.iid or o.net != it.net or not o.geom.buffer(1e-3).contains(pt):
                        continue
                    if o.kind == "pad":
                        parent[find(it.iid)] = find(("pad", o.pad.pid))
                    elif o.tag == it.tag:
                        parent[find(it.iid)] = find(o.iid)
                    elif o.tag in ("existing", "soft"):
                        contacts.append((it.iid, o.iid))   # router copper on the designer's
        clusters = {}
        for it in items:
            clusters.setdefault(find(it.iid), []).append(it)
        pads = {}
        for x in list(parent):
            if isinstance(x, tuple):
                pads.setdefault(find(x), []).append(x)
        for n, (root, members) in enumerate(clusters.items(), 1):
            rid = -n
            net = self.netName.get(members[0].net)
            nodes = list(pads.get(root, []))
            if net in self.planeNets and any(m.kind == "via" and self.onPlane(net, m.pts[0]) for m in members):
                nodes.append(("plane", net))
            for m in members:
                m.route = rid
            if net is not None:
                self.existing[rid] = (net, nodes)
                self.existingItems[rid] = members
                if members[0].tag == "soft":
                    self.softPieces.add(rid)
        # a piece of router copper touching the designer's joins it (both ways)
        byIid = {it.iid: it for it in items}
        for a, b in contacts:
            ra, rb = byIid[a].route, byIid[b].route
            if ra in self.existing and ("route", rb) not in self.existing[ra][1]:
                self.existing[ra][1].append(("route", rb))
        return len(items)

    def padlessPieces(self):
        """Pieces of existing copper of a net other than the poured ground that touch none
        of their net's pads (a via and a stub left where a part used to be, an orphan
        track): rids. A ground piece without a pad is a stitching via."""
        return [rid for rid, (net, nodes) in self.existing.items()
                if net not in self.outerPourNets and not any(n[0] == "pad" for n in nodes)]

    def dropExisting(self, rid):
        """Take a piece of existing copper out of the model; returns its board uuids."""
        uuids = set()
        for it in self.existingItems.pop(rid, []):
            self.sp.remove(it.iid)
            self.netItems.get(it.net, set()).discard(it.iid)
            uuids.add(it.uuid)
        self.existing.pop(rid, None)
        return uuids

    # ---- via tidying ---------------------------------------------------------------------
    def swapItems(self, r, old, new):
        """Replace some of a route's items with new ones (all of the route's net)."""
        for it in old:
            self.sp.remove(it.iid)
            self.netItems[it.net].discard(it.iid)
            r.items.remove(it.iid)
        for it in new:
            it.route = r.rid
            self.sp.add(it)
            self.netItems.setdefault(it.net, set()).add(it.iid)
            r.items.append(it.iid)

    def tracksAt(self, r, p, layer, skip=()):
        out = []
        for i in r.items:
            it = self.sp.items[i]
            if it.kind == "track" and layer in it.layers and i not in skip \
                    and any(math.dist(q, p) < 1e-4 for q in it.pts):
                out.append(it)
        return out

    def stubToPad(self, r, via, layer, maxLen):
        """The route's tracks on `layer` from the via to a pad of its net, when they get
        there in at most three pieces without branching and within maxLen mm."""
        cur, chain, seen, length = via.pts[0], [], set(), 0.0
        for _ in range(3):
            ts = self.tracksAt(r, cur, layer, seen)
            if len(ts) != 1:
                return None
            t = ts[0]
            chain.append(t)
            seen.add(t.iid)
            length += math.dist(*t.pts)
            if length > maxLen:
                return None
            cur = t.pts[1] if math.dist(t.pts[0], cur) < 1e-4 else t.pts[0]
            pad = self.padUnder(r.conn.net, layer, cur)
            if pad is not None:
                return pad, chain, cur
        return None

    def tidyVias(self, maxLen=3.0):
        """Vias a short stub away from a pad are moved so the stub leaves the pad's centre
        in a straight line, as a person places them: along the pad's long axis first, then
        any octilinear direction. Routes other routes hang off are left alone, and so is a
        via or stub that other copper of the net touches (moving it would leave that
        copper, or this route, hanging)."""
        moved = 0
        for rid, r in list(self.routes.items()):
            c = r.conn
            if c.kind not in ("sig", "pwr") or c.b is None or self.children.get(rid) or r.paired:
                continue
            rule = self.ruleForConn(c)
            for via in [self.sp.items[i] for i in r.items if self.sp.items[i].kind == "via"]:
                if self.joinedByOthers(via, r):
                    continue
                for layer in (F, B):
                    st = self.stubToPad(r, via, layer, maxLen)
                    if st is None:
                        continue
                    pad, chain, end = st
                    if any(self.joinedByOthers(t, r) for t in chain):
                        continue
                    ctr = pad.center
                    v = via.pts[0]
                    if len(chain) == 1 and math.dist(end, ctr) < 1e-4 and self.octilinear(ctr, v):
                        continue
                    # the route carries on from the via on whichever layers it does (the
                    # other outer layer, or an inner signal layer): each of those tails is
                    # drawn again from the moved via on its own layer
                    tails = [(t, l2) for l2 in self.allLayers if l2 != layer for t in self.tracksAt(r, v, l2)]
                    if any(len(self.tracksAt(r, v, l2, {t.iid})) > 1 for t, l2 in tails):
                        continue
                    if any(self.joinedByOthers(t, r) for t, _ in tails):
                        continue
                    # each tail as its chain of up to three unbranched pieces from the via, so
                    # the moved via can join it further along instead of doubling back; the
                    # chain stops before a piece other copper of the net touches
                    tailChains = []
                    for t, l2 in tails:
                        pieces, pts, cur = [t], [v], v
                        while True:
                            last = pieces[-1]
                            nxt = last.pts[1] if math.dist(last.pts[0], cur) < 1e-4 else last.pts[0]
                            pts.append(nxt)
                            cur = nxt
                            more = self.tracksAt(r, cur, l2, {p.iid for p in pieces})
                            if (len(pieces) >= 3 or len(more) != 1 or self.padUnder(c.net, l2, cur) is not None
                                    or self.joinedByOthers(more[0], r)):
                                break
                            pieces.append(more[0])
                        tailChains.append((l2, pieces, pts))
                    oldPieces = len(chain) + sum(len(ps) for _, ps, _ in tailChains)
                    minx, miny, maxx, maxy = pad.poly.bounds
                    axis = ((1, 0), (-1, 0)) if maxx - minx >= maxy - miny else ((0, 1), (0, -1))
                    diag = ((1, 1), (1, -1), (-1, 1), (-1, -1))
                    cross = ((0, 1), (0, -1)) if axis[0] == (1, 0) else ((1, 0), (-1, 0))
                    d0 = math.dist(ctr, v)
                    dists = sorted({round(max(0.3, d0 + dd), 3) for dd in (0, -0.25, 0.25, -0.5, 0.5, 0.75, 1.0)})
                    dists.sort(key=lambda d: abs(d - d0))
                    done = False
                    for h in axis + diag + cross:
                        hl = math.hypot(*h)
                        for d in dists:
                            nv = (ctr[0] + h[0] / hl * d, ctr[1] + h[1] / hl * d)
                            if math.dist(nv, v) < 1e-4:
                                continue
                            new = [self.viaItem(c.net, nv, rule, rid),
                                   self.trackItem(c.net, layer, ctr, nv, chain[0].width, rule.clearance, rid)]
                            ok = True
                            dropped = []
                            for l2, pieces, pts in tailChains:
                                w = pieces[0].width
                                ign = {via.iid} | {p.iid for p in pieces}
                                found = None
                                for j in range(len(pts) - 1, 0, -1):   # the farthest point first
                                    for corner in self.legs(nv, pts[j]):
                                        path = [nv, pts[j]] if corner is None else [nv, corner, pts[j]]
                                        if all(a == b or not self.check(self.trackItem(c.net, l2, a, b, w, rule.clearance, rid),
                                                                         ignore=ign) for a, b in zip(path, path[1:])):
                                            found = (j, path)
                                            break
                                    if found:
                                        break
                                if found is None:
                                    ok = False
                                    break
                                j, path = found
                                dropped += pieces[:j]
                                new += [self.trackItem(c.net, l2, a, b, w, rule.clearance, rid)
                                        for a, b in zip(path, path[1:]) if a != b]
                            if not ok or len(new) - 1 > oldPieces:
                                continue
                            ignore = {via.iid} | {t.iid for t in chain} | {p.iid for _, ps, _ in tailChains for p in ps}
                            if any(self.check(it, ignore=ignore) for it in new) or self.viaSpacing(new + [
                                    self.sp.items[i] for i in r.items if self.sp.items[i].kind == "via" and i != via.iid]):
                                continue
                            self.swapItems(r, [via] + chain + dropped, new)
                            moved += 1
                            done = True
                            break
                        if done:
                            break
                    if done:
                        break
        return moved

    # ---- differential pairs laid together ------------------------------------------------
    def padItems(self, pad):
        nid = self.netId[pad.net]
        return [self.sp.items[i] for i in self.netItems.get(nid, ())
                if self.sp.items[i].kind == "pad" and self.sp.items[i].pad is pad]

    @staticmethod
    def unit(p, q):
        L = math.dist(p, q)
        return ((q[0] - p[0]) / L, (q[1] - p[1]) / L) if L > 1e-9 else (1.0, 0.0)

    @staticmethod
    def offsetPolyline(pts, d):
        """The polyline shifted sideways by d (to the left of its direction for d > 0),
        corners mitred."""
        segs = []
        for p, q in zip(pts, pts[1:]):
            if math.dist(p, q) > 1e-9:
                u = Router.unit(p, q)
                segs.append((p, q, u, (-u[1], u[0])))
        if not segs:
            return [(pts[0][0], pts[0][1])]
        out = [(segs[0][0][0] + segs[0][3][0] * d, segs[0][0][1] + segs[0][3][1] * d)]
        for (p1, q1, u1, n1), (p2, q2, u2, n2) in zip(segs, segs[1:]):
            a = (q1[0] + n1[0] * d, q1[1] + n1[1] * d)
            b = (p2[0] + n2[0] * d, p2[1] + n2[1] * d)
            det = u1[0] * (-u2[1]) - u1[1] * (-u2[0])
            if abs(det) < 1e-9:
                out.append(a)
            else:
                rx, ry = b[0] - a[0], b[1] - a[1]
                t = (rx * (-u2[1]) - ry * (-u2[0])) / det
                out.append((a[0] + u1[0] * t, a[1] + u1[1] * t))
        p, q, u, n = segs[-1]
        out.append((q[0] + n[0] * d, q[1] + n[1] * d))
        return out

    SLOT_MM = 0.8   # how far a slotted pair end runs straight out from its pads
    PAIR_RETRY_SEARCHES = 16   # searches the side retries may take: every combination once, and a few more

    def pairEndSlots(self, lead, res):
        """(start, end) slot specs that repeat how a centreline left its start and came into
        its end: (point just behind the end, outward direction)."""
        runs = self.pathToGeometry(lead, res)[0]
        pts = [p for _, ps, _ in runs for p in ps]
        out = []
        for seq in (pts, pts[::-1]):
            p0 = seq[0]
            p1 = next((q for q in seq[1:] if math.dist(q, p0) > 1e-6), None)
            if p1 is None:
                out.append(None)
                continue
            u = self.unit(p0, p1)
            out.append(((p0[0] - u[0] * self.k.grid, p0[1] - u[1] * self.k.grid), u))
        return tuple(out)

    def searchPair(self, lead, follow, layers, winPad, extraMargin, viaMm, reach=0.0, sides=(0, 0), avoid=(),
                   keep=(None, None)):
        """One search for both nets of a pair: a centreline wide enough for two tracks at
        the pair gap, from between the two start pads to between the two end pads.
        sides: (start, end), each 0 or +-1: keep that end of the centreline on one side of
        the line through its two pads, so it arrives from that side (the side that does
        not swap the two nets over). avoid: [(x, y, r)] where the centreline may not start
        or end (a spot an earlier try could not get the pads' legs to). keep: (start, end),
        each None or (point, outward direction): an unslotted end leaves from that point in
        that direction (where and how an earlier try left)."""
        sp, k = self.sp, self.k
        rule = self.ruleForConn(lead)
        gap = self.m.classes[rule.cls].pairGap
        half = (rule.width + gap) / 2
        hv = (rule.viaDia + max(rule.clearance, gap)) / 2
        W = 2 * rule.width + gap
        crule = NetRule(W, W, rule.clearance, 2 * hv + rule.viaDia, rule.viaDrill, rule.cls)
        srcItems = self.padItems(lead.a) + self.padItems(follow.a)
        dstItems = self.padItems(lead.b) + self.padItems(follow.b)
        if len(srcItems) < 2 or len(dstItems) < 2:
            return None
        pads = [lead.a, follow.a, lead.b, follow.b]
        xs = [p.center[0] for p in pads]
        ys = [p.center[1] for p in pads]
        win = sp.winFor((min(xs), min(ys), max(xs), max(ys)), winPad)
        # the centreline keeps clear of everything, the four pads included: each track
        # leaves its pad on its own legs, so the pair never runs along a partner's pad
        cost, viaok, wideOk, _ = self.buildCost(lead, crule, win, layers, [], [], False, extraMargin,
                                                      ownNets=(-1,), ownPads=srcItems + dstItems)
        i0, j0, i1, j1 = win
        cy, cx = sp.cellCenters(win)

        walls = np.zeros((i1 - i0, j1 - j0), bool)
        slots = []   # (mid, outward unit, length) of each slotted end, start first

        def ends(a, b, items, side, spec=None):
            # close to the two pads, but far enough from both that each track's start is
            # clear of the other net's pad: pins too tight for two tracks between them
            # (an ESD array, a connector) get short legs out to where the pair starts
            m = np.zeros((len(layers), i1 - i0, j1 - j0), bool)
            mid = ((a.center[0] + b.center[0]) / 2, (a.center[1] + b.center[1]) / 2)
            r = max(half + k.grid, reach)
            disk = (cx - mid[0]) ** 2 + (cy - mid[1]) ** 2 <= r ** 2
            if side or spec:
                # a slot straight out from the pad pair on that side (or from a given point
                # in a given direction), walled along its length, so the centreline can
                # only come in head-on from there
                if spec:
                    mid, un = spec
                else:
                    L = math.dist(a.center, b.center) or 1e-9
                    ud = ((a.center[0] - b.center[0]) / L, (a.center[1] - b.center[1]) / L)
                    un = (-ud[1] * side, ud[0] * side)
                ud = (un[1], -un[0])
                along = (cx - mid[0]) * ud[0] + (cy - mid[1]) * ud[1]
                out = (cx - mid[0]) * un[0] + (cy - mid[1]) * un[1]
                lane = np.abs(along) <= 1.5 * k.grid
                disk = lane & (out > 0) & (out <= r + 1.5)
            else:
                out = None
            pads = np.zeros((i1 - i0, j1 - j0), bool)
            for it in items:
                self.paint(pads, win, it)
            if pads.any():
                disk &= edt(pads) * k.grid >= half + rule.width / 2 + rule.clearance
            # and room for both tracks' starts beside the centreline, exactly
            need = half + rule.width / 2 + rule.clearance + 2 * k.grid
            for li, l in enumerate(layers):
                if all(l in it.layers for it in items):
                    m[li] = disk & (sp.foreignDist(0, l, win, nets=(-1,)) >= need)
            m &= cost >= 0
            if out is not None and m.any():
                # only the slot's inner end: the centreline then has to run out along it,
                # far enough (SLOT_MM) to set its direction, between walls, and closed behind
                # the pads (on a layer the pads are not on, the way in would be from behind)
                first = out[m.any(axis=0)].min()
                m &= (out <= first + 2 * k.grid)[None]
                length = first + self.SLOT_MM
                slots.append((mid, un, length))
                beside = np.abs(along) <= half + 0.5
                walls[(~lane) & beside & (out > -0.1) & (out <= length - k.grid)] = True
                walls[beside & (out <= 0) & (out > -(half + 1.0))] = True
            return m
        src = ends(lead.a, follow.a, srcItems, sides[0], keep[0] if not sides[0] else None)
        dst = ends(lead.b, follow.b, dstItems, sides[1], keep[1] if not sides[1] else None)
        if walls.any():
            cost[:, walls] = -1
            src &= cost >= 0
            dst &= cost >= 0
        for x, y, rr in avoid:
            spot = (cx - x) ** 2 + (cy - y) ** 2 <= rr * rr
            src &= ~spot[None]
            dst &= ~spot[None]

        trace = os.environ.get("HR_TRACE_PAIR")
        if not src.any() or not dst.any():
            if trace:
                self.log(f"pair {lead.net}/{follow.net}: reach {reach} winPad {winPad}: "
                         f"{int(src.sum())} start cells, {int(dst.sum())} end cells")
            return None
        cells, _ = self.runKernel(lead, cost, viaok, wideOk, np.flatnonzero(src).astype(np.int32),
                                  dst.astype(np.uint8), None, win, layers, viaMm, False)
        if cells is None:
            if trace:
                self.log(f"pair {lead.net}/{follow.net}: reach {reach} winPad {winPad}: no path "
                         f"({int(src.sum())} start cells, {int(dst.sum())} end cells)")
            return None
        return dict(cells=cells, terminalVia=False, srcItems=srcItems, dstItems=dstItems, rule=crule,
                    slots=slots,
                    win=win, half=half, hv=hv, trackRule=rule)

    def layPair(self, lead, follow, res):
        """Two sets of items from a pair's centreline: each net on its own side, vias
        splayed apart at every layer change, each pad joined to its track's start with
        straight legs. Returns (itemsA, itemsB) or None (the pair would have to cross)."""
        rule, crule = res["trackRule"], res["rule"]
        half, hv = res["half"], res["hv"]
        w = rule.width
        runs, _ = self.pathToGeometry(lead, res)
        # the centreline is pulled tight as nobody's copper of double width: it must keep
        # clear of the pads of both nets, as the search did
        g = self.k.grid

        def inSlot(p):
            """The slot p lies in (its lane), if any."""
            for mid, un, length in res.get("slots", ()):
                rel = (p[0] - mid[0], p[1] - mid[1])
                out = rel[0] * un[0] + rel[1] * un[1]
                along = rel[0] * -un[1] + rel[1] * un[0]
                if abs(along) <= 2 * g and -g <= out <= length + g:
                    return mid, un, length
            return None

        def tighten(layer, pts):
            """pullTight, except along a slot: the centreline leaves (or enters) the pads
            head-on there, which is the point of the slot, and a shortcut would cut it."""
            head = tail = 0
            if len(pts) > 2 and inSlot(pts[0]):
                while head + 1 < len(pts) - 1 and inSlot(pts[head + 1]):
                    head += 1
            if len(pts) > 2 and inSlot(pts[-1]):
                while tail + 1 < len(pts) - 1 - head and inSlot(pts[-2 - tail]):
                    tail += 1
            mid = pts[head:len(pts) - tail]
            if len(mid) > 2:
                mid = self.pullTight(lead, layer, mid, [True] * len(mid), crule)[0]
            out = ([pts[0]] if head else []) + list(mid) + ([pts[-1]] if tail else [])
            return [p for i, p in enumerate(out) if i == 0 or p != out[i - 1]]
        self.asForeign = True
        try:
            tight = [(layer, tighten(layer, pts)) for layer, pts, wide in runs]
        finally:
            self.asForeign = False

        def runDir(ri, atStart):
            """Direction of the centreline at a run's start or end; a lone-point run takes
            its neighbour's."""
            pts = tight[ri][1]
            if len(pts) >= 2:
                return self.unit(pts[0], pts[1]) if atStart else self.unit(pts[-2], pts[-1])
            if ri > 0:
                return runDir(ri - 1, False)
            return runDir(ri + 1, True) if ri + 1 < len(tight) else (1.0, 0.0)

        def leftOf(padA, padB, u):
            """+1 when padA lies to the left of padB looking along u, -1 to the right, 0
            when they are in line (then the other end decides)."""
            dx, dy = padA.center[0] - padB.center[0], padA.center[1] - padB.center[1]
            v = -u[1] * dx + u[0] * dy
            return 0 if abs(v) < 0.05 else (1 if v > 0 else -1)
        p0, u0 = tight[0][1][0], runDir(0, True)
        p1, u1 = tight[-1][1][-1], runDir(len(tight) - 1, False)
        sA0, sA1 = leftOf(lead.a, follow.a, u0), leftOf(lead.b, follow.b, u1)
        sA0, sA1 = sA0 or sA1 or 1, sA1 or sA0 or 1
        sB0 = -sA0
        trace = os.environ.get("HR_TRACE_PAIR")
        if trace:
            self.log(f"pair {lead.net}/{follow.net}: centreline {[(l, [(round(x, 3), round(y, 3)) for x, y in pts]) for l, pts in tight]} sides {sA0}/{sB0}")
        self.pairCrossed = sA0 != sA1
        if sA0 != sA1:   # the pads ask for opposite sides: the pair would have to cross
            if trace:
                self.log(f"pair {lead.net}/{follow.net}: would cross; start {p0} dir {u0}, end {p1} dir {u1}")
            return None
        # at a layer change the two vias sit either side of the centreline on the
        # bisector of the incoming and outgoing directions, so each net stays on its side
        # through a turn; the tracks splay out to them and back
        viaPos = {}
        for ri in range(len(tight) - 1):
            V = tight[ri][1][-1]
            uin, uout = runDir(ri, False), runDir(ri + 1, True)
            nb = (-(uin[1] + uout[1]), uin[0] + uout[0])
            L = math.hypot(*nb)
            nb = (nb[0] / L, nb[1] / L) if L > 1e-6 else (-uin[1], uin[0])
            for sign in (1, -1):
                viaPos[(ri, sign)] = (V[0] + nb[0] * sign * hv, V[1] + nb[1] * sign * hv)
        plan = []   # per net: (conn, [(layer, points)], via points)
        for conn, sign in ((lead, sA0), (follow, sB0)):
            d = sign * half
            polys = []
            for ri, (layer, pts) in enumerate(tight):
                off = self.offsetPolyline(pts, d)
                s = hv - half
                if ri < len(tight) - 1:              # ends in a via
                    V, u = pts[-1], runDir(ri, False)
                    n = (-u[1], u[0])
                    vp = viaPos[(ri, sign)]
                    if len(pts) >= 2 and math.dist(pts[-2], pts[-1]) > s + 0.05:
                        off[-1] = (V[0] - u[0] * s + n[0] * d, V[1] - u[1] * s + n[1] * d)
                        off.append(vp)
                    else:
                        off[-1] = vp
                if ri > 0:                            # starts at a via
                    V, u = pts[0], runDir(ri, True)
                    n = (-u[1], u[0])
                    vp = viaPos[(ri - 1, sign)]
                    if len(pts) >= 2 and math.dist(pts[0], pts[1]) > s + 0.05:
                        off[0] = (V[0] + u[0] * s + n[0] * d, V[1] + u[1] * s + n[1] * d)
                        off.insert(0, vp)
                    else:
                        off[0] = vp
                polys.append((layer, off))
            plan.append((conn, polys, [vp for (ri, sg), vp in viaPos.items() if sg == sign]))

        def merged(pts):
            """Without repeated points and without the middle point of collinear pieces."""
            out = []
            for p in pts:
                if out and math.dist(out[-1], p) < 1e-6:
                    continue
                if len(out) >= 2 and self.heading(out[-2], out[-1]) == self.heading(out[-1], p):
                    out[-1] = p
                else:
                    out.append(p)
            return self.foldJogs(out)

        def pieces(conn, polys, vias):
            items = [self.trackItem(conn.net, layer, p, q, w, rule.clearance, 0)
                     for layer, pts in polys for p, q in zip(merged(pts), merged(pts)[1:])]
            return items + [self.viaItem(conn.net, vp, rule, 0) for vp in vias]
        # the legs from each pad to its track, with the pair's own copper in view so a leg
        # that has to go round a neighbouring pin does not land on the partner's track;
        # then each end is pulled tight (legs plus the track's first bend) so the join
        # reads as one line
        temp = []
        try:
            for conn, polys, vias in plan:
                for it in pieces(conn, polys, vias):
                    self.sp.add(it)
                    temp.append(it)
            for conn, polys, vias in plan:
                for end, pad in ((0, conn.a), (-1, conn.b)):
                    layer, pts = polys[end]
                    p = pts[0] if end == 0 else pts[-1]
                    legs = self.padLegs(conn, layer, pad, p, rule)
                    if legs is None:
                        legs = self.legRoute(conn, pad, p, layer, rule)
                    if legs is None:
                        ep = tight[0][1][0] if end == 0 else tight[-1][1][-1]
                        self.pairLegFail = ep
                        if trace:
                            probe = self.trackItem(conn.net, layer, pad.center, p, rule.width, rule.clearance, 0)
                            self.log(f"pair {lead.net}/{follow.net}: no way from {pad.ref}.{pad.num} {pad.center} to {p}; "
                                     f"a straight leg hits {[(o.kind, o.tag, self.netName.get(o.net, o.net), o.layers) for o in self.check(probe)][:6]}")
                        return None
                    # a tiny first piece of the track (the search's first step) may go too
                    if end == 0:
                        k = 2 if len(pts) > 2 and math.dist(pts[0], pts[1]) < 0.3 else 1
                        head = legs + pts[:k]
                        head, _ = self.pullTight(conn, layer, head, [False] * len(head), rule)
                        polys[end] = (layer, head + pts[k:])
                    else:
                        k = 2 if len(pts) > 2 and math.dist(pts[-2], pts[-1]) < 0.3 else 1
                        tail = pts[-k:] + legs[::-1]
                        tail, _ = self.pullTight(conn, layer, tail, [False] * len(tail), rule)
                        polys[end] = (layer, pts[:-k] + tail)
        finally:
            for it in temp:
                self.sp.remove(it.iid)
                it.stamped = []
        return [pieces(conn, polys, vias) for conn, polys, vias in plan]

    def legRoute(self, conn, pad, target, layer, rule):
        """A short search from a pad to a point on one layer (a pair's track start that no
        straight leg reaches): the points from the pad to just before the target, or None."""
        sp = self.sp
        srcItems = self.padItems(pad)
        if layer not in pad.layers or not srcItems:
            return None
        c = pad.center
        bounds = (min(c[0], target[0]), min(c[1], target[1]), max(c[0], target[0]), max(c[1], target[1]))
        for winPad in (1.5, 3.0):
            win = sp.winFor(bounds, winPad)
            cost, viaok, wideOk, _ = self.buildCost(conn, rule, win, [layer], srcItems, [], False, 0.0)
            src = self.endpointCells(srcItems, [layer], win, cost)
            cy, cx = sp.cellCenters(win)
            near = (cx - target[0]) ** 2 + (cy - target[1]) ** 2
            dst = (near <= (1.5 * sp.g) ** 2)[None] & (cost >= 0)
            if not src.any() or not dst.any():
                continue
            cells, _ = self.runKernel(conn, cost, viaok, wideOk, np.flatnonzero(src).astype(np.int32),
                                      dst.astype(np.uint8), None, win, [layer], None, False)
            trace = os.environ.get("HR_TRACE_PAIR")
            if cells is None:
                if trace:
                    self.log(f"leg {conn.net} {pad.ref}.{pad.num} -> {target}: no path in window {winPad}")
                continue
            res = dict(cells=cells, terminalVia=False, srcItems=srcItems, dstItems=[], rule=rule)
            (lyr, pts, wide), = self.pathToGeometry(conn, res)[0]
            entry, owner = self.entryLegs(conn, layer, pts[0], srcItems, rule)
            pts = entry[::-1] + pts
            wide = [False] * len(entry) + wide
            for corner in self.legs(pts[-1], target):
                tail = [pts[-1], target] if corner is None else [pts[-1], corner, target]
                if self.pathOk(conn, layer, tail, rule.width, rule):
                    pts = pts + tail[1:]
                    wide = wide + [False] * (len(tail) - 1)
                    break
            else:
                if trace:
                    self.log(f"leg {conn.net} {pad.ref}.{pad.num} -> {target}: path ends at {pts[-1]}, no clear tail")
                continue
            pts, widths = self.pullTight(conn, layer, pts, wide, rule)
            return pts[:-1]
        return None

    def padLegs(self, conn, layer, pad, p, rule):
        """Points from the pad's centre to p (not including p): straight or one bend, the
        first leg along the pad's long axis when it can be."""
        c = pad.center
        if math.dist(c, p) < 1e-6:
            return [c]
        minx, miny, maxx, maxy = pad.poly.bounds
        axis = (1, 0) if maxx - minx >= maxy - miny else (0, 1)
        best = None
        for corner in self.legs(c, p):
            path = [c, p] if corner is None else [c, corner, p]
            if not self.pathOk(conn, layer, path, rule.width, rule):
                continue
            h = self.heading(path[0], path[1])
            along = (abs(h[0]), abs(h[1])) == axis
            if along or best is None:
                best = path[:-1]
            if along:
                break
        return best

    def pairClear(self, itemsA, itemsB):
        """The two nets' new items against each other (the board check cannot see them)."""
        for a in itemsA:
            for b in itemsB:
                if not set(a.layers) & set(b.layers):
                    continue
                if a.geom.distance(b.geom) < max(a.clearance, b.clearance) - 1e-6:
                    return False
                if a.kind == "via" and b.kind == "via" and \
                        math.dist(a.pts[0], b.pts[0]) - a.drill / 2 - b.drill / 2 < self.m.minHoleToHole:
                    return False
        return True

    def commitPair(self, conn, items):
        rid = self.nextRid
        self.nextRid += 1
        for it in items:
            it.route = rid
            self.sp.add(it)
            self.netItems.setdefault(it.net, set()).add(it.iid)
        r = Route(rid, conn, [it.iid for it in items], [("pad", conn.a.pid), ("pad", conn.b.pid)],
                  set(), paired=True)
        self.routes[rid] = r
        conn.routed = True
        conn.rid = rid
        return r

    def layablePair(self, lead, follow, res):
        """The two nets' items for a pair centreline, or None when they would cross, miss
        a pad or fail the exact checks."""
        self.pairCrossed = False
        self.pairLegFail = None
        laid = self.layPair(lead, follow, res)
        if laid is None:
            return None
        itemsA, itemsB = laid
        bad = [it for it in itemsA + itemsB if self.check(it)]
        if bad or self.viaSpacing(itemsA) or self.viaSpacing(itemsB) or not self.pairClear(itemsA, itemsB):
            if os.environ.get("HR_TRACE_PAIR"):
                self.log(f"pair {lead.net}/{follow.net}: {len(bad)} pieces fail the exact check, "
                         f"cross clear {self.pairClear(itemsA, itemsB)}: "
                         + "; ".join(f"{it.kind} {it.layers[0]} {it.pts} w{it.width} hits "
                                     f"{[(o.kind, o.tag, self.netName.get(o.net, o.net), o.layers, o.pts) for o in self.check(it)]}"
                                     for it in bad[:3]))
            return None
        return itemsA, itemsB

    def routePair(self, lead, follow, layers, winPads, viaMm=None):
        """Lay both nets of a pair together; True when both are now routed. Tried once per
        pair (repairPair asks again at the end). When the cheapest centreline cannot be laid
        (the two nets would cross, or one's legs would run into the other's pad), the search
        runs again with each end kept on one side of its pad pair, and the shortest of those
        that lays cleanly is used."""
        if lead.routed or follow.routed or self.connected(lead) or self.connected(follow) \
                or lead.pairTried:
            return False
        lead.pairTried = follow.pairTried = True
        trace = os.environ.get("HR_TRACE_PAIR")
        retried = False     # the side retries run once: a wider window does not move the pads
        for reach in (0.0, self.k.pairStartMm, 2 * self.k.pairStartMm):   # between the pads first, then near them
            for wp in winPads:
                retry = False
                for extra in (0.0, 0.03):
                    res = self.searchPair(lead, follow, layers, wp, extra, viaMm, reach)
                    if res is None:
                        break           # no path in this window: a wider margin will not help
                    laid = self.layablePair(lead, follow, res)
                    if laid is None:
                        # it would cross, or it came in along the line of a pad pair and a
                        # leg runs into the partner's pad: another side may keep it apart
                        retry = True
                        if self.pairCrossed:
                            break
                        continue        # or a raster error: once more with margin
                    self.commitPair(lead, laid[0])
                    self.commitPair(follow, laid[1])
                    if trace:
                        self.log(f"pair {lead.net}/{follow.net}: laid together")
                    return True
                if retry and not retried:
                    retried = True
                    best = None
                    # first keep the unslotted end where the first try had it (its legs may
                    # well fit there), then let it roam
                    first = res if res is not None else None
                    ends_ = self.pairEndSlots(lead, first) if first is not None else (None, None)
                    tries = [(sd, ends_) for sd in ((0, 1), (0, -1), (1, 0), (-1, 0))] if first is not None else []
                    tries += [(sd, (None, None)) for sd in ((0, 0), (0, 1), (0, -1), (1, 0), (-1, 0),
                                                            (1, 1), (1, -1), (-1, 1), (-1, -1))]
                    budget = self.PAIR_RETRY_SEARCHES
                    for sides, keep in tries:
                        if budget <= 0:
                            break
                        avoid = []
                        for _ in range(8):
                            if budget <= 0:
                                break
                            budget -= 1
                            res = self.searchPair(lead, follow, layers, wp, 0.0, viaMm, reach, sides, avoid, keep)
                            laid = self.layablePair(lead, follow, res) if res is not None else None
                            if trace:
                                self.log(f"pair {lead.net}/{follow.net}: sides {sides}{' kept' if keep[0] else ''} reach {reach} winPad {wp}: "
                                         + ("no path" if res is None else "laid" if laid else
                                            "would cross" if self.pairCrossed else "does not lay"))
                            if laid is not None or res is None or self.pairLegFail is None:
                                break
                            # the pads' legs could not reach that end: start (or end) elsewhere
                            avoid.append((self.pairLegFail[0], self.pairLegFail[1], 0.15))
                        if laid is None:
                            continue
                        length = sum(math.dist(*it.pts) for it in laid[0] + laid[1] if it.kind == "track")
                        if best is None or length < best[0]:
                            best = (length, laid, sides)
                    if best is not None:
                        self.commitPair(lead, best[1][0])
                        self.commitPair(follow, best[1][1])
                        if trace:
                            self.log(f"pair {lead.net}/{follow.net}: laid together from the side that keeps "
                                     f"the polarity {best[2]} ({best[0]:.1f} mm of track)")
                        return True
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
        lead.pairTried = follow.pairTried = False
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
        """The run p->q with trapezoid bumps (45 deg flanks, flat top) to one side that add
        up to `extra` mm of length, at most `amax` high: few and tall, as a person draws
        them. Returns (points, mm added), or None when the run is too short."""
        L = math.dist(p, q)
        if L < 1e-6:
            return None
        u = ((q[0] - p[0]) / L, (q[1] - p[1]) / L)
        n = (-u[1] * side, u[0] * side)
        top = max(2 * w, pitch)                 # a bump's flat top
        gap = max(0.5 * amax, pitch)            # between bumps
        margin = max(2 * w, 0.3)
        perA = 2 * (math.sqrt(2) - 1)           # mm gained per mm of amplitude
        amin = max(2 * w, 0.3)
        if extra < perA * amin or amax < amin:
            return None
        nb = math.ceil(extra / (perA * amax))
        A = min(amax, extra / (nb * perA))
        while nb >= 1 and nb * (2 * A + top) + (nb - 1) * gap > L - 2 * margin:
            nb -= 1
            A = min(amax, extra / (nb * perA)) if nb else 0.0
        if nb < 1:
            return None
        span = nb * (2 * A + top) + (nb - 1) * gap
        x = (L - span) / 2
        at = lambda a, b: (p[0] + u[0] * a + n[0] * b, p[1] + u[1] * a + n[1] * b)
        pts = [p]
        for _ in range(nb):
            pts += [at(x, 0), at(x + A, A), at(x + A + top, A), at(x + 2 * A + top, 0)]
            x += 2 * A + top + gap
        pts.append(q)
        return pts, nb * perA * A

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
            if self.joinedByOthers(it, r):
                continue   # another route joins this piece: bending it would leave that one hanging
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

    def joinedByOthers(self, it, r):
        """Whether copper of the same net that is not route r's touches track piece it."""
        own = set(r.items)
        return any(o.iid not in own and o.net == it.net and o.kind in ("track", "via")
                   and o.geom.intersects(it.geom)
                   for o in self.sp.query(it.layers, it.geom.bounds))

    def underBody(self, item, layer):
        """Copper of a route that would sit under a module or antenna body."""
        win, mask = self.sp.rasterMask(item.geom)
        i0, j0, i1, j1 = win
        return bool((self.bodyKeep[layer, i0:i1, j0:j1] & mask).any()) if mask.size else False

    def matchPairs(self, partners, budgetFor, log):
        """Differential pairs whose two nets differ in length by more than their skew
        budget: the shorter net is lengthened with serpentines to match. A pair that is
        off by far more than its budget took two different paths; that is left for the
        reviewer rather than hidden under a wall of bumps. Without a skew rule only
        point-to-point pairs are matched: a branch adds to a net's length, not to the
        pair's (budgetFor returns (mm, fromRule))."""
        for a, b in sorted(partners.items()):
            if a > b or a not in self.netId or b not in self.netId:
                continue
            budget, fromRule = budgetFor(a)
            la, lb = self.netLength(a), self.netLength(b)
            skew = abs(la - lb)
            if skew <= budget:
                continue
            if not fromRule and max(len(self.m.nets.get(a, ())), len(self.m.nets.get(b, ()))) > 2:
                # a net with branches (a test point, a second load): its total length is not
                # the length along the pair, so there is nothing to match against
                log(f"length match {a}/{b}: skew {skew:.2f} mm left alone (no skew rule; the "
                    f"nets branch)")
                continue
            if not fromRule and skew > max(4 * budget, 5.0):
                log(f"length match {a}/{b}: skew {skew:.2f} mm left alone (no skew rule; the two "
                    f"nets took different paths)")
                continue
            short = a if la < lb else b
            conns = sorted({id(rt.conn): rt.conn for rt in self.routes.values()
                            if rt.conn.net == short and rt.conn.b is not None}.values(),
                           key=lambda c: -self.routeCost(c.rid)[0])
            added, want = 0.0, skew - budget * 0.5
            for c in conns:
                if want - added <= 0.05:
                    break
                w = self.ruleForConn(c).width
                added += self.lengthen(c, want - added, max(1.5, 6 * w))
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
    def stitchMask(self, net="GND", padGap=None):
        """Cells where a plane via may go, judged on the current board (padGap: bare board
        to every pad, stitchPadGapMm unless given)."""
        sp, k = self.sp, self.k
        rule = self.rules[net]
        nid = self.netId[net]
        win = (0, 0, sp.H, sp.W)
        rv = rule.viaDia / 2
        m = k.margin
        ok = sp.foreignDist(nid, None, win, reach=0) >= rv + max(rule.clearance, 0.25) + m
        ok &= ~sp.viaKeep & ~self.ruleAreas
        ok &= self.padEdgeDist >= rv + (k.stitchPadGapMm if padGap is None else padGap) + sp.g
        ok &= self.holeDist >= rule.viaDrill / 2 + self.m.minHoleToHole + m
        ok &= sp.maskDist(sp.viaAt, win, reach=0, pred=lambda a: a != 0) >= rule.viaDrill + 0.6
        return ok

    CONNECTOR_PREFIXES = {"J", "P", "CN", "CON", "X", "USB"}

    def isRingChip(self, pad):
        """A chip with a ring of pins: many pads, and not a connector (a connector's rows
        have no inside to put a via in)."""
        m = re.match(r"[A-Za-z]+", pad.ref)
        prefix = m.group(0).upper() if m else ""
        return len(self.padsOfRef.get(pad.ref, ())) > self.RING_PADS and prefix not in self.CONNECTOR_PREFIXES

    def isExposedPad(self, pad):
        """A chip's centre pad: inside its pin ring and large."""
        if not self.isRingChip(pad) or pad.poly.area < 1.0:
            return False
        if pad.ref not in self.hulls:
            self.hulls[pad.ref] = unary_union([q.poly for q in self.padsOfRef[pad.ref]]).convex_hull
        return self.hulls[pad.ref].buffer(-0.3).contains(Point(pad.center))

    def routeToPour(self, conn, target, winPads=(1.0, 2.5, 5.0)):
        """Join a pad the pour does not reach to `target` (board raster of stitched pour on
        the pad's layer) with a short track on that layer. True when done."""
        sp = self.sp
        layer = conn.a.side
        rule = self.ruleForConn(conn)
        srcItems = self.padItems(conn.a)
        if not srcItems:
            return False
        for wp in winPads:
            win = sp.winFor(conn.a.poly.bounds, wp)
            i0, j0, i1, j1 = win
            cost, viaok, wideOk, _ = self.buildCost(conn, rule, win, [layer], srcItems, [], False, 0.0)
            src = self.endpointCells(srcItems, [layer], win, cost)
            dst = target[i0:i1, j0:j1][None] & (cost >= 0) & ~src
            if not src.any() or not dst.any():
                continue
            cells, _ = self.runKernel(conn, cost, np.zeros_like(viaok), wideOk,
                                      np.flatnonzero(src).astype(np.int32), dst.astype(np.uint8), None,
                                      win, [layer], None, False)
            if cells is None:
                continue
            res = dict(cells=cells, terminalVia=False, srcItems=srcItems, dstItems=[], rule=rule, win=win)
            items = self.buildItems(conn, res, self.nextRid)
            if not items or any(self.check(it) for it in items):
                continue
            r = self.commit(conn, res, items)
            r.nodes.append(("plane", conn.net))   # the pour it reaches is stitched to the plane
            return True
        return False

    def viaInPad(self, conn):
        """A plane via inside the (exposed) pad itself. True when one fits."""
        pad, net = conn.a, conn.net
        rule = self.rules[net]
        room = pad.poly.buffer(-(rule.viaDia / 2 + 0.05))
        if room.is_empty:
            return False
        minx, miny, maxx, maxy = room.bounds
        pts = [pad.center] + [(x, y) for x in np.arange(minx, maxx + 1e-9, 0.2)
                              for y in np.arange(miny, maxy + 1e-9, 0.2) if room.contains(Point(x, y))]
        pts.sort(key=lambda p: math.dist(p, pad.center))
        own = {it.iid for it in self.padItems(pad)}
        for pt in pts[:40]:
            if not room.contains(Point(pt)):
                continue
            rid = self.nextRid
            via = self.viaItem(net, pt, rule, rid)
            if self.check(via, ignore=own):
                continue
            self.nextRid += 1
            self.sp.add(via)
            self.netItems.setdefault(via.net, set()).add(via.iid)
            self.routes[rid] = Route(rid, conn, [via.iid], [("pad", pad.pid), ("plane", net)], set())
            conn.routed, conn.rid = True, rid
            return True
        return False

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

    def returnVias(self, net="GND", reach=1.5, placeMm=2.0):
        """A plane via next to every signal via that has none within reach (placed within
        placeMm, as near as the pad gap allows): the return current of a track that changes
        layers changes planes there too."""
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
            pt = self.placePlaneVia(self.nearestOk(ok, x, y, placeMm, minMm=0.0), net)
            if pt is not None:
                gnd.append(pt)
                i, j = sp.toCell(*pt)
                r = int(0.9 / sp.g)
                ok[max(i - r, 0):i + r + 1, max(j - r, 0):j + r + 1] = False
                added += 1
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
        """Route conn; returns ('ok', route) | ('fail', None) | ('conflict', victims). A
        plane fanout that does not fit at its pad's width is tried once more thin."""
        st, payload = self.routeOnce(conn, layers, winPads, negotiate, viaMm)
        if st == "fail" and conn.b is None and conn.kind == "gnd" and not conn.thin \
                and conn.net not in self.outerPourNets:   # only a supply fanout is drawn wide
            conn.thin = True
            st, payload = self.routeOnce(conn, layers, winPads, negotiate, viaMm)
        return st, payload

    def routeOnce(self, conn, layers, winPads, negotiate, viaMm):
        if self.connected(conn):
            conn.routed = True
            return "ok", None
        mate = conn.pairMate
        if mate is not None and not negotiate and not mate.routed and not self.connected(mate) \
                and self.routePair(conn, mate, layers, winPads, viaMm):
            return "ok", self.routes[conn.rid]
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
            soft = self.softPieces
            victims = {o.route for o in bad if o.kind in ("track", "via") and (o.route > 0 or o.route in soft)}
            fixedHit = [o for o in bad if o.kind not in ("track", "via") or (o.route < 0 and o.route not in soft)]
            if negotiate and victims and not fixedHit:
                return "conflict", (res, items, victims)
            # raster error against fixed copper: retry with a wider margin
        return "fail", None
