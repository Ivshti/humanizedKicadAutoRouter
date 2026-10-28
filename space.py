"""Routing space: a per-layer copper ownership raster plus an exact geometric store.

The raster answers "how far is this cell from copper that is not mine" (a distance
transform over a window) and drives the search. The geometric store holds every
copper item as a shapely shape and answers the exact clearance question, so the
raster's sub-cell error never reaches the board.
"""
import math
from collections import defaultdict
from dataclasses import dataclass, field

import numpy as np
import shapely
from native import edt
from shapely.geometry import LineString, Point, box

HARD = -1   # owner value for copper/obstacles that belong to no routable net


@dataclass(eq=False)
class Item:
    iid: int
    net: int              # net id; HARD for no-net copper and obstacles
    kind: str             # pad | track | via | hole | edge | keepout
    layers: tuple
    geom: object          # exact copper shape (for a via: the annulus disk)
    clearance: float      # the owner's clearance (max() with the other side applies)
    width: float = 0.0
    pts: tuple = ()       # track: ((x1,y1),(x2,y2)); via: ((x,y),)
    drill: float = 0.0
    tag: str = ""         # free-form: which phase / connection placed it
    smd: bool = False
    noTracks: bool = False
    noVias: bool = False
    route: int = 0        # owning route id (tracks and vias)
    stamped: list = field(default_factory=list)


class Space:
    def __init__(self, outline, grid, margin, edgeClearance, defaultClearance, nLayers):
        self.L = nLayers   # copper layers (index 0 F.Cu, 1 B.Cu, then the inner ones)
        self.g = grid
        self.defaultClearance = defaultClearance
        self.margin = margin
        minx, miny, maxx, maxy = outline.bounds
        self.x0, self.y0 = minx - 0.5, miny - 0.5
        self.W = int(math.ceil((maxx - minx + 1.0) / grid)) + 1
        self.H = int(math.ceil((maxy - miny + 1.0) / grid)) + 1
        self.occ = np.zeros((self.L, self.H, self.W), np.int16)
        self.fixed = np.zeros((self.L, self.H, self.W), np.int16)  # pads, holes, edge only
        # copper of a net class whose clearance is below the default (fine-pitch classes):
        # a search of such a class keeps only its own clearance from this copper
        self.fine = np.zeros((self.L, self.H, self.W), bool)
        self.trackKeep = np.zeros((self.L, self.H, self.W), bool)
        self.viaKeep = np.zeros((self.H, self.W), bool)
        self.smdPad = np.zeros((self.H, self.W), np.int16)   # SMD pad owner on F or B
        self.viaAt = np.zeros((self.H, self.W), np.int16)    # via centre owner
        self.holeMask = np.zeros((self.H, self.W), bool)     # every drilled hole
        self.items = {}
        self.nextId = 1
        self.buckets = [defaultdict(set) for _ in range(self.L)]
        self.bucket = 2.0
        self.outline = outline
        self.edgeClearance = edgeClearance
        # Outside the board (pulled in so that the router's own clearance adds up to the
        # edge clearance) is a hard obstacle on every layer.
        inner = outline.buffer(-(edgeClearance - defaultClearance))
        shapely.prepare(inner)
        ys, xs = self.cellCenters((0, 0, self.H, self.W))
        inside = shapely.contains_xy(inner, xs, ys)
        for l in range(self.L):
            self.occ[l][~inside] = HARD
            self.fixed[l][~inside] = HARD

    # ---- coordinates -------------------------------------------------------------
    def toCell(self, x, y):
        return int(round((y - self.y0) / self.g)), int(round((x - self.x0) / self.g))

    def toMm(self, i, j):
        return self.x0 + j * self.g, self.y0 + i * self.g

    def cellCenters(self, win):
        i0, j0, i1, j1 = win
        ii, jj = np.mgrid[i0:i1, j0:j1]
        return self.y0 + ii * self.g, self.x0 + jj * self.g

    def winFor(self, bounds, pad=0.0):
        minx, miny, maxx, maxy = bounds
        i0, j0 = self.toCell(minx - pad, miny - pad)
        i1, j1 = self.toCell(maxx + pad, maxy + pad)
        i0, j0 = min(max(i0, 0), self.H), min(max(j0, 0), self.W)
        # an end below zero would read as "from the end" in a numpy slice: clamp both ends
        # into the raster so a window off the board is empty, not most of the board
        return (i0, j0, max(min(i1 + 1, self.H), i0), max(min(j1 + 1, self.W), j0))

    # ---- rasterising -------------------------------------------------------------
    def rasterMask(self, geom, grow=0.0):
        """Cells whose centre lies inside geom (grown by `grow`); returns (win, mask)."""
        g = geom.buffer(grow) if grow else geom
        shapely.prepare(g)  # in place; contains_xy over many cells is ~100x faster
        win = self.winFor(g.bounds, self.g)
        if win[2] <= win[0] or win[3] <= win[1]:  # off the board: empty, but the window's shape
            return win, np.zeros((max(win[2] - win[0], 0), max(win[3] - win[1], 0)), bool)
        ys, xs = self.cellCenters(win)
        return win, shapely.contains_xy(g, xs, ys)

    def segMask(self, p1, p2, radius):
        (x1, y1), (x2, y2) = p1, p2
        win = self.winFor((min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)), radius + self.g)
        ys, xs = self.cellCenters(win)
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        if L2 == 0:
            d2 = (xs - x1) ** 2 + (ys - y1) ** 2
        else:
            t = np.clip(((xs - x1) * dx + (ys - y1) * dy) / L2, 0, 1)
            d2 = (xs - x1 - t * dx) ** 2 + (ys - y1 - t * dy) ** 2
        return win, d2 <= radius * radius

    # ---- items -------------------------------------------------------------------
    def add(self, item):
        item.iid = self.nextId
        self.nextId += 1
        self.items[item.iid] = item
        self.stamp(item)
        self.index(item, True)
        return item

    def remove(self, iid):
        item = self.items.pop(iid)
        self.index(item, False)
        # copper stamped fixed (pads, holes, the board's own copper) leaves the fixed raster
        # too: a lifted piece of the designer's copper must not stay a wall
        fixed = item.kind in ("pad", "hole") or item.tag == "existing"
        for (l, win, mask) in item.stamped:
            i0, j0, i1, j1 = win
            if fixed:
                fsub = self.fixed[l, i0:i1, j0:j1]
                fsub[mask & (fsub == item.net)] = 0
            sub = self.occ[l, i0:i1, j0:j1]
            sub[mask & (sub == item.net)] = 0
            self.fine[l, i0:i1, j0:j1][mask & (sub == 0)] = False
        if item.kind == "via":
            i, j = self.toCell(*item.pts[0])
            if self.viaAt[i, j] == item.net:
                self.viaAt[i, j] = 0
        # restamp same-net neighbours that shared those cells
        minx, miny, maxx, maxy = item.geom.bounds
        for other in self.query(range(self.L), (minx, miny, maxx, maxy)):
            if other.net == item.net:
                self.stamp(other, record=False)
        return item

    def stamp(self, item, record=True):
        if item.kind == "edge":  # the outside is stamped once at construction
            return
        # copper that demands more than the default clearance is stamped grown by the
        # difference, so every search's default-based distance test still holds
        grow = max(item.clearance - self.defaultClearance, 0.0) if item.kind != "hole" else 0.0
        if item.kind == "track":
            win, mask = self.segMask(item.pts[0], item.pts[1], item.width / 2 + grow)
        elif item.kind == "via":
            win, mask = self.segMask(item.pts[0], item.pts[0], item.width / 2 + grow)
            i, j = self.toCell(*item.pts[0])
            self.viaAt[i, j] = item.net
        else:
            win, mask = self.rasterMask(item.geom, grow)
        if mask.size == 0:
            return
        i0, j0, i1, j1 = win
        # pads, holes and copper that was already on the board are fixed: negotiation
        # never rips them
        fixed = item.kind in ("pad", "hole") or item.tag == "existing"
        rasters = [self.occ, self.fixed] if fixed else [self.occ]
        for l in item.layers:
            for r in rasters:
                sub = r[l, i0:i1, j0:j1]
                if item.net == HARD:
                    sub[mask] = HARD
                else:
                    sub[mask & (sub == 0)] = item.net
            if item.clearance < self.defaultClearance - 1e-9 and item.kind != "hole":
                self.fine[l, i0:i1, j0:j1][mask] = True
            if record:
                item.stamped.append((l, win, mask))

    def index(self, item, add):
        minx, miny, maxx, maxy = item.geom.bounds
        b = self.bucket
        for l in item.layers:
            for bx in range(int(minx // b), int(maxx // b) + 1):
                for by in range(int(miny // b), int(maxy // b) + 1):
                    s = self.buckets[l][(bx, by)]
                    if add:
                        s.add(item.iid)
                    else:
                        s.discard(item.iid)

    def query(self, layers, bounds):
        minx, miny, maxx, maxy = bounds
        b = self.bucket
        seen = set()
        out = []
        for l in layers:
            for bx in range(int(minx // b), int(maxx // b) + 1):
                for by in range(int(miny // b), int(maxy // b) + 1):
                    for iid in self.buckets[l].get((bx, by), ()):
                        if iid not in seen:
                            seen.add(iid)
                            out.append(self.items[iid])
        return out

    # ---- distance fields ---------------------------------------------------------
    def foreignDist(self, net, layer, win, reach=0.9, fixedOnly=False, nets=None, only=None):
        """Distance (mm) from each cell centre in win to the nearest copper of another net
        (`nets`: several nets count as own, e.g. both of a differential pair). only: "fine"
        counts just the copper of classes below the default clearance, "coarse" the rest."""
        # copper of no net (HARD) is nobody's: not even a pair's centreline, which searches
        # as net -1 to have both its nets' copper count as foreign
        own = tuple(n for n in (nets if nets else (net,)) if n != HARD)
        i0, j0, i1, j1 = win
        r = int(math.ceil(reach / self.g))
        I0, J0 = max(i0 - r, 0), max(j0 - r, 0)
        I1, J1 = min(i1 + r, self.H), min(j1 + r, self.W)
        src = self.fixed if fixedOnly else self.occ
        sub = src[layer, I0:I1, J0:J1] if layer is not None else src[:, I0:I1, J0:J1]
        if only is not None:
            fine = self.fine[layer, I0:I1, J0:J1] if layer is not None else self.fine[:, I0:I1, J0:J1]
            sub = np.where(fine if only == "coarse" else ~fine, 0, sub)
        if layer is None:  # any layer (through vias)
            free = ~((sub != 0) & ~np.isin(sub, own)).any(axis=0)
        else:
            free = (sub == 0) | np.isin(sub, own)
        if free.all():
            return np.full((i1 - i0, j1 - j0), 1e3, np.float32)
        d = edt(~free) * self.g
        return d[i0 - I0:i1 - I0, j0 - J0:j1 - J0]

    def maskDist(self, arr2d, win, reach=0.9, pred=None):
        i0, j0, i1, j1 = win
        r = int(math.ceil(reach / self.g))
        I0, J0 = max(i0 - r, 0), max(j0 - r, 0)
        I1, J1 = min(i1 + r, self.H), min(j1 + r, self.W)
        sub = arr2d[I0:I1, J0:J1]
        if pred is not None:
            sub = pred(sub)
        if not sub.any():
            return np.full((i1 - i0, j1 - j0), 1e3, np.float32)
        d = edt(sub) * self.g
        return d[i0 - I0:i1 - I0, j0 - J0:j1 - J0]

    # ---- exact checks ------------------------------------------------------------
    def violations(self, net, layers, geom, clearance, isVia=False, drill=0.0, holeToHole=0.25,
                   holeClearance=0.2, ignore=(), local=()):
        """Exact clearance check of a would-be item against everything stored. local:
        [(area, clearance)] the design rules set where either item touches the area (a
        BGA's courtyard), in place of the net classes'."""
        reach = 0.6
        near = [(a, c) for a, c in local if a.distance(geom) <= reach] if local else ()
        mine = [c for a, c in near if a.intersects(geom)]
        minx, miny, maxx, maxy = geom.bounds
        bad = []
        for other in self.query(layers, (minx - reach, miny - reach, maxx + reach, maxy + reach)):
            if other.iid in ignore:
                continue
            if other.kind == "hole":
                if other.net == net and not isVia:
                    continue
                # copper to hole edge; via drill to hole edge
                d = geom.distance(other.geom)
                need = holeClearance
                if isVia:
                    c = Point(geom.centroid)
                    need = max(need, 0)
                    dh = c.distance(other.geom) - drill / 2
                    if dh < holeToHole - 1e-4:
                        bad.append(other)
                        continue
                if d < need - 1e-4:
                    bad.append(other)
                continue
            if isVia and other.kind == "via":
                dh = Point(geom.centroid).distance(Point(other.pts[0])) - drill / 2 - other.drill / 2
                if dh < holeToHole - 1e-4:
                    bad.append(other)
                    continue
            if isVia and other.kind == "pad" and other.smd:
                if geom.intersects(other.geom):
                    bad.append(other)
                    continue
            if other.kind == "keepout":
                if (other.noVias if isVia else other.noTracks) and geom.intersects(other.geom):
                    bad.append(other)
                continue
            if other.net == net and net != HARD:
                continue
            need = max(clearance, other.clearance)
            if near:
                cs = mine or [c for a, c in near if a.intersects(other.geom)]
                if cs:
                    need = min(cs)
            if other.kind == "edge":
                need = self.edgeClearance
            if geom.distance(other.geom) < need - 1e-4:
                bad.append(other)
        return bad
