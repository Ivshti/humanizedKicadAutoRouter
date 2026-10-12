"""Corridors: where each bundle runs, decided on a coarse grid before any of its tracks.

A person wiring block to block first decides the channels ("this bus goes down the left
edge on the bottom, that one rings the antenna on top") and only then lays tracks. Here
every bundle is split into groups whose ends sit together, and each group is routed on a
coarse tile grid whose capacity is the room left after power and block wiring. Congestion
is negotiated (overfull tiles get dearer each round), and a tile another group already
runs through is cheaper while it has room, so groups merge into shared trunks the way a
person runs several buses side by side. Running across other wiring on the same layer
costs what it costs on the board, a via pair per member, so crossing bundles end up on
opposite layers. As on a hand-routed board, each outer layer has a main diagonal and the
two layers run crosswise: travelling along the other layer's diagonal costs extra, so
long runs that head one way share a layer and cross the other layer's runs at right
angles. Which layer gets which diagonal is planned both ways and the cheaper kept. The
detailed router then pays extra to leave its group's corridor, and the corridor's layer
replaces a per-bundle layer guess.
"""
import ctypes
import math
from collections import defaultdict

import numpy as np

from boardModel import B, F
from native import edt

LAYERS = (F, B)


def octile(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)


def axisOf(dx, dy):
    """0 horizontal, 1 diagonal \\, 2 vertical, 3 diagonal / (y grows downwards)."""
    if abs(dy) < 1e-9:
        return 0
    if abs(dx) < 1e-9:
        return 2
    return 1 if dx * dy > 0 else 3


def splitBundles(bundles, blockOf, joinMm, maxGroup):
    """Groups of one bundle whose two ends are each within joinMm of a neighbour's, at most
    maxGroup wide (wider groups are cut across their width)."""
    groups = []
    for name, cs in bundles.items():
        first = name.split("|")[0]
        ends = []
        for c in cs:
            a, b = (c.a, c.b) if blockOf[c.a.ref] == first else (c.b, c.a)
            ends.append((a.center, b.center))
        parent = list(range(len(cs)))

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i
        for i in range(len(cs)):
            for j in range(i + 1, len(cs)):
                if max(math.dist(ends[i][0], ends[j][0]), math.dist(ends[i][1], ends[j][1])) < joinMm:
                    parent[find(i)] = find(j)
        clusters = defaultdict(list)
        for i in range(len(cs)):
            clusters[find(i)].append(i)
        for idx in clusters.values():
            # cut wide groups across the run: order by position across the a->b direction
            ax = np.mean([ends[i][0][0] for i in idx]); ay = np.mean([ends[i][0][1] for i in idx])
            bx = np.mean([ends[i][1][0] for i in idx]); by = np.mean([ends[i][1][1] for i in idx])
            nx, ny = -(by - ay), bx - ax
            idx.sort(key=lambda i: ends[i][0][0] * nx + ends[i][0][1] * ny)
            for s in range(0, len(idx), maxGroup):
                groups.append([cs[i] for i in idx[s:s + maxGroup]])
    return groups


class Corridors:
    def __init__(self, router, tileMm=2.0):
        self.r = router
        sp = router.sp
        self.s = max(1, int(round(tileMm / sp.g)))
        self.T = self.s * sp.g
        self.TH = -(-sp.H // self.s)
        self.TW = -(-sp.W // self.s)

    # ---- capacity --------------------------------------------------------------------
    def capacity(self):
        """Signal tracks that fit through each tile per outer layer, from the copper and
        keep-outs already on the board."""
        r, sp, k = self.r, self.r.sp, self.r.k
        rule = r.rules[next(n for n, c in r.m.netClass.items() if c == "Default")]
        pitch = k.signalWidth + rule.clearance
        need = k.signalWidth / 2 + rule.clearance + k.margin
        cap = np.zeros((2, self.TH, self.TW), np.float32)
        for li, l in enumerate(LAYERS):
            blocked = (sp.occ[l] != 0) | r.bodyKeep[l] | sp.trackKeep[l]
            free = edt(blocked) * sp.g >= need
            cap[li] = self.pool(free) * self.T / pitch
        self.axisUse = np.zeros((2, self.TH, self.TW, 4), np.float32)
        for it in sp.items.values():
            if it.kind != "track" or it.layers[0] not in LAYERS or it.net == r.netId.get("GND"):
                continue
            (x1, y1), (x2, y2) = it.pts
            L = math.hypot(x2 - x1, y2 - y1)
            if L < self.T / 2:
                continue
            li, a = LAYERS.index(it.layers[0]), axisOf(x2 - x1, y2 - y1)
            for t in {self.tileOf((x1 + (x2 - x1) * f, y1 + (y2 - y1) * f))
                      for f in np.linspace(0, 1, int(L / (self.T / 3)) + 2)}:
                self.axisUse[(li, *t, a)] = max(self.axisUse[(li, *t, a)], 1.0)
        viaFree = ~(sp.viaKeep | r.bodyViaKeep)
        self.viaok = (self.pool(viaFree) > 0.2).astype(np.uint8)
        return cap

    def pool(self, fine):
        """Mean of a fine boolean raster over each tile."""
        s = self.s
        pad = np.zeros((self.TH * s, self.TW * s), np.float32)
        pad[:fine.shape[0], :fine.shape[1]] = fine
        return pad.reshape(self.TH, s, self.TW, s).mean(axis=(1, 3))

    def tileOf(self, p):
        i, j = self.r.sp.toCell(*p)
        return min(max(i // self.s, 0), self.TH - 1), min(max(j // self.s, 0), self.TW - 1)

    def endTiles(self, pads):
        out = set()
        for p in pads:
            ti, tj = self.tileOf(p.center)
            for li, l in enumerate(LAYERS):
                if l in p.layers:
                    out.add((li, ti, tj))
        return out

    # ---- one group -------------------------------------------------------------------
    def pathAxes(self, path):
        """(layer, ti, tj, axis) for each step of a path that stays on its layer."""
        out = set()
        for a, b in zip(path, path[1:]):
            if a[0] == b[0]:
                ax = axisOf(b[2] - a[2], b[1] - a[1])
                out.add((*a, ax))
                out.add((*b, ax))
        return out

    def dirCost(self, axisUse, g):
        """Entering a tile along an axis that is not the axis of wiring already there on
        that layer is a crossing: a via pair per member. The ends' own tiles are free
        (that is where a bundle fans into its pads)."""
        k = self.r.k
        busy = axisUse > 0
        other = np.stack([busy[..., [b for b in range(4) if b != a]].any(axis=-1) for a in range(4)], -1)
        dc = other.astype(np.float32) * (2 * k.corridorViaMm / self.T)
        for (li, ti, tj) in g["src"] | g["dst"]:
            dc[li, ti, tj] = 0
        return np.ascontiguousarray(dc)

    def searchGroup(self, g, cost, dirCost, layerDir):
        k = self.r.k
        idx = lambda li, ti, tj: (li * self.TH + ti) * self.TW + tj
        src = np.array([idx(*t) for t in g["src"]], np.int32)
        tgt = np.zeros(2 * self.TH * self.TW, np.uint8)
        for t in g["dst"]:
            tgt[idx(*t)] = 1
        c = cost.copy()
        for (li, ti, tj) in g["src"] | g["dst"]:
            c[li, ti, tj] = min(c[li, ti, tj], 1.0)
        tis = [t[1] for t in g["dst"]]
        tjs = [t[2] for t in g["dst"]]
        out = np.empty(4 * self.TH * self.TW + 16, np.int32)
        n = self.r.kernel.astar(2, self.TH, self.TW, np.ascontiguousarray(c).ctypes.data,
                                self.viaok.ctypes.data, ctypes.c_float(k.corridorViaMm / self.T),
                                tgt.ctypes.data, None, src.ctypes.data, len(src),
                                ctypes.c_float(0.05), ctypes.c_float(0.25), ctypes.c_float(1.0),
                                min(tjs), min(tis), max(tjs), max(tis), out.ctypes.data, len(out),
                                ctypes.c_int64(10_000_000), None, dirCost.ctypes.data,
                                ctypes.c_float(k.corridorViaMm / self.T), layerDir.ctypes.data)
        if n <= 0:
            return None
        path = []
        for f in out[:n].tolist():
            li, rem = divmod(f, self.TH * self.TW)
            path.append((li, *divmod(rem, self.TW)))
        return path

    def pathCost(self, path, mainAxis):
        """Millimetres a member pays along a planned path: length, against-diagonal
        surcharge and layer changes."""
        k = self.r.k
        mm = 0.0
        for a, b in zip(path, path[1:]):
            if a[0] != b[0]:
                mm += k.corridorViaMm
                continue
            ax = axisOf(b[2] - a[2], b[1] - a[1])
            step = self.T * (math.sqrt(2) if ax in (1, 3) else 1.0)
            mm += step * (1 + (k.againstCost if ax == 4 - mainAxis[a[0]] else 0.0))
        return mm

    # ---- negotiation -----------------------------------------------------------------
    def negotiate(self, groups, mainAxis, iters):
        """Corridors for all groups with the given main diagonal per layer (index 0 F,
        1 B). Returns the groups with their paths and the plan's cost."""
        k = self.r.k
        cap = self.cap
        hist = np.zeros_like(cap)
        use = np.zeros_like(cap)
        axisUse = self.axisUse.copy()
        layerDir = np.zeros((2, 4), np.float32)
        for li in range(2):
            layerDir[li, 4 - mainAxis[li]] = k.againstCost
        gs = []
        for cs in groups:
            ends = [(c.a, c.b) for c in cs]
            gs.append(dict(conns=cs, n=len(cs), path=None,
                           src=self.endTiles([a for a, _ in ends]),
                           dst=self.endTiles([b for _, b in ends])))
        gs.sort(key=lambda g: -g["n"])
        pres = 0.5
        for it in range(iters):
            todo = [g for g in gs if g["path"] is None]
            for g in todo:
                over = np.maximum(0.0, use + g["n"] - cap)
                cost = (1.0 + hist) * (1.0 + pres * over / g["n"])
                cost[(use > 0) & (over == 0)] *= k.corridorShare
                cost[cap < 0.25] *= 20.0
                g["path"] = self.searchGroup(g, cost.astype(np.float32), self.dirCost(axisUse, g),
                                             layerDir)
                if g["path"] is not None:
                    for t in set(g["path"]):
                        use[t] += g["n"]
                    for t in self.pathAxes(g["path"]):
                        axisUse[t] += g["n"]
            overflow = np.maximum(0.0, use - cap)
            hot = overflow > 0.5
            nHot = int(hot.sum())
            if not nHot or it == iters - 1:
                break
            hist[hot] += 0.4 * overflow[hot] / np.maximum(cap[hot], 1.0)
            pres *= 1.7
            for g in gs:
                if g["path"] is not None and any(hot[t] for t in g["path"]):
                    for t in set(g["path"]):
                        use[t] -= g["n"]
                    for t in self.pathAxes(g["path"]):
                        axisUse[t] -= g["n"]
                    g["path"] = None
        # a group left without a corridor or a tile left overfull costs what a detour would
        cost = sum(g["n"] * (self.pathCost(g["path"], mainAxis) if g["path"] else 100.0) for g in gs)
        cost += float(overflow.sum()) * 10.0
        return gs, cost, dict(rounds=it + 1, hot=nHot)

    def chooseAxes(self, groups, log, iters=15):
        """Which diagonal each outer layer runs along, decided before anything is routed
        (power and long block wiring follow it too): every bundle is planned both ways
        on the bare board and the cheaper way is kept."""
        names = {1: "\\", 3: "/"}
        forced = {"/": (3, 1), "\\": (1, 3)}.get(self.r.k.topDiagonal)
        if forced:
            mainAxis = forced
            log(f"main diagonals: top {names[mainAxis[0]]}, bottom {names[mainAxis[1]]} (set)")
        else:
            self.cap = self.capacity()
            costs = {ax: self.negotiate(groups, ax, iters)[1] for ax in ((3, 1), (1, 3))}
            mainAxis = min(costs, key=costs.get)
            log(f"main diagonals: top {names[mainAxis[0]]}, bottom {names[mainAxis[1]]} "
                f"(planned cost {costs[mainAxis]:.0f} vs {costs[mainAxis[::-1]]:.0f} mm)")
        self.r.mainAxis = {F: mainAxis[0], B: mainAxis[1]}

    def plan(self, groups, log, iters=15):
        """Plan every group's corridor on the board as routed so far."""
        self.cap = self.capacity()
        mainAxis = (self.r.mainAxis[F], self.r.mainAxis[B])
        gs, cost, st = self.negotiate(groups, mainAxis, iters)
        planned = [g for g in gs if g["path"]]
        nVia = sum(g["n"] * sum(1 for a, b in zip(g["path"], g["path"][1:]) if a[0] != b[0])
                   for g in planned)
        onB = sum(g["n"] for g in planned if sum(t[0] for t in g["path"]) * 2 > len(g["path"]))
        log(f"corridors: {len(gs)} groups, {st['rounds']} rounds, {st['hot']} overfull tiles left, "
            f"~{nVia} planned vias, {onB}/{sum(g['n'] for g in gs)} members mostly on bottom")
        for rank, g in enumerate(gs):
            mask = self.corridorMask(g)
            viaZone = self.viaZone(g)
            for pos, c in enumerate(g["conns"]):
                c.corridor = mask
                c.corridorVia = viaZone
                c.groupRank, c.groupPos = rank, pos
                c.prefLayer = -1
        return gs

    def corridorMask(self, g):
        """Tiles of the path, one tile either side, both layers around layer changes, and
        the area around each end's pads."""
        m = np.zeros((2, self.TH, self.TW), bool)
        if g["path"] is None:
            return None
        path = g["path"]
        for (li, ti, tj) in path:
            m[li, max(ti - 1, 0):ti + 2, max(tj - 1, 0):tj + 2] = True
        for a, b in zip(path, path[1:]):
            if a[0] != b[0]:
                m[:, max(a[1] - 1, 0):a[1] + 2, max(a[2] - 1, 0):a[2] + 2] = True
        for ends in (g["src"], g["dst"]):
            ti = [t[1] for t in ends]
            tj = [t[2] for t in ends]
            m[:, max(min(ti) - 1, 0):max(ti) + 2, max(min(tj) - 1, 0):max(tj) + 2] = True
        return m

    def viaZone(self, g):
        """Tiles where the plan changes layer (and one tile around): with the escapes next
        to the pads, the only places a member may take a via."""
        if g["path"] is None:
            return None
        z = np.zeros((self.TH, self.TW), bool)
        for a, b in zip(g["path"], g["path"][1:]):
            if a[0] != b[0]:
                z[max(a[1] - 1, 0):a[1] + 2, max(a[2] - 1, 0):a[2] + 2] = True
        return z

    def fine(self, mask, li, win):
        """Corridor of one layer over a fine-raster window."""
        i0, j0, i1, j1 = win
        ti = np.minimum(np.arange(i0, i1) // self.s, self.TH - 1)
        tj = np.minimum(np.arange(j0, j1) // self.s, self.TW - 1)
        return (mask if li is None else mask[li])[np.ix_(ti, tj)]
