"""Review a routed board the way a person would look at it.

    kicad-python analyze.py <a.kicad_pcb> [<b.kicad_pcb> ...] [--plane GND]

Per board:
  direction   track length per outer layer by direction (0/45/90/135 deg), all tracks
              and long ones (> 3 mm) only: does each layer have a main direction?
  vias        signal/power vias, and how many have no ground via within 1.5 mm
              (their return current has no nearby path between the planes)
  reference   signal track length on an outer layer that does not run over the
              adjacent inner ground plane (plane voids, slots between via clearances);
              the clearance hole around the track's own via or through-hole pad is
              not counted
  pairs       differential pairs (found by name): length of each net, mismatch, and the
              share of length that runs coupled (beside its partner, same layer,
              edge gap 0.35 mm or less)
  pour gaps   areas on the outer layers with no copper at all, larger than 1 mm^2
              after ignoring clearance slivers: places a ground pour could not reach
  tidiness    how the tracks read: bends per 10 mm of track, jogs (a run that shifts
              sideways and carries on, within 1 mm), steps (any track piece under
              0.5 mm between two bends), and the share of track ends on SMD pads that
              land on the pad's centre
"""
import math
import sys
from collections import Counter

import numpy as np
import pcbnew
import shapely
from shapely.geometry import LineString, Point
from shapely.ops import unary_union

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from boardModel import polySetToShapely  # noqa: E402

NM = 1e6
OUTER = {"F.Cu": "In1.Cu", "B.Cu": "In2.Cu"}


def layerCopper(board, layerName):
    lid = board.GetLayerID(layerName)
    sps = pcbnew.SHAPE_POLY_SET()
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.IsOnLayer(lid):
                pad.TransformShapeToPolygon(sps, lid, 0, 5000, pcbnew.ERROR_OUTSIDE)
    for t in board.GetTracks():
        if t.IsOnLayer(lid):
            t.TransformShapeToPolygon(sps, lid, 0, 5000, pcbnew.ERROR_OUTSIDE)
    for z in board.Zones():
        if z.IsOnLayer(lid) and not z.GetIsRuleArea():
            sps.Append(z.GetFilledPolysList(lid))
    sps.Simplify()
    return polySetToShapely(sps)


def planeFill(board, layerName, net):
    lid = board.GetLayerID(layerName)
    parts = [polySetToShapely(z.GetFilledPolysList(lid)) for z in board.Zones()
             if z.IsOnLayer(lid) and z.GetNetname() == net and not z.GetIsRuleArea()]
    parts = [p for p in parts if p is not None]
    return unary_union(parts) if parts else None


def direction(board):
    out = {}
    for ln in OUTER:
        lid = board.GetLayerID(ln)
        allL, longL = Counter(), Counter()
        for t in board.GetTracks():
            if t.GetClass() == "PCB_VIA" or t.GetLayer() != lid:
                continue
            s, e = t.GetStart(), t.GetEnd()
            dx, dy = (e.x - s.x) / NM, (e.y - s.y) / NM
            L = math.hypot(dx, dy)
            if L == 0:
                continue
            a = round(math.degrees(math.atan2(-dy, dx)) / 45) * 45 % 180
            allL[a] += L
            if L > 3:
                longL[a] += L
        out[ln] = {k: {a: round(v) for a, v in sorted(c.items())} for k, c in (("all", allL), ("long", longL))}
    return out


def vias(board, plane):
    gnd, sig = [], []
    for t in board.GetTracks():
        if t.GetClass() != "PCB_VIA":
            continue
        p = t.GetPosition()
        (gnd if t.GetNetname() == plane else sig).append((p.x / NM, p.y / NM))
    lonely = 0
    if gnd:
        g = np.array(gnd)
        for (x, y) in sig:
            if np.min(np.hypot(g[:, 0] - x, g[:, 1] - y)) > 1.5:
                lonely += 1
    return dict(signal=len(sig), ground=len(gnd), noReturnVia=lonely)


def reference(board, plane):
    own = {}
    for t in board.GetTracks():
        if t.GetClass() == "PCB_VIA":
            own.setdefault(t.GetNetname(), []).append(t.GetPosition())
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.HasHole():
                own.setdefault(pad.GetNetname(), []).append(pad.GetPosition())
    own = {n: np.array([(p.x / NM, p.y / NM) for p in ps]) for n, ps in own.items()}
    out = {}
    for ln, ref in OUTER.items():
        fill = planeFill(board, ref, plane)
        lid = board.GetLayerID(ln)
        total = bad = 0.0
        for t in board.GetTracks():
            if t.GetClass() == "PCB_VIA" or t.GetLayer() != lid or t.GetNetname() == plane:
                continue
            s, e = t.GetStart(), t.GetEnd()
            seg = LineString([(s.x / NM, s.y / NM), (e.x / NM, e.y / NM)])
            total += seg.length
            if fill is None:
                bad += seg.length
                continue
            n = max(2, int(seg.length / 0.1) + 1)
            pts = [seg.interpolate(i / (n - 1), normalized=True) for i in range(n)]
            xs, ys = np.array([p.x for p in pts]), np.array([p.y for p in pts])
            outside = ~shapely.contains_xy(fill, xs, ys)
            holes = own.get(t.GetNetname())
            if holes is not None:
                d = np.min(np.hypot(xs[:, None] - holes[None, :, 0], ys[:, None] - holes[None, :, 1]), axis=1)
                outside &= d > 0.8
            bad += seg.length * outside.mean()
        out[ln] = dict(trackMm=round(total), overVoidMm=round(bad, 1))
    return out


def pairs(board):
    from strategy import pairPartners
    tracks = {}
    for t in board.GetTracks():
        if t.GetClass() == "PCB_VIA":
            continue
        s, e = t.GetStart(), t.GetEnd()
        tracks.setdefault(t.GetNetname(), []).append(
            (t.GetLayer(), LineString([(s.x / NM, s.y / NM), (e.x / NM, e.y / NM)]), t.GetWidth() / NM))
    partners = pairPartners(set(tracks))
    out = {}
    for a, b in partners.items():
        if a > b:
            continue
        lens = [sum(seg.length for _, seg, _ in tracks[n]) for n in (a, b)]
        total = coupled = 0.0
        for la, seg, wa in tracks[a]:
            n = max(2, int(seg.length / 0.1) + 1)
            for i in range(n):
                p = seg.interpolate(i / (n - 1), normalized=True)
                total += seg.length / n
                if any(lb == la and seg2.distance(p) <= wa / 2 + wb / 2 + 0.35 + 1e-6
                       for lb, seg2, wb in tracks[b]):
                    coupled += seg.length / n
        out[f"{a}/{b}"] = dict(mm=[round(x, 1) for x in lens], skewMm=round(abs(lens[0] - lens[1]), 2),
                               coupled=f"{100 * coupled / max(total, 1e-9):.0f}%")
    return out


def pourGaps(board, cell=0.1, minArea=1.0, sliver=0.35):
    """Empty areas per outer layer: board area with no copper, opened by `sliver` so
    clearance gaps between neighbouring copper do not count."""
    from scipy import ndimage
    outlineSet = pcbnew.SHAPE_POLY_SET()
    board.GetBoardPolygonOutlines(outlineSet, False)
    outline = polySetToShapely(outlineSet).buffer(-0.5)
    shapely.prepare(outline)
    minx, miny, maxx, maxy = outline.bounds
    xs = np.arange(minx, maxx, cell)
    ys = np.arange(miny, maxy, cell)
    X, Y = np.meshgrid(xs, ys)
    inBoard = shapely.contains_xy(outline, X, Y)
    out = {}
    for ln in OUTER:
        cu = layerCopper(board, ln)
        shapely.prepare(cu)
        empty = inBoard & ~shapely.contains_xy(cu, X, Y)
        r = int(round(sliver / cell))
        st = np.hypot(*np.mgrid[-r:r + 1, -r:r + 1]) <= r
        empty = ndimage.binary_opening(empty, structure=st)
        lab, n = ndimage.label(empty)
        areas = ndimage.sum(empty, lab, range(1, n + 1)) * cell * cell
        big = sorted((a for a in areas if a >= minArea), reverse=True)
        out[ln] = dict(gaps=len(big), areaMm2=round(sum(big), 1), largest=[round(a, 1) for a in big[:5]])
    return out


def tidiness(board):
    segs = []
    for t in board.GetTracks():
        if t.GetClass() != "PCB_TRACK" or t.GetLength() < 1e3:
            continue
        s, e = t.GetStart(), t.GetEnd()
        segs.append((t.GetNetname(), t.GetLayer(), (s.x / NM, s.y / NM), (e.x / NM, e.y / NM)))
    total = sum(math.dist(p, q) for _, _, p, q in segs)
    ends = {}
    for i, (net, layer, p, q) in enumerate(segs):
        for pt in (p, q):
            ends.setdefault((net, layer, round(pt[0], 4), round(pt[1], 4)), []).append(i)

    def heading(p, q):
        dx, dy = q[0] - p[0], q[1] - p[1]
        return (0 if abs(dx) < 1e-6 else (1 if dx > 0 else -1), 0 if abs(dy) < 1e-6 else (1 if dy > 0 else -1))

    def other(i, pt):
        _, _, p, q = segs[i]
        return q if math.dist(p, pt) < 1e-4 else p

    def bendAt(key):
        """True when exactly two pieces meet here at an angle."""
        ids = ends[key]
        if len(ids) != 2:
            return False
        pt = (key[2], key[3])
        a, b = (other(i, pt) for i in ids)
        return heading(pt, a) != tuple(-x for x in heading(pt, b))
    bends = sum(1 for key in ends if bendAt(key))
    jogs = steps = 0
    for i, (net, layer, p, q) in enumerate(segs):
        kp, kq = (net, layer, round(p[0], 4), round(p[1], 4)), (net, layer, round(q[0], 4), round(q[1], 4))
        if not (bendAt(kp) and bendAt(kq)):
            continue
        L = math.dist(p, q)
        if L < 0.5:
            steps += 1
        if L < 1.0:
            before = other(next(j for j in ends[kp] if j != i), p)
            after = other(next(j for j in ends[kq] if j != i), q)
            if heading(before, p) == heading(q, after):
                jogs += 1
    centred = onPad = 0
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() != pcbnew.PAD_ATTRIB_SMD:
                continue
            poly = polySetToShapely(pad.GetEffectivePolygon(pad.GetLayer()))
            c = pad.GetCenter()
            c = (c.x / NM, c.y / NM)
            for key, ids in ends.items():
                if key[0] != pad.GetNetname() or len(ids) != 1 or key[1] != pad.GetLayer():
                    continue
                pt = (key[2], key[3])
                if poly.contains(Point(pt)):
                    onPad += 1
                    centred += math.dist(pt, c) < 0.05
    return dict(bendsPer10mm=round(10 * bends / max(total, 1e-9), 2), jogs=jogs, steps=steps,
                padEndsCentred=f"{centred}/{onPad}")


def analyze(path, plane="GND"):
    b = pcbnew.LoadBoard(path)
    return dict(direction=direction(b), vias=vias(b, plane), reference=reference(b, plane),
                pairs=pairs(b), pourGaps=pourGaps(b), tidiness=tidiness(b))


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    plane = sys.argv[sys.argv.index("--plane") + 1] if "--plane" in sys.argv else "GND"
    args = [a for a in args if a != plane]
    for p in args:
        print(p)
        for k, v in analyze(p, plane).items():
            print(f"  {k}: {v}")
