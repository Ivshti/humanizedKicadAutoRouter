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


def analyze(path, plane="GND"):
    b = pcbnew.LoadBoard(path)
    return dict(direction=direction(b), vias=vias(b, plane), reference=reference(b, plane),
                pairs=pairs(b), pourGaps=pourGaps(b))


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    plane = sys.argv[sys.argv.index("--plane") + 1] if "--plane" in sys.argv else "GND"
    args = [a for a in args if a != plane]
    for p in args:
        print(p)
        for k, v in analyze(p, plane).items():
            print(f"  {k}: {v}")
