"""Right-angle corners are cut into two 45-degree bends.

    kicad-python tests/test_corners.py

Net A leaves its pad diagonally, steps a stub of a flat and comes back diagonally, a V
around net B's pad: the V's point gets a straight across it, kept clear of B's pad. Net C
turns a square corner: it gets a diagonal across it.
"""
import json
import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import pcbnew  # noqa: E402

from boardModel import loadModel  # noqa: E402
from router import Conn, Knobs, Route, Router  # noqa: E402

MM = 1e6
v = lambda x, y: pcbnew.VECTOR2I(int(round(x * MM)), int(round(y * MM)))
PRO = {
    "board": {"design_settings": {"rules": {"min_hole_clearance": 0.25, "min_hole_to_hole": 0.25,
                                             "min_copper_edge_clearance": 0.3}}},
    "net_settings": {"classes": [{"name": "Default", "track_width": 0.2, "clearance": 0.15,
                                  "via_diameter": 0.6, "via_drill": 0.3, "diff_pair_gap": 0.2}],
                     "netclass_patterns": []},
}
PADS = (("PA1", "A", (5, 5)), ("PA2", "A", (5, 13)), ("PB", "B", (5, 7.0)),
        ("PC1", "C", (12, 4)), ("PC2", "C", (16, 8)))


def build(path):
    b = pcbnew.BOARD()
    nets = {n: pcbnew.NETINFO_ITEM(b, n) for n in ("A", "B", "C")}
    for n in nets.values():
        b.Add(n)
    corners = [(0, 0), (20, 0), (20, 20), (0, 20)]
    for (ax, ay), (bx, by) in zip(corners, corners[1:] + corners[:1]):
        s = pcbnew.PCB_SHAPE(b)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(v(ax, ay))
        s.SetEnd(v(bx, by))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(int(0.1 * MM))
        b.Add(s)
    for ref, net, p in PADS:
        fp = pcbnew.FOOTPRINT(b)
        fp.SetReference(ref)
        b.Add(fp)
        pd = pcbnew.PAD(fp)
        pd.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
        pd.SetShape(pcbnew.PAD_SHAPE_RECT)
        pd.SetLayerSet(pd.SMDMask())
        pd.SetSize(v(1.0, 1.0))
        pd.SetPosition(v(*p))
        pd.SetNumber("1")
        pd.SetNet(nets[net])
        fp.Add(pd)
    b.Save(path)
    with open(path.replace(".kicad_pcb", ".kicad_pro"), "w") as f:
        json.dump(PRO, f)
    return pcbnew.LoadBoard(path)


def lay(r, net, pts, cid):
    pad = {p.ref: p for p in r.m.pads}
    a, b = [pad[ref] for ref, n, _ in PADS if n == net]
    conn = Conn(cid, net, a, b, "sig")
    rid = r.nextRid
    r.nextRid += 1
    items = [r.trackItem(net, 0, p, q, 0.2, 0.15, rid) for p, q in zip(pts, pts[1:])]
    for it in items:
        assert not r.check(it), f"test setup: {net} {it.pts} violates clearance"
        r.sp.add(it)
        r.netItems.setdefault(it.net, set()).add(it.iid)
    r.routes[rid] = Route(rid, conn, [it.iid for it in items], [], set())
    return r.routes[rid]


def polyline(r, route):
    segs = [r.sp.items[i].pts for i in route.items]
    return sorted(segs)


FAILS = []


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        FAILS.append(what)


if __name__ == "__main__":
    path = os.path.join(tempfile.mkdtemp(), "c.kicad_pcb")
    board = build(path)
    model = loadModel(board, path.replace(".kicad_pcb", ".kicad_pro"))
    model.planes = {}
    r = Router(model, Knobs(), lambda m: None)
    va = lay(r, "A", [(5, 5), (3, 7), (3.3, 7), (5.0, 8.7), (5, 13)], 1)
    sc = lay(r, "C", [(12, 4), (12, 8), (16, 8)], 2)
    n = r.chamferCorners()
    check(n == 2, f"two corners cut ({n})")
    segs = polyline(r, va)
    heads = {r.heading(p, q) for p, q in segs}
    check((0, 1) in heads or (0, -1) in heads, "A's V point became a straight down the side of B's pad")
    xs = [p[0] for p, q in segs for _ in (0,) if p[0] == q[0] and p[0] < 4.5]
    check(xs and max(xs) > 3.1, f"cut into the corner, not left at its point ({xs})")
    check(all(not r.check(r.sp.items[i]) for i in va.items + sc.items), "every new track keeps its clearance")
    segs = polyline(r, sc)
    diag = [s for s in segs if abs(abs(s[1][0] - s[0][0]) - abs(s[1][1] - s[0][1])) < 1e-6 and s[0] != s[1]]
    check(len(diag) == 1, "C's square corner got one diagonal across it")
    ends = sorted({p for s in segs for p in s})
    check((12, 4) in ends and (16, 8) in ends, "the pads' ends stay where they were")
    length = sum(math.dist(*s) for s in segs)
    check(length < 8.0, f"and the corner is shorter ({length:.2f} mm < 8)")
    if FAILS:
        print(f"FAIL: {len(FAILS)} checks")
        sys.exit(1)
    print("PASS: right-angle corners cut into two 45-degree bends")
