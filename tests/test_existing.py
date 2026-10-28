"""The board's own copper is joined the way KiCad joins it, and a transaction undoes.

    kicad-python tests/test_existing.py

1. Net A's two pads are joined by a track that ends 0.05 mm short of the second pad, its
   width covering the pad's edge (KiCad: connected). The router must see the connection
   as made, not route it again.
2. A route is laid, then in a transaction ripped and another laid instead; rollback puts
   the first back exactly and removes the second.
"""
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import pcbnew  # noqa: E402

from boardModel import existingCopper, loadModel  # noqa: E402
from router import Conn, Knobs, Router  # noqa: E402

MM = 1e6
v = lambda x, y: pcbnew.VECTOR2I(int(round(x * MM)), int(round(y * MM)))
PRO = {
    "board": {"design_settings": {"rules": {"min_hole_clearance": 0.25, "min_hole_to_hole": 0.25,
                                             "min_copper_edge_clearance": 0.3}}},
    "net_settings": {"classes": [{"name": "Default", "track_width": 0.2, "clearance": 0.15,
                                  "via_diameter": 0.6, "via_drill": 0.3, "diff_pair_gap": 0.2}],
                     "netclass_patterns": []},
}
PADS = (("PA1", "A", (5, 5)), ("PA2", "A", (10, 5)), ("PB1", "B", (5, 12)), ("PB2", "B", (12, 12)))


def build(path):
    b = pcbnew.BOARD()
    nets = {n: pcbnew.NETINFO_ITEM(b, n) for n in ("A", "B")}
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
    # A: from PA1's centre to 0.05 mm short of PA2's edge (x 9.5); 0.3 mm wide, so its
    # round end reaches 0.1 mm into the pad
    t = pcbnew.PCB_TRACK(b)
    t.SetStart(v(5, 5))
    t.SetEnd(v(9.45, 5))
    t.SetWidth(int(0.3 * MM))
    t.SetLayer(pcbnew.F_Cu)
    t.SetNet(nets["A"])
    b.Add(t)
    b.Save(path)
    with open(path.replace(".kicad_pcb", ".kicad_pro"), "w") as f:
        json.dump(PRO, f)
    return pcbnew.LoadBoard(path)


FAILS = []


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        FAILS.append(what)


if __name__ == "__main__":
    path = os.path.join(tempfile.mkdtemp(), "e.kicad_pcb")
    board = build(path)
    model = loadModel(board, path.replace(".kicad_pcb", ".kicad_pro"))
    model.planes = {}
    r = Router(model, Knobs(), lambda m: None)
    tracks, vias, _ = existingCopper(board)
    r.loadExisting(tracks, vias)
    pad = {p.ref: p for p in model.pads}
    a = Conn(1, "A", pad["PA1"], pad["PA2"], "sig")
    check(r.connected(a), "a track whose width covers a pad's edge joins it")

    b = Conn(2, "B", pad["PB1"], pad["PB2"], "sig")
    st, first = r.route(b, [0])
    check(st == "ok" and first is not None, "B is routed")
    before = sorted(tuple(r.sp.items[i].pts) for i in first.items)
    r.begin()
    r.rip(first.rid)
    st, second = r.route(b, [0, 1])
    check(st == "ok" and second is not None and second.rid != first.rid, "in the transaction, B is laid again")
    check(r.rollback(), "rollback succeeds")
    check(first.rid in r.routes and second.rid not in r.routes, "the first route is back, the second gone")
    check(sorted(tuple(r.sp.items[i].pts) for i in r.routes[first.rid].items) == before,
          "with the same copper")
    check(r.connected(b) and b.rid == first.rid, "and B is connected by it")
    if FAILS:
        print(f"FAIL: {len(FAILS)} checks")
        sys.exit(1)
    print("PASS: existing copper joins on overlap; a transaction rolls back")
