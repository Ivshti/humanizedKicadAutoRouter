"""Copper an earlier run drew may be moved; the designer's may not.

    kicad-python tests/test_soft_copper.py

Net B runs straight across the board on the top layer, between net A's two pads. A is
held to the top layer, so it can only get through by taking B's place. When B's track
is the router's own (listed as soft), push and shove rips it and routes B again another
way; when it is the designer's, A stays unrouted and B is untouched.
"""
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import pcbnew  # noqa: E402

from boardModel import B, F, existingCopper, loadModel  # noqa: E402
from router import Knobs, Router  # noqa: E402
from run import negotiate  # noqa: E402
from strategy import plan  # noqa: E402

MM = 1e6
v = lambda x, y: pcbnew.VECTOR2I(int(round(x * MM)), int(round(y * MM)))
PRO = {
    "board": {"design_settings": {"rules": {"min_hole_clearance": 0.25, "min_hole_to_hole": 0.25,
                                             "min_copper_edge_clearance": 0.3}}},
    "net_settings": {"classes": [{"name": "Default", "track_width": 0.2, "clearance": 0.15,
                                  "via_diameter": 0.6, "via_drill": 0.3, "diff_pair_gap": 0.2}],
                     "netclass_patterns": []},
}


def build(path):
    b = pcbnew.BOARD()
    nets = {n: pcbnew.NETINFO_ITEM(b, n) for n in ("A", "B")}
    for n in nets.values():
        b.Add(n)
    corners = [(0, 0), (30, 0), (30, 30), (0, 30)]
    for (ax, ay), (bx, by) in zip(corners, corners[1:] + corners[:1]):
        s = pcbnew.PCB_SHAPE(b)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(v(ax, ay))
        s.SetEnd(v(bx, by))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(int(0.1 * MM))
        b.Add(s)
    for ref, net, p in (("PA1", "A", (5, 15)), ("PA2", "A", (25, 15)), ("PB1", "B", (15, 0.9)), ("PB2", "B", (15, 29.1))):
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
    t = pcbnew.PCB_TRACK(b)
    t.SetStart(v(15, 0.9))
    t.SetEnd(v(15, 29.1))
    t.SetWidth(int(0.2 * MM))
    t.SetLayer(pcbnew.F_Cu)
    t.SetNet(nets["B"])
    b.Add(t)
    b.Save(path)
    with open(path.replace(".kicad_pcb", ".kicad_pro"), "w") as f:
        json.dump(PRO, f)
    return pcbnew.LoadBoard(path)


def attempt(soft):
    tmp = tempfile.mkdtemp()
    path = os.path.join(tmp, "s.kicad_pcb")
    b = build(path)
    tracks, vias, uuids = existingCopper(b)
    model = loadModel(b, path.replace(".kicad_pcb", ".kicad_pro"))
    model.planes = {}
    r = Router(model, Knobs(), lambda m: None)
    r.loadExisting(tracks, vias, uuids if soft else set())
    conns, _, _ = plan(model, r, lambda m: None, [])
    r.allConns = conns
    a = next(c for c in conns if c.net == "A")
    b_ = next(c for c in conns if c.net == "B")
    a.strict, a.prefLayer = True, F          # A may not leave the top layer
    import run
    run.layersFor = lambda c: [F] if c.strict else [F, B]
    negotiate(r, [a], lambda m: None, maxRounds=10)
    return r, a, b_, uuids


FAILS = []


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        FAILS.append(what)


if __name__ == "__main__":
    r, a, b_, uuids = attempt(soft=True)
    check(r.connected(a), "soft: A gets through on the top layer")
    check(r.connected(b_), "soft: B is routed again another way")
    check(uuids <= r.droppedUuids, "soft: B's old track is dropped from the board")
    r, a, b_, uuids = attempt(soft=False)
    check(not r.connected(a), "designer's: A does not get through")
    check(r.connected(b_) and not r.droppedUuids, "designer's: B's track is untouched")
    if FAILS:
        print(f"FAIL: {len(FAILS)} checks")
        sys.exit(1)
    print("PASS: router copper moves, the designer's stays")
