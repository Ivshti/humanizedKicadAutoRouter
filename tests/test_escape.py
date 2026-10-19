"""A differential pair leaves a BGA the way a person fans it out.

    kicad-python tests/test_escape.py

A 6 x 6 ball grid at 0.5 mm pitch, every ball ground except two pairs that sit in the
outer two rows of its north side (N on the outer row, P on the one inside it), and a
design rule that sets 0.08 mm clearance inside its courtyard (one track fits between two
balls). Each pair escapes north: N straight out, P one diagonal step into the gap beside
its N and then straight out, so past the courtyard the four tracks run N, P, N, P at
0.25 mm. Without the courtyard rule a track does not fit beside a ground ball.
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
from router import Knobs, Router  # noqa: E402

MM = 1e6
v = lambda x, y: pcbnew.VECTOR2I(int(round(x * MM)), int(round(y * MM)))
PRO = {
    "board": {"design_settings": {"rules": {"min_hole_clearance": 0.25, "min_hole_to_hole": 0.25,
                                             "min_copper_edge_clearance": 0.3}}},
    "net_settings": {"classes": [{"name": "Default", "track_width": 0.15, "clearance": 0.15,
                                  "via_diameter": 0.6, "via_drill": 0.3, "diff_pair_gap": 0.15},
                                 {"name": "LVDS", "track_width": 0.08, "clearance": 0.08,
                                  "via_diameter": 0.45, "via_drill": 0.2, "diff_pair_gap": 0.1}],
                     "netclass_patterns": [{"pattern": "LV_*", "netclass": "LVDS"}]},
}
PITCH, X0, Y0 = 0.5, 10.0, 10.0
PAIRS = {(1, 0): "LV_A_N", (1, 1): "LV_A_P", (2, 0): "LV_B_N", (2, 1): "LV_B_P"}   # (col, row)


def build(path):
    b = pcbnew.BOARD()
    nets = {n: pcbnew.NETINFO_ITEM(b, n) for n in ["GND"] + list(PAIRS.values())}
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
    fp = pcbnew.FOOTPRINT(b)
    fp.SetReference("U1")
    fp.SetPosition(v(X0 + 1.25, Y0 + 1.25))
    b.Add(fp)
    n = 0
    for col in range(6):
        for row in range(6):
            n += 1
            pd = pcbnew.PAD(fp)
            pd.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
            pd.SetShape(pcbnew.PAD_SHAPE_CIRCLE)
            pd.SetLayerSet(pd.SMDMask())
            pd.SetSize(v(0.25, 0.25))
            pd.SetPosition(v(X0 + col * PITCH, Y0 + row * PITCH))
            pd.SetNumber(str(n))
            pd.SetNet(nets[PAIRS.get((col, row), "GND")])
            fp.Add(pd)
    lo, hi = -0.6, 5 * PITCH + 0.6
    pts = [(lo, lo), (hi, lo), (hi, hi), (lo, hi)]
    for (ax, ay), (bx, by) in zip(pts, pts[1:] + pts[:1]):
        s = pcbnew.PCB_SHAPE(fp)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(v(X0 + ax, Y0 + ay))
        s.SetEnd(v(X0 + bx, Y0 + by))
        s.SetLayer(pcbnew.F_CrtYd)
        s.SetWidth(int(0.05 * MM))
        fp.Add(s)
    b.Save(path)
    with open(path.replace(".kicad_pcb", ".kicad_pro"), "w") as f:
        json.dump(PRO, f)
    return pcbnew.LoadBoard(path)


def router(withRule):
    tmp = tempfile.mkdtemp()
    path = os.path.join(tmp, "e.kicad_pcb")
    b = build(path)
    model = loadModel(b, path.replace(".kicad_pcb", ".kicad_pro"))
    model.planes = {}
    r = Router(model, Knobs(), lambda m: None)
    if withRule:
        r.setCourtyardClearances({"U1": 0.08})
    return r, model


FAILS = []


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        FAILS.append(what)


def escapes(r, model):
    pad = {p.net: p for p in model.pads if p.net in PAIRS.values()}
    out = {}
    for pair in ("LV_A", "LV_B"):
        out[pair] = r.pairEscape(pad[pair + "_N"], pad[pair + "_P"], r.rules[pair + "_N"])
    return out


if __name__ == "__main__":
    r, model = router(True)
    check(r.gridArray("U1") is not None, "a 6 x 6 grid at 0.5 mm is a ball grid")
    esc = escapes(r, model)
    check(all(e is not None for e in esc.values()), "both pairs escape with the courtyard rule")
    if all(e is not None for e in esc.values()):
        a = esc["LV_A"]
        check(a["u"] == (0, -1), "the way out is north, from the inner ball to the outer one")
        n, p = a["paths"]["LV_A_N"], a["paths"]["LV_A_P"]
        check(len(n) == 2 and n[0][0] == n[1][0], "the outer ball's net runs straight out")
        check(len(p) == 3 and math.isclose(p[1][0] - p[0][0], PITCH / 2) and math.isclose(p[0][1] - p[1][1], PITCH / 2),
              "the inner ball's net steps diagonally into the gap a quarter turn from the way out (east)")
        ends = sorted((e["paths"][net][-1][0], net) for e in esc.values() for net in e["paths"])
        check([net for _, net in ends] == ["LV_A_N", "LV_A_P", "LV_B_N", "LV_B_P"],
              "past the courtyard they run N, P, N, P")
        check(all(math.isclose(b_[0] - a_[0], PITCH / 2) for a_, b_ in zip(ends, ends[1:])),
              "at half the ball pitch")
        top = Y0 - 0.6
        check(all(e["paths"][net][-1][1] < top for e in esc.values() for net in e["paths"]),
              "and end past the courtyard")
    r, model = router(False)
    esc = escapes(r, model)
    check(esc["LV_B"] is None, "without the rule, a track does not fit beside a ground ball "
                               "(and the pair does not take the other side, its neighbour's gap)")
    if FAILS:
        print(f"FAIL: {len(FAILS)} checks")
        sys.exit(1)
    print("PASS: pairs escape the ball grid N, P, N, P")
