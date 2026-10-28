"""The designer's track is nudged aside to let a blocked route through, as a person would.

    kicad-python tests/test_nudge.py

Two long pads of other nets leave a 1.0 mm channel. The designer's net B runs down its
middle; net A, held to the top layer, has to get through the same channel: with B in the
middle there is no room beside it (0.2 mm tracks at 0.15 mm), with B moved to one side
there is room for both. With the nudge knob A gets through and B is laid again beside it: still
connected, on its own layer, no via, near its old length, and the two keep the design
clearance. Without it A stays unrouted and B is untouched.
"""
import json
import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import pcbnew  # noqa: E402
from shapely.geometry import LineString  # noqa: E402

from boardModel import existingCopper, loadModel  # noqa: E402
from router import Knobs, Router  # noqa: E402
from run import negotiate  # noqa: E402
from strategy import plan  # noqa: E402

MM = 1e6
v = lambda x, y: pcbnew.VECTOR2I(int(round(x * MM)), int(round(y * MM)))
CLR = 0.15
PRO = {
    "board": {"design_settings": {"rules": {"min_hole_clearance": 0.25, "min_hole_to_hole": 0.25,
                                             "min_copper_edge_clearance": 0.3}}},
    "net_settings": {"classes": [{"name": "Default", "track_width": 0.2, "clearance": CLR,
                                  "via_diameter": 0.6, "via_drill": 0.3, "diff_pair_gap": 0.2}],
                     "netclass_patterns": []},
}
B_PATH = [(1.0, 7.0), (7.0, 5.0), (13.0, 5.0), (19.0, 7.0)]


def keepout(b, x0, y0, x1, y1):
    z = pcbnew.ZONE(b)
    z.SetIsRuleArea(True)
    z.SetDoNotAllowTracks(True)
    z.SetDoNotAllowVias(True)
    z.SetDoNotAllowZoneFills(True)
    z.SetDoNotAllowPads(False)
    z.SetDoNotAllowFootprints(False)
    ls = pcbnew.LSET()
    ls.AddLayer(pcbnew.F_Cu)
    ls.AddLayer(pcbnew.B_Cu)
    z.SetLayerSet(ls)
    o = z.Outline()
    o.NewOutline()
    for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
        o.Append(int(x * MM), int(y * MM))
    b.Add(z)


def build(path):
    b = pcbnew.BOARD()
    nets = {n: pcbnew.NETINFO_ITEM(b, n) for n in ("A", "B", "W1", "W2")}
    for n in nets.values():
        b.Add(n)
    corners = [(0, 0), (20, 0), (20, 10), (0, 10)]
    for (ax, ay), (bx, by) in zip(corners, corners[1:] + corners[:1]):
        s = pcbnew.PCB_SHAPE(b)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(v(ax, ay))
        s.SetEnd(v(bx, by))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(int(0.1 * MM))
        b.Add(s)
    for ref, net, p, size in (("PA1", "A", (1.0, 3.0), (0.8, 0.8)), ("PA2", "A", (19.0, 3.0), (0.8, 0.8)),
                              ("PB1", "B", B_PATH[0], (0.8, 0.8)), ("PB2", "B", B_PATH[-1], (0.8, 0.8)),
                              # the channel's walls, y 0.5 .. 4.5 and 5.5 .. 9.5 at x 8 .. 12
                              ("W1", "W1", (10.0, 2.5), (4.0, 4.0)), ("W2", "W2", (10.0, 7.5), (4.0, 4.0))):
        fp = pcbnew.FOOTPRINT(b)
        fp.SetReference(ref)
        b.Add(fp)
        pd = pcbnew.PAD(fp)
        pd.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
        pd.SetShape(pcbnew.PAD_SHAPE_RECT)
        pd.SetLayerSet(pd.SMDMask())
        pd.SetSize(v(*size))
        pd.SetPosition(v(*p))
        pd.SetNumber("1")
        pd.SetNet(nets[net])
        fp.Add(pd)
    for a, c in zip(B_PATH, B_PATH[1:]):
        t = pcbnew.PCB_TRACK(b)
        t.SetStart(v(*a))
        t.SetEnd(v(*c))
        t.SetWidth(int(0.2 * MM))
        t.SetLayer(pcbnew.F_Cu)
        t.SetNet(nets["B"])
        b.Add(t)
    b.Save(path)
    with open(path.replace(".kicad_pcb", ".kicad_pro"), "w") as f:
        json.dump(PRO, f)
    return pcbnew.LoadBoard(path)


def attempt(nudge):
    tmp = tempfile.mkdtemp()
    path = os.path.join(tmp, "n.kicad_pcb")
    b = build(path)
    tracks, vias, uuids = existingCopper(b)
    model = loadModel(b, path.replace(".kicad_pcb", ".kicad_pro"))
    model.planes = {}
    r = Router(model, Knobs(nudge=nudge), lambda m: None)
    r.loadExisting(tracks, vias, set())          # B is the designer's
    conns, _, _ = plan(model, r, lambda m: None, [])
    r.allConns = conns
    r.markMovable()
    a = next(c for c in conns if c.net == "A")
    bconn = next(c for c in conns if c.net == "B")
    for c in (a, bconn):
        c.strict, c.prefLayer = True, 0          # both held to the top layer
    import run
    run.layersFor = lambda c: [0] if c.strict else [0, 3]
    negotiate(r, [a], lambda m: None, maxRounds=10)
    return r, a, bconn, uuids


FAILS = []


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        FAILS.append(what)


def copper(r, net):
    nid = r.netId[net]
    return [it for it in r.sp.items.values() if it.net == nid and it.kind in ("track", "via")]


if __name__ == "__main__":
    r, a, bconn, uuids = attempt(nudge=True)
    check(r.connected(a), "nudge: A gets through the channel")
    check(r.connected(bconn), "nudge: B is still connected")
    check(r.nudged == ["B"] and uuids <= r.droppedUuids, "nudge: B's old track is replaced")
    bnew = copper(r, "B")
    old = sum(math.dist(p, q) for p, q in zip(B_PATH, B_PATH[1:]))
    length = sum(math.dist(*it.pts) for it in bnew if it.kind == "track")
    check(all(it.kind == "track" and it.layers == (0,) for it in bnew), "nudge: B stays on the top layer, no via")
    check(length <= old * 1.3 + 1.0, f"nudge: B keeps near its length ({length:.1f} vs {old:.1f} mm)")
    gap = min(LineString(x.pts).distance(LineString(y.pts)) - x.width / 2 - y.width / 2
              for x in copper(r, "A") if x.kind == "track" for y in bnew)
    check(gap >= CLR - 1e-6, f"nudge: A and B keep the clearance ({gap:.3f} >= {CLR})")
    r, a, bconn, uuids = attempt(nudge=False)
    check(not r.connected(a), "no nudge: A does not get through")
    check(r.connected(bconn) and not r.droppedUuids and not r.nudged, "no nudge: B's track is untouched")
    if FAILS:
        print(f"FAIL: {len(FAILS)} checks")
        sys.exit(1)
    print("PASS: the designer's track is nudged aside, and only with the knob")
