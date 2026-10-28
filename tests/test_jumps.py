"""A signal may pass under a two-pad part, straight between its pads: a person's "jump".

    kicad-python tests/test_jumps.py

Net A has to get from one side of an 0402 resistor to the other on the top layer: keep-
outs above and below the resistor leave only the 0.48 mm gap between its pads. A 0.2 mm
track there keeps 0.14 mm from each pad, and this board's clearance is 0.13 mm: legal,
but not with the search's safety margin on top. The margin is set to 0.05 mm so that,
without jump channels, no grid alignment can open the gap. With them, A crosses exactly
midway and keeps the design clearance from both pads; without them it stays unrouted.
"""
import json
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
CLR = 0.13
PRO = {
    "board": {"design_settings": {"rules": {"min_hole_clearance": 0.25, "min_hole_to_hole": 0.25,
                                             "min_copper_edge_clearance": 0.3}}},
    "net_settings": {"classes": [{"name": "Default", "track_width": 0.2, "clearance": CLR,
                                  "via_diameter": 0.6, "via_drill": 0.3, "diff_pair_gap": 0.2}],
                     "netclass_patterns": []},
}
R_AT = (5.0, 3.0)
PAD = (0.64, 0.54)      # KiCad's R_0402_1005Metric, turned so its pads sit above each other
PITCH = 1.02


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
    nets = {n: pcbnew.NETINFO_ITEM(b, n) for n in ("A", "X", "Y")}
    for n in nets.values():
        b.Add(n)
    corners = [(0, 0), (10, 0), (10, 6), (0, 6)]
    for (ax, ay), (bx, by) in zip(corners, corners[1:] + corners[:1]):
        s = pcbnew.PCB_SHAPE(b)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(v(ax, ay))
        s.SetEnd(v(bx, by))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(int(0.1 * MM))
        b.Add(s)

    def footprint(ref, pads):
        fp = pcbnew.FOOTPRINT(b)
        fp.SetReference(ref)
        b.Add(fp)
        for num, net, p, size in pads:
            pd = pcbnew.PAD(fp)
            pd.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
            pd.SetShape(pcbnew.PAD_SHAPE_RECT)
            pd.SetLayerSet(pd.SMDMask())
            pd.SetSize(v(*size))
            pd.SetPosition(v(*p))
            pd.SetNumber(num)
            pd.SetNet(nets[net])
            fp.Add(pd)
        return fp

    footprint("TP1", [("1", "A", (1.5, 3.0), (1.0, 1.0))])
    footprint("TP2", [("1", "A", (8.5, 3.0), (1.0, 1.0))])
    x, y = R_AT
    footprint("R1", [("1", "X", (x, y - PITCH / 2), PAD), ("2", "Y", (x, y + PITCH / 2), PAD)])
    keepout(b, 4.0, 0.0, 6.0, 2.3)       # the resistor's pads stick into both
    keepout(b, 4.0, 3.7, 6.0, 6.0)
    b.Save(path)
    with open(path.replace(".kicad_pcb", ".kicad_pro"), "w") as f:
        json.dump(PRO, f)
    return pcbnew.LoadBoard(path)


def attempt(jumps):
    tmp = tempfile.mkdtemp()
    path = os.path.join(tmp, "j.kicad_pcb")
    b = build(path)
    tracks, vias, uuids = existingCopper(b)
    model = loadModel(b, path.replace(".kicad_pcb", ".kicad_pro"))
    model.planes = {}
    r = Router(model, Knobs(margin=0.05, jumps=jumps), lambda m: None)
    r.loadExisting(tracks, vias, set())
    conns, _, _ = plan(model, r, lambda m: None, [])
    r.allConns = conns
    a = next(c for c in conns if c.net == "A")
    a.strict, a.prefLayer = True, 0          # A may not leave the top layer
    import run
    run.layersFor = lambda c: [0] if c.strict else [0, 3]
    negotiate(r, [a], lambda m: None, maxRounds=10)
    return r, a, model


FAILS = []


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        FAILS.append(what)


if __name__ == "__main__":
    r, a, model = attempt(jumps=True)
    check(len(r.jumpChannels) == 1 and r.jumpChannels[0]["ref"] == "R1", "jumps: one channel, under R1")
    check(r.connected(a), "jumps: A gets through under R1")
    nid = r.netId["A"]
    tracks = [it for it in r.sp.items.values() if it.net == nid and it.kind == "track"]
    pads = [p for p in model.pads if p.ref == "R1"]
    x, y = R_AT
    under = [t for t in tracks if LineString(t.pts).crosses(LineString([(x, y - PITCH / 2), (x, y + PITCH / 2)]))]
    check(bool(under), "jumps: a track crosses between R1's pads")
    gaps = [LineString(t.pts).distance(p.poly) - t.width / 2 for t in tracks for p in pads]
    check(min(gaps) >= CLR - 1e-6, f"jumps: the design clearance holds from both pads ({min(gaps):.4f} >= {CLR})")
    r, a, model = attempt(jumps=False)
    check(not r.connected(a), "no jumps: the gap stays closed with the safety margin")
    if FAILS:
        print(f"FAIL: {len(FAILS)} checks")
        sys.exit(1)
    print("PASS: a signal jumps under a two-pad part at exact clearance")
