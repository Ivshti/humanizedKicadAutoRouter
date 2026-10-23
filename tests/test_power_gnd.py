"""No slabs of thick track and no ground tracks unless asked for.

    kicad-python tests/test_power_gnd.py

The Ambire signer's owner (2026-10-23) called both "slop": power tracks the router had
widened to 1-2 mm on big pads, and ground tracks it drew to tie pads the pour missed.

Power: net P, in a Power class 0.5 mm wide, joins two 2 mm pads across an empty board.
Routed, it is 0.5 mm wide. The power copper pass leaves it so by default, and widens it
only with the fatten knob, never past the 2 mm pads.

Ground: under fanoutParts "none", a ground pad the pour misses is listed for the
designer and nothing is drawn (gndRepair "report", the default). With gndRepair "join"
the old repair runs. An unknown gndRepair value is refused before routing.
"""
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import pcbnew  # noqa: E402

import run  # noqa: E402
from boardModel import loadModel  # noqa: E402
from router import Knobs, Router  # noqa: E402
from run import negotiate, powerCopper, repairMissedGnd  # noqa: E402
from strategy import plan  # noqa: E402

MM = 1e6
v = lambda x, y: pcbnew.VECTOR2I(int(round(x * MM)), int(round(y * MM)))
PRO = {
    "board": {"design_settings": {"rules": {"min_hole_clearance": 0.25, "min_hole_to_hole": 0.25,
                                             "min_copper_edge_clearance": 0.3}}},
    "net_settings": {"classes": [{"name": "Default", "track_width": 0.2, "clearance": 0.15,
                                  "via_diameter": 0.6, "via_drill": 0.3, "diff_pair_gap": 0.2},
                                 {"name": "Power", "track_width": 0.5, "clearance": 0.15,
                                  "via_diameter": 0.6, "via_drill": 0.3, "diff_pair_gap": 0.2}],
                     "netclass_patterns": [{"netclass": "Power", "pattern": "P"}]},
}


def build(path):
    b = pcbnew.BOARD()
    net = pcbnew.NETINFO_ITEM(b, "P")
    b.Add(net)
    corners = [(0, 0), (20, 0), (20, 10), (0, 10)]
    for (ax, ay), (bx, by) in zip(corners, corners[1:] + corners[:1]):
        s = pcbnew.PCB_SHAPE(b)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(v(ax, ay))
        s.SetEnd(v(bx, by))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(int(0.1 * MM))
        b.Add(s)
    for ref, x in (("J1", 3.0), ("J2", 17.0)):
        fp = pcbnew.FOOTPRINT(b)
        fp.SetReference(ref)
        b.Add(fp)
        pd = pcbnew.PAD(fp)
        pd.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
        pd.SetShape(pcbnew.PAD_SHAPE_RECT)
        pd.SetLayerSet(pd.SMDMask())
        pd.SetSize(v(2.0, 2.0))
        pd.SetPosition(v(x, 5.0))
        pd.SetNumber("1")
        pd.SetNet(net)
        fp.Add(pd)
    b.Save(path)
    with open(path.replace(".kicad_pcb", ".kicad_pro"), "w") as f:
        json.dump(PRO, f)
    return pcbnew.LoadBoard(path)


def routedPower(fatten):
    path = os.path.join(tempfile.mkdtemp(), "p.kicad_pcb")
    b = build(path)
    model = loadModel(b, path.replace(".kicad_pcb", ".kicad_pro"))
    model.planes = {}
    r = Router(model, Knobs(fatten=fatten), lambda m: None)
    conns, _, _ = plan(model, r, lambda m: None, [])
    r.allConns = conns
    negotiate(r, conns, lambda m: None, maxRounds=5)
    lines = []
    widened = powerCopper(r, model, lines.append)
    nid = r.netId["P"]
    widths = sorted({round(it.width, 3) for it in r.sp.items.values() if it.net == nid and it.kind == "track"})
    return r, conns, widened, widths, lines


class Pad:
    def __init__(self, ref, num):
        self.ref, self.num = ref, num


class Conn:
    def __init__(self, ref, num):
        self.a, self.deferred = Pad(ref, num), True


FAILS = []


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        FAILS.append(what)


if __name__ == "__main__":
    r, conns, widened, widths, lines = routedPower(fatten=False)
    check(all(r.connected(c) for c in conns), "power: P is routed")
    check(widened == 0 and widths == [0.5], f"power: fatten off keeps the class width ({widths})")
    check(any("fatten off" in s for s in lines), "power: the log says the widths were kept")
    r, conns, widened, widths, lines = routedPower(fatten=True)
    check(widened > 0 and max(widths) > 0.5, f"power: fatten on widens ({widths})")
    check(max(widths) <= 2.0 + 1e-6, f"power: never past the 2 mm pads ({max(widths)})")

    missed = [Conn("R1", "2"), Conn("C3", "1")]
    calls = []
    real = run.repairUnreached
    run.repairUnreached = lambda board, router, ms, log: calls.append(ms) or (["fan"], 1)
    try:
        for mode, joins in (("report", False), ("join", True)):
            router = type("R", (), {"k": Knobs(fanoutParts="none", gndRepair=mode)})()
            lines = []
            fan, solid = repairMissedGnd(None, router, missed, lines.append)
            if joins:
                check(calls == [missed] and (fan, solid) == (["fan"], 1), "gnd: join ties the pads it is given")
            else:
                check(not calls and (fan, solid) == ([], 0), "gnd: report adds nothing")
                check(any("C3.1, R1.2" in s for s in lines), "gnd: report lists the pads for the designer")
    finally:
        run.repairUnreached = real
    check(Knobs().gndRepair == "report" and Knobs().fatten is False, "defaults: report, fatten off")
    path = os.path.join(tempfile.mkdtemp(), "k.kicad_pcb")
    build(path)
    out = subprocess.run([sys.executable, os.path.join(os.path.dirname(HERE), "run.py"), path,
                          "--out", tempfile.mkdtemp(), "--knob", "gndRepair=tie"], capture_output=True, text=True)
    check(out.returncode != 0 and "gndRepair must be" in out.stdout + out.stderr,
          "gnd: an unknown gndRepair value is refused")
    if FAILS:
        print(f"FAIL: {len(FAILS)} checks")
        sys.exit(1)
    print("PASS: power keeps its class width and ground gets no tracks unless asked")
