"""A differential pair whose cheapest approach would swap its two nets over.

    kicad-python tests/test_pair_polarity.py

A synthetic board: part U1 has the pair's pads one above the other (DP above DN) and
part U2, south-east of it, has them side by side (DP west of DN). Leaving U1 eastwards,
the short way into U2 comes down from the north, which puts the nets on the wrong
sides: the tracks would have to cross. The ways that keep the polarity are longer (come
up into U2 from the south, or leave U1 westwards). The router must lay the pair together that way (one
centreline, no crossing) instead of giving up and routing the two nets apart: on near-square
pads by retrying from the side that keeps the polarity, on pins by leaving each pin
straight out first.
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
from run import layersFor, viaMmFor, winPadsFor  # noqa: E402
from strategy import plan  # noqa: E402

MM = 1e6
v = lambda x, y: pcbnew.VECTOR2I(int(round(x * MM)), int(round(y * MM)))

PRO = {
    "board": {"design_settings": {"rules": {"min_hole_clearance": 0.25, "min_hole_to_hole": 0.25,
                                             "min_copper_edge_clearance": 0.3}}},
    "net_settings": {
        "classes": [{"name": "Default", "track_width": 0.2, "clearance": 0.15, "via_diameter": 0.6,
                     "via_drill": 0.3, "diff_pair_gap": 0.2}],
        "netclass_patterns": [],
    },
}


def board(path, pin):
    b = pcbnew.BOARD()
    nets = {}
    for n in ("USB_DP", "USB_DN"):
        nets[n] = pcbnew.NETINFO_ITEM(b, n)
        b.Add(nets[n])
    corners = [(0, 0), (30, 0), (30, 30), (0, 30)]
    for (ax, ay), (bx, by) in zip(corners, corners[1:] + corners[:1]):
        s = pcbnew.PCB_SHAPE(b)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(v(ax, ay))
        s.SetEnd(v(bx, by))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(int(0.1 * MM))
        b.Add(s)

    def part(ref, pads):
        fp = pcbnew.FOOTPRINT(b)
        fp.SetReference(ref)
        b.Add(fp)
        for num, net, (x, y), (w, h) in pads:
            p = pcbnew.PAD(fp)
            p.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
            p.SetShape(pcbnew.PAD_SHAPE_RECT)
            p.SetLayerSet(p.SMDMask())
            p.SetSize(v(w, h))
            p.SetPosition(v(x, y))
            p.SetNumber(num)
            p.SetNet(nets[net])
            fp.Add(p)
    # U1: DP above DN, pins pointing east
    part("U1", [("1", "USB_DP", (6.0, 8.0), (pin, 0.3)), ("2", "USB_DN", (6.0, 8.5), (pin, 0.3))])
    # U2: DP west of DN, pins pointing north... and south (both ways are open)
    part("U2", [("1", "USB_DP", (20.0, 18.0), (0.3, pin)), ("2", "USB_DN", (20.5, 18.0), (0.3, pin))])
    b.Save(path)
    with open(path.replace(".kicad_pcb", ".kicad_pro"), "w") as f:
        json.dump(PRO, f)
    return pcbnew.LoadBoard(path)


def attempt(pin, how):
    """Route the pair on pads `pin` mm long; `how`: the log line that must show how it
    was laid. Returns the failures."""
    tmp = tempfile.mkdtemp()
    path = os.path.join(tmp, "pair.kicad_pcb")
    b = board(path, pin)
    model = loadModel(b, path.replace(".kicad_pcb", ".kicad_pro"))
    model.planes = {}
    logs = []
    router = Router(model, Knobs(), logs.append)
    conns, _, _ = plan(model, router, logs.append, [])
    lead = next(c for c in conns if c.pairMate is not None and c.pairLeader is None)
    follow = lead.pairMate
    os.environ["HR_TRACE_PAIR"] = "1"
    ok = router.routePair(lead, follow, layersFor(lead), winPadsFor(lead), viaMmFor(lead, router.k))
    failures = []
    if not ok:
        failures.append("the pair was not laid together")
    else:
        if not any(how[0] in m for m in logs):
            failures.append(how[1])
        a = [router.sp.items[i] for i in router.routes[lead.rid].items if router.sp.items[i].kind == "track"]
        d = [router.sp.items[i] for i in router.routes[follow.rid].items if router.sp.items[i].kind == "track"]
        # the nets do not touch, and the pair stays coupled: every DP piece has a DN piece
        # alongside it, one pair pitch away
        pitch = router.ruleForConn(lead).width + model.classes["Default"].pairGap
        for t in a:
            if any(t.geom.intersects(u.geom) for u in d):
                failures.append(f"DP and DN copper touch at {t.pts}")
        coupled = sum(math.dist(*t.pts) for t in a
                      if min(t.geom.distance(u.geom) for u in d) <= pitch - router.ruleForConn(lead).width + 0.05)
        total = sum(math.dist(*t.pts) for t in a)
        if coupled < 0.6 * total:
            failures.append(f"only {coupled:.1f} of {total:.1f} mm of DP runs beside DN")
    if failures:
        failures.append("router log:\n    " + "\n    ".join(m for m in logs if m.startswith("pair")))
    return failures


def main():
    failures = []
    # near-square pads: nothing to escape along, the side retry has to find the way
    failures += attempt(0.4, ("keeps the polarity", "square pads: laid, but not by the side retry "
                                                    "(the cheapest centreline should not lay)"))
    # pins: each net runs straight out of its pin first, and the pair lays from there
    failures += attempt(0.8, ("escaped the pins", "pins: laid, but not out of the pins"))
    if failures:
        print("FAIL\n  " + "\n  ".join(failures))
        sys.exit(1)
    print("PASS: pair laid together, keeping the polarity, from square pads and out of pins")


if __name__ == "__main__":
    main()
