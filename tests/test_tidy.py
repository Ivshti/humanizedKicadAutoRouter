"""The copper clean-up classes, each on a case it must change and one it must leave.

    kicad-python tests/test_tidy.py

Most checks use a judge that accepts everything, so they test what each class finds and
how it edits; the last one runs KiCad's DRC as the judge on the same board.
"""
import json
import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import pcbnew  # noqa: E402

import tidy as T  # noqa: E402

MM = 1e6
v = lambda x, y: pcbnew.VECTOR2I(int(round(x * MM)), int(round(y * MM)))
PRO = {
    "board": {"design_settings": {"rules": {"min_hole_clearance": 0.25, "min_hole_to_hole": 0.25,
                                             "min_copper_edge_clearance": 0.3}}},
    "net_settings": {"classes": [{"name": "Default", "track_width": 0.2, "clearance": 0.15,
                                  "via_diameter": 0.6, "via_drill": 0.3, "diff_pair_gap": 0.2}],
                     "netclass_patterns": []},
}


class Board:
    def __init__(self):
        self.b = pcbnew.BOARD()
        self.nets = {}
        corners = [(0, 0), (80, 0), (80, 80), (0, 80)]
        for (ax, ay), (bx, by) in zip(corners, corners[1:] + corners[:1]):
            s = pcbnew.PCB_SHAPE(self.b)
            s.SetShape(pcbnew.SHAPE_T_SEGMENT)
            s.SetStart(v(ax, ay))
            s.SetEnd(v(bx, by))
            s.SetLayer(pcbnew.Edge_Cuts)
            s.SetWidth(int(0.1 * MM))
            self.b.Add(s)

    def net(self, name):
        if name not in self.nets:
            self.nets[name] = pcbnew.NETINFO_ITEM(self.b, name)
            self.b.Add(self.nets[name])
        return self.nets[name]

    def track(self, net, a, b, w=0.2, layer=pcbnew.F_Cu):
        t = pcbnew.PCB_TRACK(self.b)
        t.SetStart(v(*a))
        t.SetEnd(v(*b))
        t.SetWidth(int(w * MM))
        t.SetLayer(layer)
        t.SetNet(self.net(net))
        self.b.Add(t)
        return t

    def via(self, net, p):
        t = pcbnew.PCB_VIA(self.b)
        t.SetPosition(v(*p))
        t.SetViaType(pcbnew.VIATYPE_THROUGH)
        t.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        t.SetWidth(int(0.6 * MM))
        t.SetDrill(int(0.3 * MM))
        t.SetNet(self.net(net))
        self.b.Add(t)
        return t

    def pad(self, ref, net, p, size=(1.0, 1.0)):
        fp = pcbnew.FOOTPRINT(self.b)
        fp.SetReference(ref)
        self.b.Add(fp)
        pd = pcbnew.PAD(fp)
        pd.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
        pd.SetShape(pcbnew.PAD_SHAPE_RECT)
        pd.SetLayerSet(pd.SMDMask())
        pd.SetSize(v(*size))
        pd.SetPosition(v(*p))
        pd.SetNumber("1")
        pd.SetNet(self.net(net))
        fp.Add(pd)
        return pd


def alive(board, item):
    return any(t is item or t.m_Uuid.AsString() == item.m_Uuid.AsString() for t in board.GetTracks())


def ends(t):
    return (round(t.GetStart().x / MM, 3), round(t.GetStart().y / MM, 3)), \
           (round(t.GetEnd().x / MM, 3), round(t.GetEnd().y / MM, 3))


class Yes:
    """A judge that accepts everything (nets all complete)."""
    runs = 0

    def __call__(self, board):
        return (0, 0), set()


def run(B, classes):
    logs = []
    T.tidy(B.b, Yes(), logs.append, classes=classes)
    return logs


FAILS = []


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        FAILS.append(what)


def testOverlap():
    B = Board()
    B.pad("P1", "A", (5, 5)); B.pad("P2", "A", (15, 5))
    base = B.track("A", (5, 5), (15, 5))
    covered = B.track("A", (7, 5), (10, 5))                # lies on base, same width: goes
    wider = B.track("A", (11, 5), (12, 5), w=0.3)          # lies on base but wider: stays
    partial = B.track("A", (13, 5), (18, 5))               # shares a stretch, then leaves: stays
    run(B, ["overlap"])
    check(not alive(B.b, covered), "overlap: a segment on a wider-or-equal one is removed")
    check(alive(B.b, wider) and alive(B.b, base), "overlap: the wider segment and its host stay")
    check(alive(B.b, partial), "overlap: a partial overlap is left alone")
    B = Board()
    B.pad("P1", "A", (5, 5), size=(0.6, 0.6)); B.pad("P2", "A", (5, 12))
    through = B.track("A", (5, 2), (5, 12), w=0.5)              # runs through P1's centre
    buried = B.track("A", (5.005, 5.005), (5.005, 5.3), w=0.5)  # 5 um off it, half out of the pad
    run(B, ["overlap"])
    check(not alive(B.b, buried) and alive(B.b, through), "overlap: a piece buried in its net's copper is removed")
    B = Board()
    B.pad("P1", "A", (5, 5), size=(0.6, 1.07)); B.pad("P2", "A", (8, 9))
    leg = B.track("A", (5, 5), (5, 5.45), w=0.3)                # from the centre, a little out of the pad
    on = B.track("A", (5, 5.45), (8, 9), w=0.3)                 # whose round start covers the rest
    run(B, ["overlap"])
    check(alive(B.b, leg), "overlap: a route's start on the pad centre stays")
    B = Board()
    B.pad("P1", "A", (5, 2)); B.pad("P2", "A", (5, 12)); B.pad("P3", "A", (9, 6))
    wide = B.track("A", (5, 2), (5, 12), w=0.8)
    stub = B.track("A", (5.0, 6.0), (5.3, 6.0), w=0.2)          # inside the wide track, from its line...
    branch = B.track("A", (5.3, 6.0), (9, 6), w=0.2)            # ...to a branch that ends off it
    run(B, ["overlap"])
    check(alive(B.b, stub), "overlap: a piece that carries a join off the centreline stays")
    B = Board()
    B.pad("U1", "A", (8.0, 41.85), size=(0.88, 0.25)); B.pad("U2", "A", (8.0, 42.35), size=(0.88, 0.25))
    B.pad("U3", "A", (8.0, 41.35), size=(0.88, 0.25)); B.pad("P9", "A", (12, 42.35))
    link = B.track("A", (8.1, 42.35), (8.1, 41.85), w=0.3)        # pin to pin, ends off the pad centres
    B.track("A", (8.1, 41.85), (8.1, 41.35), w=0.3)
    B.track("A", (8.0, 42.35), (12, 42.35), w=0.8)                # wide, its copper over the next pin
    run(B, ["overlap"])
    check(alive(B.b, link), "overlap: the only centreline link between two pins stays under a wide track")


def testInPad():
    B = Board()
    B.pad("P1", "A", (20, 10), size=(2.0, 1.0))
    inside = B.track("A", (19.6, 10), (20.4, 10))
    leaving = B.track("A", (20.4, 10), (23, 10))
    run(B, ["inPad"])
    check(not alive(B.b, inside), "inPad: a track inside its pad is removed")
    check(alive(B.b, leaving), "inPad: a track leaving the pad stays")
    B = Board()
    B.pad("P1", "A", (20, 10), size=(2.0, 1.0))
    centring = B.track("A", (20, 10), (20.8, 10))              # edge-to-centre leg of a route
    route = B.track("A", (20.8, 10), (23, 10))
    bendIn = B.track("A", (19.3, 10), (19.5, 10.2))            # a leg with a bend inside the pad...
    bendOn = B.track("A", (19.0, 10.2), (19.3, 10))            # ...joined to the route here
    away = B.track("A", (19.0, 10.2), (16, 10.2))
    toCentre = B.track("A", (19.5, 10.2), (20, 10))
    lone = B.track("A", (20.2, 10.3), (20.6, 10.3))            # touches nothing but the pad
    run(B, ["inPad"])
    check(alive(B.b, centring), "inPad: the leg that brings a route to the pad centre stays")
    check(alive(B.b, bendIn) and alive(B.b, toCentre), "inPad: a leg with a bend in the pad stays")
    check(not alive(B.b, lone), "inPad: a lone stub inside the pad is removed")
    B = Board()
    B.pad("P1", "A", (49.52, 5.6), size=(0.56, 0.62)); B.pad("P2", "A", (49.75, 1.0))
    B.track("A", (49.75, 1.0), (49.75, 5.325), w=0.2)
    link = B.track("A", (49.75, 5.325), (49.75, 5.37), w=0.2)   # a short link inside the pad...
    leg = B.track("A", (49.75, 5.37), (49.52, 5.6), w=0.2)      # ...to the leg on to its centre
    run(B, ["overlap", "inPad"])
    check(alive(B.b, link) and alive(B.b, leg), "inPad: a link to the centre leg stays, though the leg leaves the pad")
    B = Board()
    B.pad("L2", "A", (16.75, 3.0), size=(1.4, 2.7)); B.pad("P2", "A", (19, 6))
    B.track("A", (19, 6), (17.3, 4.3))
    first = B.track("A", (17.3, 4.3), (16.75, 3.75))            # into the pad...
    second = B.track("A", (16.75, 3.75), (16.75, 3.0))          # ...and on to its centre
    run(B, ["overlap", "inPad"])
    check(alive(B.b, first) and alive(B.b, second), "overlap/inPad: a two-piece way on to a large pad's centre stays")


def testCollinear():
    B = Board()
    B.pad("P1", "A", (5, 20)); B.pad("P2", "A", (15, 20))
    s1 = B.track("A", (5, 20), (10, 20))
    s2 = B.track("A", (10, 20), (15, 20))
    B.pad("P3", "B", (5, 25)); B.pad("P4", "B", (15, 25))
    t1 = B.track("B", (5, 25), (10, 25))
    t2 = B.track("B", (10, 25), (15, 25))
    B.via("B", (10, 25))                                   # something joins there: no merge
    run(B, ["collinear"])
    merged = [t for t in B.b.GetTracks() if t.GetNetname() == "A"]
    check(len(merged) == 1 and set(ends(merged[0])) == {(5, 20), (15, 20)}, "collinear: a split is merged")
    check(alive(B.b, t1) and alive(B.b, t2), "collinear: a split at a via stays")


def testOvershoot():
    B = Board()
    B.pad("P1", "A", (5, 30)); B.pad("P2", "A", (12, 36))
    a = B.track("A", (5, 30), (14, 30))
    back = B.track("A", (14, 30), (12, 30))                # doubles back to where the route turns
    B.track("A", (12, 30), (12, 36))
    run(B, ["overshoot"])
    netA = [ends(t) for t in B.b.GetTracks() if t.GetNetname() == "A"]
    check(not alive(B.b, back) and ((5, 30), (12, 30)) in netA, "overshoot: cut back to the junction")
    B = Board()
    B.pad("P1", "A", (13, 18.2), size=(0.4, 0.4)); B.pad("P2", "A", (13, 14), size=(0.4, 0.4))
    stub = B.track("A", (13, 18.2), (13, 16.9))
    route = B.track("A", (13, 14), (13, 17.025))            # meets the stub head-on and runs into it
    run(B, ["overshoot"])
    check(alive(B.b, stub) and ends(route) in (((13, 14), (13, 16.9)), ((13, 16.9), (13, 14))),
          "overshoot: an end run on along a stub is pulled back to the stub's end")


def testDangling():
    B = Board()
    B.pad("P1", "A", (5, 40)); B.pad("P2", "A", (15, 40))
    B.track("A", (5, 40), (15, 40))
    stub1 = B.track("A", (10, 40), (10, 43))               # leaves the route ...
    stub2 = B.track("A", (10, 43), (12, 45))               # ... and ends in nothing
    run(B, ["dangling"])
    check(not alive(B.b, stub1) and not alive(B.b, stub2), "dangling: a stub goes back to the junction")
    check(len([t for t in B.b.GetTracks() if t.GetNetname() == "A"]) == 1, "dangling: the route itself stays")
    B = Board()
    B.pad("P1", "A", (10.6, 19.0), size=(0.4, 0.4)); B.pad("P2", "A", (14, 17.5), size=(0.4, 0.4))
    lane = B.track("A", (10.6, 19.0), (10.6, 16.9))        # an escape lane past where the route joins it
    B.track("A", (14, 17.5), (10.6, 17.5))                  # the route, T-joined to the lane's middle
    run(B, ["dangling"])
    check(alive(B.b, lane) and ends(lane) in (((10.6, 19.0), (10.6, 17.5)), ((10.6, 17.5), (10.6, 19.0))),
          "dangling: a lane is cut back to where the route joins it, not removed")


def testLoneVia():
    B = Board()
    lone = B.via("A", (30, 10))
    B.pad("P1", "B", (35, 10)); B.pad("P2", "B", (45, 10))
    B.track("B", (35, 10), (40, 10))
    used = B.via("B", (40, 10))
    B.track("B", (40, 10), (45, 10), layer=pcbnew.B_Cu)
    run(B, ["loneVia"])
    check(not alive(B.b, lone), "loneVia: a via touching nothing goes")
    check(alive(B.b, used), "loneVia: a via joining two layers stays")
    B = Board()
    B.pad("P1", "C", (30, 20)); B.pad("P2", "C", (30, 25))
    B.track("C", (30, 20), (30, 25))
    tail = B.track("C", (30, 22), (33, 22))                  # a branch to a via that goes nowhere
    end = B.via("C", (33, 22))
    run(B, ["dangling", "loneVia"])
    check(not alive(B.b, end) and not alive(B.b, tail), "loneVia: the track left hanging by a lone via goes too")


def testPadlessGnd():
    B = Board()
    B.via("GND", (30, 30)); B.via("GND", (33, 30))
    crumb = B.track("GND", (30, 30), (33, 30))
    stitch = B.via("GND", (40, 30))
    B.pad("P1", "GND", (45, 30))
    feeding = B.track("GND", (45, 30), (48, 30))
    B.via("GND", (48, 30))
    run(B, ["padlessGnd"])
    check(not alive(B.b, crumb), "padlessGnd: a ground track reaching no pad goes")
    check(alive(B.b, stitch), "padlessGnd: a lone ground via (stitching) stays")
    check(alive(B.b, feeding), "padlessGnd: a ground track from a pad stays")


def testStale():
    B = Board()
    B.pad("P1", "SIG", (30, 50))
    dead = B.track("SIG", (35, 50), (38, 50))
    live = B.track("SIG", (30, 50), (33, 50))
    run(B, ["stale"])
    check(not alive(B.b, dead), "stale: copper touching no pad of its net goes")
    check(alive(B.b, live), "stale: copper from a pad stays")


def gndPour(B):
    z = pcbnew.ZONE(B.b)
    z.SetLayer(pcbnew.F_Cu)
    z.SetNetCode(B.net("GND").GetNetCode())
    o = z.Outline()
    o.NewOutline()
    for x, y in ((50, 50), (78, 50), (78, 78), (50, 78)):
        o.Append(int(x * MM), int(y * MM))
    B.b.Add(z)


def part(B, ref, pads):
    fp = pcbnew.FOOTPRINT(B.b)
    fp.SetReference(ref)
    B.b.Add(fp)
    for num, net, p in pads:
        pd = pcbnew.PAD(fp)
        pd.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
        pd.SetShape(pcbnew.PAD_SHAPE_RECT)
        pd.SetLayerSet(pd.SMDMask())
        pd.SetSize(v(0.6, 0.6))
        pd.SetPosition(v(*p))
        pd.SetNumber(num)
        pd.SetNet(B.net(net))
        fp.Add(pd)


def padStubBoard():
    """A chip and a capacitor, each with a ground stub into a ground pour; saved and
    loaded back (KiCad's zone filler needs a board that came from a file)."""
    B = Board()
    gndPour(B)
    part(B, "U1", [("1", "GND", (60, 60)), ("2", "X", (61, 60)), ("3", "Y", (62, 60))])
    part(B, "C1", [("1", "GND", (60, 70)), ("2", "Z", (61, 70))])
    s1 = B.track("GND", (60, 60), (60, 63)).m_Uuid.AsString()
    s2 = B.track("GND", (60, 70), (60, 73)).m_Uuid.AsString()
    path = os.path.join(tempfile.mkdtemp(), "stub.kicad_pcb")
    B.b.Save(path)
    b = pcbnew.LoadBoard(path)
    T.fillZones(b)
    track = {t.m_Uuid.AsString(): t for t in b.GetTracks()}
    pad = lambda ref: b.FindFootprintByReference(ref).FindPadByNumber("1")
    return b, track[s1], track[s2], pad("U1"), pad("C1")


class NeedsJoint:
    """A judge for which a pad-to-pour stub may only go if its pad is solid instead."""
    runs = 0

    def __init__(self, pads):
        self.pads = pads

    def __call__(self, board):
        n = 0
        for pad, stub in self.pads:
            if not alive(board, stub) and pad.GetLocalZoneConnection() != pcbnew.ZONE_CONNECTION_FULL:
                n += 1
        return (n, 0), set()


def testPadStub():
    b, s1, s2, chip, cap = padStubBoard()
    logs = []
    T.tidy(b, Yes(), logs.append, classes=["padStub"])
    check(not alive(b, s1) and not alive(b, s2), "padStub: stubs the board does not need go")
    b, s1, s2, chip, cap = padStubBoard()
    T.tidy(b, NeedsJoint([(chip, s1), (cap, s2)]), logs.append, classes=["padStub"])
    check(not alive(b, s1) and chip.GetLocalZoneConnection() == pcbnew.ZONE_CONNECTION_FULL,
          "padStub: a chip pad that needs it is joined solid instead")
    check(alive(b, s2) and cap.GetLocalZoneConnection() != pcbnew.ZONE_CONNECTION_FULL,
          "padStub: a two-pad part keeps its track")


def testWithDrc():
    """The same clean-up with KiCad's DRC as the judge: nothing it accepts may make the
    board worse."""
    B = Board()
    B.pad("P1", "A", (5, 5)); B.pad("P2", "A", (15, 5))
    B.track("A", (5, 5), (15, 5))
    covered = B.track("A", (7, 5), (10, 5))
    lone = B.via("Z", (30, 30))
    tmp = tempfile.mkdtemp()
    with open(os.path.join(tmp, "t.kicad_pro"), "w") as f:
        json.dump(PRO, f)
    judge = T.Judge(os.path.join(tmp, "work"), "t", os.path.join(tmp, "t.kicad_pro"), None)
    before, _ = judge(B.b)
    logs = []
    T.tidy(B.b, judge, logs.append)
    after, _ = judge(B.b)
    check(not alive(B.b, covered) and not alive(B.b, lone), "with DRC: overlap and lone via removed")
    check(after[0] <= before[0] and after[1] <= before[1], f"with DRC: no worse ({before} -> {after})")


if __name__ == "__main__":
    for f in (testOverlap, testInPad, testCollinear, testOvershoot, testDangling, testLoneVia,
              testPadlessGnd, testStale, testPadStub, testWithDrc):
        f()
    if FAILS:
        print(f"FAIL: {len(FAILS)} checks")
        sys.exit(1)
    print("PASS: all tidy checks")
