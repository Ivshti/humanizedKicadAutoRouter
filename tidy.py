"""Copper clean-up, the router's own "Cleanup Tracks & Vias": what a person tidies before
calling a board done. Each class of edit is found on the board, applied, and checked with
KiCad's DRC; a class that makes the board worse (more unconnected items or copper
violations) is retried in halves, and only the edits that do no harm are kept.

Classes, in the order they run:
  overlap     a segment lying entirely on another of the same net and layer that is at
              least as wide, or buried in the copper of its net's other tracks and pads
              (the copper does not change). Partial overlaps (two runs that share a
              stretch and part) are left alone.
  inPad       a track lying entirely inside a pad of its net, except the piece that
              brings a route on to the pad's centre.
  collinear   two segments of the same width that meet end to end in a straight line,
              where nothing else connects: merged into one.
  overshoot   a track that runs past a junction and doubles back to it: cut back to the
              junction; a track end that runs on in line along another track past that
              track's end: pulled back to it.
  dangling    a track end that touches nothing: cut back to the nearest point where
              copper of its net joins the track, or removed when nothing does.
  loneVia     a via that touches copper of its net on one layer only.
  padlessGnd  ground tracks that reach no ground pad (a lone ground via is stitching and
              stays; tracks that join two pieces of pour stay too).
  stale       pieces of another net's copper that touch none of its pads and no pour of
              it.
  padStub     a ground track from one pad into the pour, with no via (a repair the pour
              may no longer need): dropped when the board stays as good without it; else,
              unless the pad is one of a two-pad part, the pad is joined to the pour solid
              instead; a two-pad part keeps its track (a solid joint on one end of an 0402
              risks tombstoning).
dangling, loneVia, padlessGnd and stale only touch nets with nothing left unconnected: on
a net still being routed, a stub is the start of the next route. No edit may leave two
pads joined only because copper overlaps (a track end inside another track's width, a
wide track over a neighbouring pin): KiCad counts that as connected, but it breaks when a
person narrows the track.
"""
import math
import os
import re
import shutil
import subprocess
import json
from collections import defaultdict

import pcbnew

from boardModel import fillZones, polySetToShapely
from shapely.geometry import LineString, Point
from shapely.ops import unary_union
from shapely.strtree import STRtree

NM = 1e6
TOL = 2   # nm: two points closer than this are the same point
COVER = 10000   # nm: copper this close to being covered by other copper adds nothing
ON_LINE = 30000  # nm: an end this close to a track's centreline stays inside any track a
                 # person would draw there (0.1 mm and up): it is on the line, not overlapping
CLASSES = ("overlap", "inPad", "collinear", "overshoot", "dangling", "loneVia", "padlessGnd", "stale", "padStub")
COSMETIC = ("silk_", "lib_footprint", "courtyard", "text_", "footprint_type")


# ---- geometry helpers (nm) ------------------------------------------------------------
def xy(v):
    return (v.x, v.y)


def same(p, q):
    return abs(p[0] - q[0]) <= TOL and abs(p[1] - q[1]) <= TOL


def onSegment(p, a, b):
    """p lies on segment a-b (within TOL)."""
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return same(p, a)
    t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2
    if t < -1e-9 or t > 1 + 1e-9:
        return False
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(p[0] - cx, p[1] - cy) <= TOL


def direction(a, b):
    L = math.dist(a, b)
    return ((b[0] - a[0]) / L, (b[1] - a[1]) / L) if L else (0.0, 0.0)


def tracksOf(board):
    return [t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"]


def viasOf(board):
    return [t for t in board.GetTracks() if t.GetClass() == "PCB_VIA"]


def uid(item):
    """pcbnew hands out a new Python wrapper on every call: identity is the uuid."""
    return item.m_Uuid.AsString()


def touchesAt(board, item, p, layer, net, skip=(), pads=True):
    """Whether something of `net` other than `item` (and `skip`) touches point p on
    `layer`: a track end or body, a via, a pad (pads=False: copper other than pads)."""
    other = {uid(item)} | {uid(s) for s in skip}
    for t in board.GetTracks():
        if uid(t) in other or t.GetNetname() != net:
            continue
        if t.GetClass() == "PCB_VIA":
            if t.IsOnLayer(layer) and math.dist(p, xy(t.GetPosition())) <= t.GetWidth(layer) / 2:
                return True
        elif t.GetLayer() == layer:
            a, b = xy(t.GetStart()), xy(t.GetEnd())
            if LineString([a, b]).distance(Point(p)) <= t.GetWidth() / 2:
                return True
    if not pads:
        return False
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetNetname() == net and pad.IsOnLayer(layer) and \
                    pad.HitTest(pcbnew.VECTOR2I(int(p[0]), int(p[1]))):
                return True
    return False


def zoneTouches(board, p, layer, net):
    for z in board.Zones():
        if z.GetIsRuleArea() or z.GetNetname() != net or not z.IsOnLayer(layer):
            continue
        if z.GetFilledPolysList(layer).Contains(pcbnew.VECTOR2I(int(p[0]), int(p[1]))):
            return True
    return False


# ---- finders: each returns a list of edits ---------------------------------------------
# an edit is a list of ops: ("remove", item) or ("set", track, start, end)
def findOverlap(board, nets=None):
    out = []
    groups = defaultdict(list)
    for t in tracksOf(board):
        groups[(t.GetNetname(), t.GetLayer())].append(t)
    gone = set()
    for ts in groups.values():
        for s in ts:
            a, b = xy(s.GetStart()), xy(s.GetEnd())
            for t in ts:
                if uid(t) == uid(s) or uid(t) in gone or t.GetWidth() < s.GetWidth():
                    continue
                c, d = xy(t.GetStart()), xy(t.GetEnd())
                if onSegment(a, c, d) and onSegment(b, c, d):
                    # identical twins: drop one of them only
                    if t.GetWidth() == s.GetWidth() and onSegment(c, a, b) and onSegment(d, a, b) \
                            and t.m_Uuid.AsString() > s.m_Uuid.AsString():
                        continue
                    out.append([("remove", s)])
                    gone.add(uid(s))
                    break
    # then a piece buried in the copper of its net's other tracks (and pads), off their
    # line by a few microns (a grid-snapped end beside a pad-centred one); a piece covered
    # by pads alone is inPad's to judge
    pads = defaultdict(list)
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetNetname():
                pads[pad.GetNetname()].append(pad)
    for (net, layer), ts in groups.items():
        padCentred = [(padShape(pd, layer), xy(pd.GetPosition())) for pd in pads.get(net, ())
                      if pd.IsOnLayer(layer) and padShape(pd, layer) is not None]
        padGeom = [g for g, _ in padCentred]
        padGeom = [g for g in padGeom if g is not None]
        shapes = [trackGeom(t) for t in ts]
        tree = STRtree(shapes)
        for si, s in enumerate(ts):
            if uid(s) in gone:
                continue
            if _entersPad(s, ts, gone, padCentred):
                continue   # a route's way on to a pad's centre is inPad's to judge
            geom = shapes[si]
            box = geom.envelope.buffer(COVER)
            near = [shapes[n] for n in tree.query(box) if n != si and uid(ts[n]) not in gone]
            if not near:
                continue
            onPads = [g for g in padGeom if g.intersects(box)]
            if onPads and unary_union(onPads).buffer(TOL).contains(geom):
                continue
            if unary_union(near + onPads).buffer(COVER).contains(geom) \
                    and stillJoinedWithout(board, s, gone):
                out.append([("remove", s)])
                gone.add(uid(s))
    return out


_padShapes = {}


def padShape(pd, layer):
    """A pad's copper on a layer (shapely, nm), kept for the run: pads do not move."""
    key = (uid(pd), layer, pd.GetPosition().x, pd.GetPosition().y)
    if key not in _padShapes:
        _padShapes[key] = polySetToShapely(pd.GetEffectivePolygon(layer, pcbnew.ERROR_INSIDE), scale=1)
    return _padShapes[key]


def strictGraph(board, net, gone=()):
    """The joins of a net's copper and pads that a person cannot break by narrowing a
    track: a track end on another track's centreline, in a pad or in a via; a via in a
    pad or on a centreline; a pad whose centre a track runs through. Copper that merely
    overlaps (a wide track over a neighbouring pin) does not count, nor does the pour.
    Returns ({key: set of keys}, pad keys); pads are ("pad", i), copper its uuid."""
    gone = set(gone)
    items = [t for t in board.GetTracks() if t.GetNetname() == net and uid(t) not in gone]
    layers = tuple(board.GetEnabledLayers().CuStack())   # every copper layer of the board
    pads = [pd for fp in board.GetFootprints() for pd in fp.Pads() if pd.GetNetname() == net]
    geoms, meta = [], []   # meta: (key, layers, kind)
    for i, pd in enumerate(pads):
        for l in layers:
            g = padShape(pd, l) if pd.IsOnLayer(l) else None
            if g is not None:
                geoms.append(g.buffer(TOL))
                meta.append((("pad", i), (l,), "pad"))
    for t in items:
        if t.GetClass() == "PCB_VIA":
            geoms.append(Point(xy(t.GetPosition())).buffer(t.GetWidth(pcbnew.F_Cu) / 2))
            meta.append((uid(t), tuple(l for l in layers if t.IsOnLayer(l)), "via"))
        else:
            geoms.append(LineString([xy(t.GetStart()), xy(t.GetEnd())]) if t.GetLength()
                         else Point(xy(t.GetStart())))
            meta.append((uid(t), (t.GetLayer(),), "line"))
    tree = STRtree(geoms) if geoms else None
    edges = defaultdict(set)

    def join(a, b):
        edges[a].add(b)
        edges[b].add(a)

    def near(q):
        return tree.query(q.buffer(ON_LINE)) if tree is not None else ()
    for i, pd in enumerate(pads):
        edges[("pad", i)]
        c = Point(xy(pd.GetPosition()))
        for n in near(c):
            key, ls, kind = meta[n]
            if kind == "line" and pd.IsOnLayer(ls[0]) and geoms[n].distance(c) <= ON_LINE:
                join(("pad", i), key)
    for t in items:
        own = uid(t)
        edges[own]
        if t.GetClass() == "PCB_VIA":
            anchors = [(xy(t.GetPosition()), tuple(l for l in layers if t.IsOnLayer(l)))]
        else:
            anchors = [(xy(t.GetStart()), (t.GetLayer(),)), (xy(t.GetEnd()), (t.GetLayer(),))]
        for p, ls in anchors:
            q = Point(p)
            for n in near(q):
                key, kls, kind = meta[n]
                if key == own or not set(kls) & set(ls):
                    continue
                if kind == "line":
                    if geoms[n].distance(q) <= ON_LINE:
                        join(own, key)
                elif geoms[n].contains(q):
                    join(own, key)
    return edges, [("pad", i) for i in range(len(pads))]

def components(edges):
    comp, n = {}, 0
    for k in edges:
        if k in comp:
            continue
        comp[k], todo = n, [k]
        while todo:
            for o in edges[todo.pop()]:
                if o not in comp:
                    comp[o] = n
                    todo.append(o)
        n += 1
    return comp


def stillJoinedWithout(board, s, gone=()):
    """Whether everything `s` joins stays joined without it, by joins that do not depend
    on track width (strictGraph)."""
    edges, _ = strictGraph(board, s.GetNetname(), gone)
    mine = edges.pop(uid(s), set())
    for k in edges:
        edges[k].discard(uid(s))
    comp = components(edges)
    return len({comp[k] for k in mine}) <= 1


def looseNets(board, skip=()):
    """Nets whose pads are joined only partly by joins that do not depend on track width
    (KiCad may still see them connected: the copper overlaps)."""
    out = []
    for net in sorted({str(n) for n in board.GetNetsByName().keys()} - set(skip) - {""}):
        edges, padKeys = strictGraph(board, net)
        if len(padKeys) < 2 or not any(isinstance(k, str) for k in edges):
            continue
        comp = components(edges)
        if len({comp[k] for k in padKeys}) > 1:
            out.append(net)
    return out

def _entersPad(s, ts, gone, padCentred):
    """Whether track s is part of a route's way on to a pad: an end inside the pad, and
    no other track (one not joined end to end with s) already reaches the pad's centre."""
    ends = (xy(s.GetStart()), xy(s.GetEnd()))
    for g, c in padCentred:
        if not any(g.buffer(TOL).contains(Point(p)) for p in ends):
            continue
        reached = False
        for o in ts:
            if uid(o) == uid(s) or uid(o) in gone:
                continue
            oe = (xy(o.GetStart()), xy(o.GetEnd()))
            if any(same(p, q) for p in ends for q in oe):
                continue
            if LineString(oe).distance(Point(c)) <= ON_LINE:
                reached = True
                break
        if not reached:
            return True
    return False


def trackGeom(t):
    a, b = xy(t.GetStart()), xy(t.GetEnd())
    return (LineString([a, b]) if t.GetLength() else Point(a)).buffer(t.GetWidth() / 2)


def findInPad(board, nets=None):
    out = []
    pads = defaultdict(list)
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetNetname():
                pads[pad.GetNetname()].append(pad)
    for t in tracksOf(board):
        layer = t.GetLayer()
        geom = LineString([xy(t.GetStart()), xy(t.GetEnd())]).buffer(t.GetWidth() / 2) \
            if t.GetLength() else Point(xy(t.GetStart())).buffer(t.GetWidth() / 2)
        for pad in pads.get(t.GetNetname(), ()):
            if not pad.IsOnLayer(layer):
                continue
            poly = polySetToShapely(pad.GetEffectivePolygon(layer, pcbnew.ERROR_INSIDE), scale=1)
            if poly is not None and poly.buffer(TOL).contains(geom):
                if not _centringLeg(board, t, pad, poly, layer):
                    out.append([("remove", t)])
                break
    return out


def _centringLeg(board, t, pad, poly, layer):
    """Whether an in-pad track is the last piece of a route that brings it on to the pad's
    centre, as a person draws it: joined to other copper of its net, and either ending at
    the centre or joined to another piece inside the pad (a leg with a bend in the pad)."""
    net = t.GetNetname()
    ends = (xy(t.GetStart()), xy(t.GetEnd()))
    if not any(touchesAt(board, t, p, layer, net, pads=False) for p in ends):
        return False
    c = xy(pad.GetPosition())
    if any(math.dist(p, c) <= 1e3 for p in ends):
        return True
    for o in tracksOf(board):
        if uid(o) == uid(t) or o.GetNetname() != net or o.GetLayer() != layer:
            continue
        oe = (xy(o.GetStart()), xy(o.GetEnd()))
        # end to end with a piece that goes on to the centre (or stays inside the pad)
        if any(same(p, q) for p in ends for q in oe) and \
                (any(math.dist(q, c) <= 1e3 for q in oe)
                 or poly.buffer(TOL).contains(LineString(oe).buffer(o.GetWidth() / 2))):
            return True
    return False


def endsAt(board):
    """{(net, layer, point): [tracks ending there]}."""
    ends = defaultdict(list)
    for t in tracksOf(board):
        for p in (xy(t.GetStart()), xy(t.GetEnd())):
            ends[(t.GetNetname(), t.GetLayer(), (round(p[0] / TOL), round(p[1] / TOL)))].append((t, p))
    return ends


def findCollinear(board, nets=None):
    out, used = [], set()
    for (net, layer, _), hits in endsAt(board).items():
        if len(hits) != 2:
            continue
        (s, p), (t, _) = hits
        if uid(s) == uid(t) or uid(s) in used or uid(t) in used or s.GetWidth() != t.GetWidth():
            continue
        sa = xy(s.GetStart()) if not same(xy(s.GetStart()), p) else xy(s.GetEnd())
        ta = xy(t.GetStart()) if not same(xy(t.GetStart()), p) else xy(t.GetEnd())
        u, v = direction(sa, p), direction(p, ta)
        if abs(u[0] - v[0]) > 1e-6 or abs(u[1] - v[1]) > 1e-6:
            continue
        if touchesAt(board, s, p, layer, net, skip=(t,)) and not \
                _onlyTheseAt(board, p, layer, net, (s, t)):
            continue
        out.append([("set", s, sa, ta), ("remove", t)])
        used.update((uid(s), uid(t)))
    return out


def _onlyTheseAt(board, p, layer, net, items):
    """Nothing but `items` touches p on layer (no via, pad, or third track)."""
    mine = {uid(i) for i in items}
    for it in board.GetTracks():
        if uid(it) in mine or it.GetNetname() != net:
            continue
        if it.GetClass() == "PCB_VIA":
            if math.dist(p, xy(it.GetPosition())) <= it.GetWidth(layer) / 2:
                return False
        elif it.GetLayer() == layer:
            if LineString([xy(it.GetStart()), xy(it.GetEnd())]).distance(Point(p)) <= it.GetWidth() / 2:
                return False
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetNetname() == net and pad.IsOnLayer(layer) and \
                    pad.HitTest(pcbnew.VECTOR2I(int(p[0]), int(p[1]))):
                return False
    return True


def findOvershoot(board, nets=None):
    """Two segments meeting at p in a 0 deg bend (the second doubles back along the
    first): the stretch past the far end of the shorter one is an overshoot."""
    out, used = [], set()
    for (net, layer, _), hits in endsAt(board).items():
        if nets is not None and net not in nets:
            continue
        if len(hits) != 2:
            continue
        (s, p), (t, _) = hits
        if uid(s) == uid(t) or uid(s) in used or uid(t) in used:
            continue
        sa = xy(s.GetStart()) if not same(xy(s.GetStart()), p) else xy(s.GetEnd())
        ta = xy(t.GetStart()) if not same(xy(t.GetStart()), p) else xy(t.GetEnd())
        u, v = direction(p, sa), direction(p, ta)
        if abs(u[0] - v[0]) > 1e-6 or abs(u[1] - v[1]) > 1e-6:
            continue   # not doubling back
        if not _onlyTheseAt(board, p, layer, net, (s, t)):
            continue
        # the longer one is cut back to the shorter one's far end; the shorter goes
        longer, shorter, lfar, sfar = (s, t, sa, ta) if math.dist(p, sa) >= math.dist(p, ta) else (t, s, ta, sa)
        out.append([("set", longer, lfar, sfar), ("remove", shorter)])
        used.update((uid(s), uid(t)))
    # an end that runs on in line along another track past that track's end (a route that
    # landed a little inside a stub it meets head-on): pulled back to the stub's end
    groups = defaultdict(list)
    for t in tracksOf(board):
        if nets is None or t.GetNetname() in nets:
            groups[(t.GetNetname(), t.GetLayer())].append(t)
    for (net, layer), ts in groups.items():
        for s_ in ts:
            if uid(s_) in used:
                continue
            ends = (xy(s_.GetStart()), xy(s_.GetEnd()))
            for k, p in enumerate(ends):
                r = ends[1 - k]
                hit = None
                for t in ts:
                    if uid(t) == uid(s_) or uid(t) in used:
                        continue
                    a, b = xy(t.GetStart()), xy(t.GetEnd())
                    if same(p, a) or same(p, b) or not onSegment(p, a, b):
                        continue
                    u, v = direction(r, p), direction(a, b)
                    if abs(u[0] * v[1] - u[1] * v[0]) > 1e-6:
                        continue          # not in line
                    e = next((x for x in (a, b) if onSegment(x, r, p) and not same(x, p) and not same(x, r)), None)
                    if e is not None and _onlyTheseAt(board, p, layer, net, (s_, t)):
                        hit = (t, e)
                        break
                if hit is not None:
                    out.append([("set", s_, r, hit[1])] if k == 1 else [("set", s_, hit[1], r)])
                    used.update((uid(s_), uid(hit[0])))
                    break
    return out


def findDangling(board, nets=None):
    """Track ends that touch nothing of their net (no pad, via, track or pour): cut back to
    the nearest point where copper of the net joins the track (another track's end on it,
    a via on it, a pad over it), or the whole track goes when nothing joins it before its
    other end; its neighbour is looked at again on the next round."""
    out = []
    for t in tracksOf(board):
        net = t.GetNetname()
        if nets is not None and net not in nets:
            continue
        layer = t.GetLayer()
        ends = (xy(t.GetStart()), xy(t.GetEnd()))
        for k, p in enumerate(ends):
            if touchesAt(board, t, p, layer, net) or zoneTouches(board, p, layer, net):
                continue
            q = ends[1 - k]
            j = _nearestJoin(board, t, p, q, layer, net)
            out.append([("remove", t)] if j is None else [("set", t, j, q)])
            break
    return out


def _nearestJoin(board, t, p, q, layer, net):
    """The point of track p-q nearest p (not q itself) where other copper of the net joins
    it: another track's end on its line, a via on it, a pad over its line."""
    line = LineString([p, q])
    L = line.length
    best = None

    def consider(pt):
        nonlocal best
        d = line.project(Point(pt))
        if d <= TOL or d >= L - TOL:
            return
        if best is None or d < best:
            best = d
    for o in board.GetTracks():
        if uid(o) == uid(t) or o.GetNetname() != net:
            continue
        if o.GetClass() == "PCB_VIA":
            c = xy(o.GetPosition())
            if o.IsOnLayer(layer) and line.distance(Point(c)) <= COVER:
                consider(c)
        elif o.GetLayer() == layer:
            for e in (xy(o.GetStart()), xy(o.GetEnd())):
                if line.distance(Point(e)) <= COVER:
                    consider(e)
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetNetname() != net or not pad.IsOnLayer(layer):
                continue
            poly = polySetToShapely(pad.GetEffectivePolygon(layer, pcbnew.ERROR_INSIDE), scale=1)
            if poly is None or not poly.intersects(line):
                continue
            hit = poly.intersection(line)
            # the part of the track over the pad nearest p: cut where it enters the pad
            consider(min(([c for g in getattr(hit, "geoms", [hit]) for c in g.coords]),
                         key=lambda c: math.dist(c, p)))
    if best is None:
        return None
    pt = line.interpolate(best)
    return (pt.x, pt.y)


def viaLayers(board, via):
    """Copper layers on which something of the via's net touches it."""
    net = via.GetNetname()
    p = xy(via.GetPosition())
    layers = set()
    for layer in board.GetEnabledLayers().CuStack():
        if not via.IsOnLayer(layer):
            continue
        if touchesAt(board, via, p, layer, net) or zoneTouches(board, p, layer, net):
            layers.add(layer)
    return layers


def findLoneVia(board, nets=None):
    return [[("remove", v)] for v in viasOf(board)
            if (nets is None or v.GetNetname() in nets) and len(viaLayers(board, v)) <= 1]


def clusters(board, net):
    """Connected pieces of one net's tracks and vias (pads and pours not counted as joins)."""
    items = [t for t in board.GetTracks() if t.GetNetname() == net]
    parent = {id(i): id(i) for i in items}

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    geoms = {}
    for i in items:
        if i.GetClass() == "PCB_VIA":
            geoms[id(i)] = (None, Point(xy(i.GetPosition())).buffer(i.GetWidth(pcbnew.F_Cu) / 2))
        else:
            g = LineString([xy(i.GetStart()), xy(i.GetEnd())]) if i.GetLength() else Point(xy(i.GetStart()))
            geoms[id(i)] = (i.GetLayer(), g.buffer(i.GetWidth() / 2))
    for x in items:
        for y in items:
            if id(x) >= id(y):
                continue
            lx, gx = geoms[id(x)]
            ly, gy = geoms[id(y)]
            if (lx is None or ly is None or lx == ly) and gx.intersects(gy):
                parent[find(id(x))] = find(id(y))
    out = defaultdict(list)
    for i in items:
        out[find(id(i))].append(i)
    return list(out.values()), geoms


def _touchesPad(board, piece, geoms, net):
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetNetname() != net:
                continue
            for i in piece:
                layer, g = geoms[id(i)]
                for l in ([layer] if layer is not None else list(pad.GetLayerSet().CuStack())):
                    if not pad.IsOnLayer(l):
                        continue
                    poly = polySetToShapely(pad.GetEffectivePolygon(l, pcbnew.ERROR_INSIDE), scale=1)
                    if poly is not None and poly.intersects(g):
                        return True
    return False


def _pourPieces(board, piece, geoms, net):
    """How many separate pieces of the net's outer pour the copper touches."""
    touched = set()
    for z in board.Zones():
        if z.GetIsRuleArea() or z.GetNetname() != net:
            continue
        for l in (pcbnew.F_Cu, pcbnew.B_Cu):
            if not z.IsOnLayer(l):
                continue
            fill = polySetToShapely(z.GetFilledPolysList(l), scale=1)
            for k, g in enumerate(getattr(fill, "geoms", [fill]) if fill is not None else []):
                if any((geoms[id(i)][0] in (None, l)) and g.intersects(geoms[id(i)][1]) for i in piece):
                    touched.add((z.m_Uuid.AsString(), l, k))
    return len(touched)


def findPadlessGnd(board, nets=None, net="GND"):
    if nets is not None and net not in nets:
        return []
    pieces, geoms = clusters(board, net)
    out = []
    for piece in pieces:
        tracks = [i for i in piece if i.GetClass() == "PCB_TRACK"]
        if not tracks or _touchesPad(board, piece, geoms, net) or _pourPieces(board, piece, geoms, net) >= 2:
            continue
        out.append([("remove", t) for t in tracks])
    return out


def findStale(board, nets=None, plane="GND"):
    out = []
    for net in sorted({t.GetNetname() for t in board.GetTracks()}):
        if not net or net == plane or (nets is not None and net not in nets):
            continue
        pieces, geoms = clusters(board, net)
        for piece in pieces:
            if _touchesPad(board, piece, geoms, net) or _pourPieces(board, piece, geoms, net):
                continue
            out.append([("remove", i) for i in piece])
    return out


def findPadStub(board, nets=None, net="GND"):
    """Ground track pieces with no via that touch one pad and the net's outer pour. Each
    edit carries the pad, for the solid fallback."""
    pieces, geoms = clusters(board, net)
    out = []
    for piece in pieces:
        if any(i.GetClass() == "PCB_VIA" for i in piece):
            continue
        pads = []
        for fp in board.GetFootprints():
            for pad in fp.Pads():
                if pad.GetNetname() != net:
                    continue
                for i in piece:
                    layer, g = geoms[id(i)]
                    if pad.IsOnLayer(layer):
                        poly = polySetToShapely(pad.GetEffectivePolygon(layer, pcbnew.ERROR_INSIDE), scale=1)
                        if poly is not None and poly.intersects(g):
                            pads.append(pad)
                            break
        if len(pads) != 1 or not _pourPieces(board, piece, geoms, net):
            continue
        out.append([("remove", i) for i in piece] + [("pad", pads[0])])
    return out


FINDERS = dict(overlap=findOverlap, inPad=findInPad, collinear=findCollinear, overshoot=findOvershoot,
               dangling=findDangling, loneVia=findLoneVia, padlessGnd=findPadlessGnd, stale=findStale,
               padStub=findPadStub)
COMPLETE_ONLY = {"overshoot", "dangling", "loneVia", "padlessGnd", "stale"}


# ---- applying, undoing, judging ------------------------------------------------------------
def apply(board, edit):
    undo = []
    for op in edit:
        if op[0] == "pad":
            continue
        if op[0] == "solid":
            undo.append(("conn", op[1], op[1].GetLocalZoneConnection()))
            op[1].SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
        elif op[0] == "remove":
            board.Remove(op[1])
            undo.append(("add", op[1]))
        else:
            _, t, a, b = op
            undo.append(("set", t, xy(t.GetStart()), xy(t.GetEnd())))
            t.SetStart(pcbnew.VECTOR2I(int(a[0]), int(a[1])))
            t.SetEnd(pcbnew.VECTOR2I(int(b[0]), int(b[1])))
    return undo


def revert(board, undo):
    for op in reversed(undo):
        if op[0] == "add":
            board.Add(op[1])
        elif op[0] == "conn":
            op[1].SetLocalZoneConnection(op[2])
        else:
            _, t, a, b = op
            t.SetStart(pcbnew.VECTOR2I(int(a[0]), int(a[1])))
            t.SetEnd(pcbnew.VECTOR2I(int(b[0]), int(b[1])))


def describe(edit):
    parts = []
    for op in edit:
        it = op[1]
        if op[0] in ("pad", "solid"):
            fp = it.GetParentFootprint()
            parts.append(f"{op[0]} {fp.GetReference() if fp else '?'}.{it.GetNumber()}")
            continue
        p = xy(it.GetPosition()) if it.GetClass() == "PCB_VIA" else xy(it.GetStart())
        what = "via" if it.GetClass() == "PCB_VIA" else f"track {it.GetLayerName()} {it.GetLength() / NM:.3f} mm"
        parts.append(f"{op[0]} {it.GetNetname()} {what} at ({p[0] / NM:.3f}, {p[1] / NM:.3f})")
    return "; ".join(parts)


class Judge:
    """KiCad's DRC on a scratch copy of the board: unconnected items and copper violations."""

    def __init__(self, workDir, name, proPath, druPath):
        os.makedirs(workDir, exist_ok=True)
        self.path = os.path.join(workDir, name + ".kicad_pcb")
        for src, ext in ((proPath, ".kicad_pro"), (druPath, ".kicad_dru")):
            if src and os.path.exists(src):
                shutil.copy(src, os.path.join(workDir, name + ext))
        self.runs = 0

    def __call__(self, board):
        board.Save(self.path)
        out = self.path + ".drc.json"
        subprocess.run(["kicad-cli", "pcb", "drc", "--format", "json", "--severity-error",
                        "--severity-warning", "--refill-zones", "-o", out, self.path],
                       check=True, capture_output=True)
        self.runs += 1
        with open(out) as f:
            d = json.load(f)
        copper = sum(1 for v in d["violations"] if not v["type"].startswith(COSMETIC))
        apart = {m.group(1) for u in d["unconnected_items"] for it in u["items"]
                 for m in [re.search(r"\[([^\]]+)\]", it["description"])] if m}
        return (len(d["unconnected_items"]), copper), apart


def padPieces(board, net):
    """How many pieces a net's pads fall into by joins that do not depend on track width."""
    edges, pads = strictGraph(board, net)
    comp = components(edges)
    return len({comp[k] for k in pads})


def keepJoined(board, edits, log, label):
    """The edits that leave no net's pads more loosely joined (strictGraph): KiCad's DRC
    counts overlapping copper as joined, so a join that only a track's width makes would
    pass it and break when a person narrows the track. Leaves the board as it was."""
    kept, undo, cache = [], [], {}
    for e in edits:
        nets = {op[1].GetNetname() for op in e if op[0] in ("remove", "set")}
        before = {n: cache[n] if n in cache else padPieces(board, n) for n in nets}
        u = apply(board, e)
        after = {n: padPieces(board, n) for n in nets}
        if any(after[n] > before[n] for n in nets):
            revert(board, u)
            log(f"tidy {label}: kept back (a join would hang on track width): {describe(e)}")
            continue
        cache.update(after)
        kept.append(e)
        undo += u
    revert(board, undo)
    return kept


def keepSafe(board, edits, judge, base, log, label):
    """Apply edits, keeping those that leave the DRC no worse than base: all at once,
    else in halves. Returns (kept edits, new base)."""
    if not edits:
        return [], base
    undo = []
    for e in edits:
        undo += apply(board, e)
    score, _ = judge(board)
    if score[0] <= base[0] and score[1] <= base[1]:
        return edits, score
    revert(board, undo)
    if len(edits) == 1:
        log(f"tidy {label}: kept back (DRC worse: {base} -> {score}): {describe(edits[0])}")
        return [], base
    mid = len(edits) // 2
    keptA, base = keepSafe(board, edits[:mid], judge, base, log, label)
    keptB, base = keepSafe(board, edits[mid:], judge, base, log, label)
    return keptA + keptB, base


def inScope(edit, nets, fixed):
    """Whether an edit touches only copper it may: of `nets` (None: any net) and not among
    the `fixed` uuids (the designer's copper)."""
    for op in edit:
        it = op[1]
        if op[0] in ("pad", "solid"):
            if nets is not None and it.GetNetname() not in nets:
                return False
            continue
        if (nets is not None and it.GetNetname() not in nets) or it.m_Uuid.AsString() in fixed:
            return False
    return True


def tidy(board, judge, log, report=False, classes=CLASSES, plane="GND", nets=None, fixed=frozenset()):
    """Run the clean-up classes in order. report: list what each class would do and
    change nothing. nets: only copper of these nets is edited (None: every net); fixed:
    uuids of copper that is never edited (the designer's, when routing around it).
    Returns {class: edits kept (or found)}."""
    base, apart = judge(board)
    allNets = {t.GetNetname() for t in board.GetTracks()}
    complete = allNets - apart
    scope = None if nets is None else set(nets)
    done = {}

    def find(cls, which):
        return [e for e in FINDERS[cls](board, which) if inScope(e, scope, fixed)]
    for cls in classes:
        which = complete if cls in COMPLETE_ONLY else None
        total = []
        for _ in range(20):   # dangling stubs peel off one piece per round
            edits = find(cls, which)
            if report:
                total = edits
                break
            if not edits:
                break
            kept, base = keepSafe(board, keepJoined(board, edits, log, cls), judge, base, log, cls)
            if cls == "padStub":
                # kept back: the pour needs the track; a pad of a part with more than two
                # pads may take a solid joint instead
                for e in edits:
                    if e in kept:
                        continue
                    pad = next(op[1] for op in e if op[0] == "pad")
                    fp = pad.GetParentFootprint()
                    if fp is None or len(list(fp.Pads())) <= 2:
                        continue
                    more, base = keepSafe(board, [[op for op in e if op[0] == "remove"] + [("solid", pad)]],
                                          judge, base, log, cls)
                    kept += more
            total += kept
            if not kept or cls not in ("dangling", "collinear", "overshoot"):
                break
        if cls == "loneVia" and total and "dangling" in done and not report:
            # a removed via leaves the track that ran to it hanging, and a removed stub
            # leaves its via alone: the two take turns until neither finds anything
            for _ in range(5):
                more = 0
                for again in ("dangling", "loneVia"):
                    edits = find(again, complete)
                    kept, base = keepSafe(board, keepJoined(board, edits, log, again), judge, base, log, again)
                    (done["dangling"] if again == "dangling" else total).extend(kept)
                    more += len(kept)
                if not more:
                    break
        done[cls] = total
        verb = "would change" if report else "changed"
        log(f"tidy {cls}: {verb} {len(total)}")
        if report:
            for e in total:
                log(f"  {describe(e)}")
    if not report:
        fillZones(board)
    log(f"tidy: DRC {base[1]} copper violations, {base[0]} unconnected ({judge.runs} DRC runs)")
    return done
