"""Where a planned current really runs on the finished board, and whether it squeezes.

KiCad's DRC cannot see a power path that runs through a thin tap or a neck: every track is
legal on its own. This follows each flow (boards/*.flows.json) through its net's copper,
from the source part's pads to each sink part's pads, along the widest path there is, and
reports any stretch of it narrower than the trunk was meant to be. A short neck onto a pad
narrower than the trunk (a fine-pitch pin) is how a person lands a wide track, and is not
reported.
"""
import heapq
from collections import defaultdict

import pcbnew
from shapely.geometry import LineString, Point, Polygon
from shapely.strtree import STRtree

MM = 1e6
NECK_SLACK_MM = 0.3   # a neck onto a pad may run this much past neckMm (grid, bends)
SHORT_OF = 0.85       # copper this much of the trunk's width or more counts as the trunk


def netCopper(board, net):
    """Tracks, vias and pads of one net as shapes: (kind, layers or None for all, shape,
    width (pads and vias: unlimited), length, extra)."""
    out = []
    for t in board.GetTracks():
        if t.GetNetname() != net:
            continue
        if t.GetClass() == "PCB_VIA":
            p = t.GetPosition()
            out.append(("via", None, Point(p.x / MM, p.y / MM).buffer(t.GetWidth(pcbnew.F_Cu) / MM / 2),
                        float("inf"), 0.0, None))
            continue
        s, e = t.GetStart(), t.GetEnd()
        line = LineString([(s.x / MM, s.y / MM), (e.x / MM, e.y / MM)])
        w = t.GetWidth() / MM
        out.append(("track", {t.GetLayer()}, line.buffer(max(w / 2 - 0.005, 0.001)), w, line.length, line))
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetname() != net:
                continue
            layers = None if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH else \
                {l for l in (pcbnew.F_Cu, pcbnew.B_Cu) if p.IsOnLayer(l)}
            layer = pcbnew.F_Cu if p.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
            poly = p.GetEffectivePolygon(layer)
            pts = [(poly.CVertex(i).x / MM, poly.CVertex(i).y / MM) for i in range(poly.FullPointCount())]
            shape = Polygon(pts).buffer(0) if len(pts) >= 3 else \
                Point(p.GetPosition().x / MM, p.GetPosition().y / MM).buffer(0.1)
            minx, miny, maxx, maxy = shape.bounds
            out.append(("pad", layers, shape, float("inf"), 0.0,
                        (fp.GetReference(), max(maxx - minx, maxy - miny))))
    return out


def touching(items):
    tree = STRtree([it[2] for it in items])
    adj = defaultdict(set)
    for a, it in enumerate(items):
        for b in tree.query(it[2]):
            b = int(b)
            if b == a:
                continue
            la, lb = it[1], items[b][1]
            if la is not None and lb is not None and not la & lb:
                continue
            if it[2].intersects(items[b][2]):
                adj[a].add(b)
    return adj


def widestPath(items, adj, srcs, dsts):
    """The path whose narrowest piece is widest (ties: shortest)."""
    best = {s: (float("inf"), 0.0) for s in srcs}
    prev = {s: None for s in srcs}
    heap = [(-float("inf"), 0.0, s) for s in srcs]
    heapq.heapify(heap)
    done = set()
    while heap:
        nw, L, u = heapq.heappop(heap)
        if u in done:
            continue
        done.add(u)
        if u in dsts:
            path = [u]
            while prev[path[-1]] is not None:
                path.append(prev[path[-1]])
            return path[::-1]
        for v in adj[u]:
            cand = (min(-nw, items[v][3]), L + items[v][4])
            if v not in best or (cand[0], -cand[1]) > (best[v][0], -best[v][1]):
                best[v] = cand
                prev[v] = u
                heapq.heappush(heap, (-cand[0], cand[1], v))
    return None


def squeezes(board, flows, want, neckMm, trunkMinAmps=0.3):
    """Per flow carrying at least trunkMinAmps to each sink: (flow, sink, width wanted,
    [(width, mm, layer name, (x, y))] stretches of its current path narrower than that,
    necks onto narrower pads excepted). want(net, amps) is the trunk width the router meant.
    A flow with no copper path between its parts is reported with the stretch list None."""
    found = []
    cache = {}
    for f in flows:
        share = f["amps"] / max(len(f["sinks"]), 1)
        if share < trunkMinAmps:
            continue
        net = f["net"]
        if net not in cache:
            items = netCopper(board, net)
            cache[net] = (items, touching(items))
        items, adj = cache[net]
        w = want(net, share)
        srcs = {i for i, it in enumerate(items) if it[0] == "pad" and it[5][0] == f["source"]}
        for sink in f["sinks"]:
            dsts = {i for i, it in enumerate(items) if it[0] == "pad" and it[5][0] == sink}
            path = widestPath(items, adj, srcs, dsts) if srcs and dsts else None
            if path is None:
                found.append((f, sink, w, None))
                continue
            thin = []
            for k, i in enumerate(path):
                kind, layers, shape, width, length, line = items[i]
                if kind != "track" or width >= SHORT_OF * w - 1e-3:
                    continue
                pads = [items[j] for j in (path[k - 1] if k else None, path[k + 1] if k + 1 < len(path) else None)
                        if j is not None and items[j][0] == "pad"]
                if any(size < w and length <= size / 2 + neckMm + NECK_SLACK_MM for *_, (_, size) in pads):
                    continue
                c = line.interpolate(0.5, normalized=True)
                thin.append((round(width, 3), round(length, 2), board.GetLayerName(next(iter(layers))),
                             (round(c.x, 2), round(c.y, 2))))
            if thin:
                found.append((f, sink, w, thin))
    return found
