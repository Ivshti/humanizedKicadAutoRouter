"""Write routes and planes into a pcbnew board, fill zones, run DRC."""
import json
import math
import subprocess
from collections import Counter, defaultdict
from dataclasses import replace

import pcbnew
from shapely.geometry import LineString, Point

from boardModel import KICAD_LAYERS

NM = 1e6


def v2(p):
    return pcbnew.VECTOR2I(int(round(p[0] * NM)), int(round(p[1] * NM)))


def writeItem(board, netInfo, it):
    if it.kind == "track":
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(v2(it.pts[0]))
        t.SetEnd(v2(it.pts[1]))
        t.SetWidth(int(round(it.width * NM)))
        t.SetLayer(KICAD_LAYERS[it.layers[0]])
    elif it.kind == "via":
        t = pcbnew.PCB_VIA(board)
        t.SetPosition(v2(it.pts[0]))
        t.SetViaType(pcbnew.VIATYPE_THROUGH)
        t.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        t.SetWidth(int(round(it.width * NM)))
        t.SetDrill(int(round(it.drill * NM)))
    else:
        return None
    t.SetNet(netInfo)
    board.Add(t)
    return t


def writeRoutes(board, router):
    """Write every route. A track that another route's track ends on the middle of is
    written in pieces split at that joint, so every join is a shared end point: KiCad does
    not always join a track end to the middle of another track (seen on a 45-degree one
    on a 6-layer board, where it reported the joining piece as dangling)."""
    nets = board.GetNetsByName()
    count = Counter()
    tracks = []
    for r in router.routes.values():
        for iid in r.items:
            it = router.sp.items[iid]
            if it.kind == "track":
                tracks.append((r, it))
            elif writeItem(board, nets[r.conn.net], it) is not None:
                count[it.kind] += 1
    ends = defaultdict(list)
    for _, it in tracks:
        ends[(it.net, it.layers[0])] += list(it.pts)
    for r, it in tracks:
        a, b = it.pts
        seg = LineString([a, b]) if a != b else None
        cuts = sorted({p for p in ends[(it.net, it.layers[0])]
                       if seg is not None and math.dist(p, a) > 1e-4 and math.dist(p, b) > 1e-4
                       and seg.distance(Point(p)) < 1e-4}, key=lambda p: math.dist(a, p))
        pts = [a] + cuts + [b]
        for p, q in zip(pts, pts[1:]):
            if writeItem(board, nets[r.conn.net], replace(it, pts=(p, q))) is not None:
                count["track"] += 1
    return count["track"], count["via"]


def addPlanes(board, outline, planes, removeIslands=True):
    """planes: [(layerIndex, netName, thermal)] covering the whole board outline."""
    nets = board.GetNetsByName()
    added = []
    for layer, net, thermal in planes:
        z = pcbnew.ZONE(board)
        added.append(z)
        z.SetLayer(KICAD_LAYERS[layer])
        z.SetNetCode(nets[net].GetNetCode())
        o = z.Outline()
        o.NewOutline()
        for x, y in list(outline.exterior.coords)[:-1]:
            o.Append(int(round(x * NM)), int(round(y * NM)))
        z.SetLocalClearance(int(0.25 * NM))
        z.SetMinThickness(int(0.2 * NM))
        z.SetThermalReliefGap(int(0.25 * NM))
        z.SetThermalReliefSpokeWidth(int(0.4 * NM))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL if thermal else pcbnew.ZONE_CONNECTION_FULL)
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS if removeIslands
                               else pcbnew.ISLAND_REMOVAL_MODE_NEVER)
        board.Add(z)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    return added


def runDrc(boardPath, outJson):
    subprocess.run(["kicad-cli", "pcb", "drc", "--format", "json", "--severity-error",
                    "--severity-warning", "-o", outJson, boardPath], check=True,
                   capture_output=True)
    with open(outJson) as f:
        return json.load(f)
