"""Write routes and planes into a pcbnew board, fill zones, run DRC."""
import json
import subprocess
from collections import Counter

import pcbnew

from boardModel import KICAD_LAYERS, polySetToShapely

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
    nets = board.GetNetsByName()
    count = Counter()
    for r in router.routes.values():
        for iid in r.items:
            it = router.sp.items[iid]
            if writeItem(board, nets[r.conn.net], it) is not None:
                count[it.kind] += 1
    return count["track"], count["via"]


def floatingIslands(board, layer, net, anchors):
    """Filled pieces of the net's pour on `layer` that touch none of `anchors` (shapely
    shapes of the net's own pads, tracks and vias on that layer)."""
    lid = KICAD_LAYERS[layer]
    out = []
    for z in board.Zones():
        if z.GetNetname() != net or not z.IsOnLayer(lid) or z.GetIsRuleArea():
            continue
        fill = polySetToShapely(z.GetFilledPolysList(lid))
        if fill is None:
            continue
        for piece in getattr(fill, "geoms", [fill]):
            if not any(piece.intersects(a) for a in anchors):
                out.append(piece)
    return out


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
