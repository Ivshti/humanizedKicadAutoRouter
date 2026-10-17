"""Write routes and planes into a pcbnew board, fill zones, run DRC."""
import json
import subprocess
from collections import Counter

import pcbnew

from boardModel import KICAD_LAYERS, fillZones

NM = 1e6


def v2(p):
    return pcbnew.VECTOR2I(int(round(p[0] * NM)), int(round(p[1] * NM)))


def writeItem(board, netInfo, it, free=False):
    """free: a via that no track or pad of its net touches (stitching, a return via):
    marked so in KiCad, which otherwise hands such a via the net of whatever plane it
    meets (a ground stitching via would come back as a supply via)."""
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
        t.SetIsFree(free)
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
            if writeItem(board, nets[r.conn.net], it, free=r.conn.kind == "stitch") is not None:
                count[it.kind] += 1
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
    fillZones(board)
    return added


def runDrc(boardPath, outJson):
    # refilled, as a person checks it: the verdict must not depend on how the fill saved
    # in the file came about
    subprocess.run(["kicad-cli", "pcb", "drc", "--format", "json", "--severity-error",
                    "--severity-warning", "--refill-zones", "-o", outJson, boardPath], check=True,
                   capture_output=True)
    with open(outJson) as f:
        return json.load(f)
