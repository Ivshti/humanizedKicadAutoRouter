"""Write routes and planes into a pcbnew board, fill zones, run DRC."""
import json
import subprocess

import pcbnew

from boardModel import KICAD_LAYERS

NM = 1e6


def v2(p):
    return pcbnew.VECTOR2I(int(round(p[0] * NM)), int(round(p[1] * NM)))


def writeRoutes(board, router):
    nets = board.GetNetsByName()
    nTracks = nVias = 0
    for r in router.routes.values():
        ni = nets[r.conn.net]
        for iid in r.items:
            it = router.sp.items[iid]
            if it.kind == "track":
                t = pcbnew.PCB_TRACK(board)
                t.SetStart(v2(it.pts[0]))
                t.SetEnd(v2(it.pts[1]))
                t.SetWidth(int(round(it.width * NM)))
                t.SetLayer(KICAD_LAYERS[it.layers[0]])
                t.SetNet(ni)
                board.Add(t)
                nTracks += 1
            elif it.kind == "via":
                v = pcbnew.PCB_VIA(board)
                v.SetPosition(v2(it.pts[0]))
                v.SetViaType(pcbnew.VIATYPE_THROUGH)
                v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
                v.SetWidth(int(round(it.width * NM)))
                v.SetDrill(int(round(it.drill * NM)))
                v.SetNet(ni)
                board.Add(v)
                nVias += 1
    return nTracks, nVias


def addPlanes(board, outline, planes):
    """planes: [(layerIndex, netName, thermal)] covering the whole board outline."""
    nets = board.GetNetsByName()
    for layer, net, thermal in planes:
        z = pcbnew.ZONE(board)
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
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        board.Add(z)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())


def runDrc(boardPath, outJson):
    subprocess.run(["kicad-cli", "pcb", "drc", "--format", "json", "--severity-error",
                    "--severity-warning", "-o", outJson, boardPath], check=True,
                   capture_output=True)
    with open(outJson) as f:
        return json.load(f)
