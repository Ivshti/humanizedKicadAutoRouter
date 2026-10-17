"""Read a KiCad board into plain geometry: pads, nets, net classes, keepouts, outline.

Runs under KiCad's bundled Python (pcbnew + shapely). Units are mm; layer indices are
stack order: 0 F.Cu, 1 In1.Cu, 2 In2.Cu, 3 B.Cu.
"""
import fnmatch
import json
import os
import re
from dataclasses import dataclass, field

import pcbnew
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

NM = 1e6
F, IN1, IN2, B = 0, 1, 2, 3
KICAD_LAYERS = {F: pcbnew.F_Cu, IN1: pcbnew.In1_Cu, IN2: pcbnew.In2_Cu, B: pcbnew.B_Cu}
LAYER_NAMES = {F: "F.Cu", IN1: "In1.Cu", IN2: "In2.Cu", B: "B.Cu"}


@dataclass
class NetClass:
    name: str
    width: float
    clearance: float
    viaDia: float
    viaDrill: float
    pairGap: float        # differential pair gap (edge to edge)


@dataclass
class Pad:
    ref: str
    num: str
    net: str
    layers: list          # copper layer indices the pad exists on
    poly: object          # shapely polygon (same footprint on every layer it exists on)
    center: tuple
    smd: bool
    drill: float          # 0 for SMD
    side: int             # F or B: the footprint's side
    pid: int = -1
    ownClearance: float = 0.0   # pad-level clearance / solder mask opening that overrides the class
    polys: dict = field(default_factory=dict)   # layer -> copper shape when the padstack differs


@dataclass
class Keepout:
    poly: object
    layers: list
    noTracks: bool
    noVias: bool
    noFill: bool


@dataclass
class Footprint:
    ref: str
    value: str
    pos: tuple
    side: int
    pads: list = field(default_factory=list)
    courtyard: object = None


@dataclass
class Model:
    outline: object
    classes: dict
    netClass: dict        # net name -> class name
    nets: dict            # net name -> [Pad]
    pads: list
    footprints: dict
    keepouts: list
    holes: list           # (x, y, radius, net) of every drilled hole that is not a via
    minHoleClearance: float
    minHoleToHole: float
    edgeClearance: float
    planes: dict = field(default_factory=dict)  # plane net -> inner copper (shapely), None = whole board


def polySetToShapely(sps):
    polys = []
    for i in range(sps.OutlineCount()):
        o = sps.Outline(i)
        pts = [(o.CPoint(j).x / NM, o.CPoint(j).y / NM) for j in range(o.PointCount())]
        holes = []
        for h in range(sps.HoleCount(i)):
            hc = sps.Hole(i, h)
            holes.append([(hc.CPoint(j).x / NM, hc.CPoint(j).y / NM) for j in range(hc.PointCount())])
        if len(pts) >= 3:
            polys.append(Polygon(pts, holes))
    return unary_union(polys) if polys else None


def readClasses(proPath):
    with open(proPath) as f:
        pro = json.load(f)
    ns = pro["net_settings"]
    classes = {}
    for c in ns["classes"]:
        classes[c["name"]] = NetClass(c["name"], c["track_width"], c["clearance"],
                                      c["via_diameter"], c["via_drill"],
                                      max(c.get("diff_pair_gap") or 0.0, c["clearance"]))
    patterns = [(p["pattern"], p["netclass"]) for p in (ns.get("netclass_patterns") or [])]
    rules = pro["board"]["design_settings"]["rules"]
    return classes, patterns, rules


def loadModel(board, proPath):
    classes, patterns, rules = readClasses(proPath)

    outlineSet = pcbnew.SHAPE_POLY_SET()
    board.GetBoardPolygonOutlines(outlineSet, False)
    outline = polySetToShapely(outlineSet)

    netClass = {}
    for name in board.GetNetsByName().keys():
        name = str(name)
        cls = "Default"
        for pat, c in patterns:
            if fnmatch.fnmatchcase(name, pat):
                cls = c
                break
        netClass[name] = cls

    pads, nets, footprints, holes = [], {}, {}, []
    keepouts = []
    for fp in board.GetFootprints():
        side = B if fp.IsFlipped() else F
        p = fp.GetPosition()
        fpm = Footprint(fp.GetReference(), fp.GetValue(), (p.x / NM, p.y / NM), side)
        crt = fp.GetCourtyard(pcbnew.B_CrtYd if side == B else pcbnew.F_CrtYd)
        fpm.courtyard = polySetToShapely(crt) if crt.OutlineCount() else None
        for pad in fp.Pads():
            attr = pad.GetAttribute()
            c = pad.GetPosition()
            center = (c.x / NM, c.y / NM)
            if attr == pcbnew.PAD_ATTRIB_NPTH:
                d = pad.GetDrillSize()
                holes.append((center[0], center[1], max(d.x, d.y) / NM / 2, ""))
                continue
            layers = [k for k, kl in KICAD_LAYERS.items() if pad.IsOnLayer(kl)]
            if not layers:
                continue
            refLayer = KICAD_LAYERS[layers[0]]
            poly = polySetToShapely(pad.GetEffectivePolygon(refLayer, pcbnew.ERROR_OUTSIDE))
            if poly is None:
                continue
            drill = 0.0
            if attr == pcbnew.PAD_ATTRIB_PTH:
                d = pad.GetDrillSize()
                drill = max(d.x, d.y) / NM
                holes.append((center[0], center[1], drill / 2, str(pad.GetNetname())))
            pm = Pad(fp.GetReference(), str(pad.GetNumber()), str(pad.GetNetname()), layers, poly,
                     center, attr == pcbnew.PAD_ATTRIB_SMD, drill, side, len(pads))
            for l in layers[1:]:
                pl = polySetToShapely(pad.GetEffectivePolygon(KICAD_LAYERS[l], pcbnew.ERROR_OUTSIDE))
                if pl is not None and not pl.equals(poly):
                    pm.polys[l] = pl
            local = pad.GetLocalClearance()
            mask = pad.GetSolderMaskExpansion(pcbnew.F_Mask if side == F else pcbnew.B_Mask)
            pm.ownClearance = max((local or 0) / NM, mask / NM)
            pads.append(pm)
            fpm.pads.append(pm)
            if pm.net:
                nets.setdefault(pm.net, []).append(pm)
        for z in fp.Zones():
            keepouts.append(zoneToKeepout(z))
        footprints[fpm.ref] = fpm
    for i in range(board.GetAreaCount()):
        z = board.GetArea(i)
        if z.GetIsRuleArea():
            keepouts.append(zoneToKeepout(z))
    keepouts = [k for k in keepouts if k is not None]

    return Model(outline, classes, netClass, nets, pads, footprints, keepouts, holes,
                 rules["min_hole_clearance"], rules["min_hole_to_hole"],
                 rules["min_copper_edge_clearance"])


def zoneToKeepout(z):
    if not z.GetIsRuleArea():
        return None
    layers = [k for k, kl in KICAD_LAYERS.items() if z.GetLayerSet().Contains(kl)]
    poly = polySetToShapely(z.Outline())
    if poly is None or not layers:
        return None
    return Keepout(poly, layers, z.GetDoNotAllowTracks(), z.GetDoNotAllowVias(),
                   z.GetDoNotAllowZoneFills())


INNER_KICAD = (pcbnew.In1_Cu, pcbnew.In2_Cu)


def ripCopper(board, keep=(), keepTracks=()):
    """Remove every track, via and copper zone except the zones in `keep` and the tracks
    and vias whose uuids are in `keepTracks`; rule areas stay (they are design intent)."""
    keepTracks = set(keepTracks)
    for t in list(board.GetTracks()):
        if t.m_Uuid.AsString() not in keepTracks:
            board.Delete(t)
    keep = {z.m_Uuid.AsString() for z in keep}  # wrappers differ per call, uuids do not
    for z in [board.GetArea(i) for i in range(board.GetAreaCount())]:
        if not z.GetIsRuleArea() and z.m_Uuid.AsString() not in keep:
            board.Delete(z)


LAYER_BY_NAME = {"F.Cu": F, "In1.Cu": IN1, "In2.Cu": IN2, "B.Cu": B}


def existingCopper(board):
    """The board's tracks [(net, layer, p1, p2, width)] (an arc as two chords) and vias
    [(net, p, dia, drill)], in mm, plus the uuids of all of them."""
    tracks, vias, uuids = [], [], set()
    mm = lambda v: (v.x / NM, v.y / NM)
    for t in board.GetTracks():
        uuids.add(t.m_Uuid.AsString())
        net = t.GetNetname()
        if t.GetClass() == "PCB_VIA":
            vias.append((net, mm(t.GetPosition()), t.GetWidth(pcbnew.F_Cu) / NM, t.GetDrill() / NM))
            continue
        layer = LAYER_BY_NAME.get(t.GetLayerName())
        if layer is None:
            raise ValueError(f"track of {net} on {t.GetLayerName()}: only 4-layer boards are supported")
        w = t.GetWidth() / NM
        if t.GetClass() == "PCB_ARC":
            a, m, b = mm(t.GetStart()), mm(t.GetMid()), mm(t.GetEnd())
            tracks += [(net, layer, a, m, w), (net, layer, m, b, w)]
        else:
            tracks.append((net, layer, mm(t.GetStart()), mm(t.GetEnd()), w))
    return tracks, vias, uuids


def boardPlanes(board):
    """The board's own planes: its copper zones on the inner layers (teardrops are not
    planes). They are kept as the designer drew them; everything else is routed afresh."""
    zones = [board.GetArea(i) for i in range(board.GetAreaCount())]
    return [z for z in zones if not z.GetIsRuleArea() and not z.IsTeardropArea()
            and z.GetNetname() and any(z.IsOnLayer(l) for l in INNER_KICAD)]


def innerPlaneFills(board, zones, outline):
    """{net: inner copper of that net's planes} filled on the ripped board (pads and holes
    only); a plane that covers the board is None."""
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())  # on the ripped board: these and rule areas
    fills = {}
    for z in zones:
        for l in INNER_KICAD:
            if not z.IsOnLayer(l):
                continue
            fill = polySetToShapely(z.GetFilledPolysList(l))
            if fill is not None:
                net = z.GetNetname()
                fills[net] = fill if net not in fills else fills[net].union(fill)
    return {net: (None if f.area >= 0.9 * outline.area else f) for net, f in fills.items()}


def readSkewRules(druPath):
    """Skew budgets from a .kicad_dru: [(diff-pair name pattern, max mm)] from rules with
    `A.inDiffPair('pattern')` conditions and a `(constraint skew (max X))`."""
    if not os.path.exists(druPath):
        return []
    with open(druPath) as f:
        text = f.read()
    out = []
    for rule in re.split(r"\(rule\s", text)[1:]:
        m = re.search(r"\(constraint\s+skew\s+\(max\s+([\d.]+)\s*(mm|mil)?\)", rule)
        if not m:
            continue
        mm = float(m.group(1)) * (0.0254 if m.group(2) == "mil" else 1.0)
        for pat in re.findall(r"inDiffPair\('([^']+)'\)", rule):
            out.append((pat, mm))
    return out
