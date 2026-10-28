"""Read a KiCad board into plain geometry: pads, nets, net classes, keepouts, outline.

Runs under KiCad's bundled Python (pcbnew + shapely). Units are mm. Layer indices: 0 is
F.Cu and 1 is B.Cu on every board, and In<k>.Cu is k + 1, so the outer layers keep their
index whatever the layer count. PHYSICAL lists them in stack order (F first, B last).
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
F, B = 0, 1
# filled by setStack() for the board being routed (4 layers until a board says otherwise);
# updated in place, so every module that imported them sees the board's stack
KICAD_LAYERS = {}        # index -> pcbnew layer id
LAYER_NAMES = {}         # index -> "F.Cu", "In1.Cu", ...
LAYER_BY_NAME = {}       # "In1.Cu" -> index
INNER = []               # inner layer indices, in stack order
INNER_KICAD = []         # their pcbnew ids
PHYSICAL = []            # every layer index in stack order


def setStack(nCopper):
    """Layer tables for a board with nCopper copper layers (2, 4, 6, 8, ...)."""
    if nCopper < 2 or nCopper % 2:
        raise ValueError(f"{nCopper} copper layers: an even count of 2 or more is expected")
    for t in (KICAD_LAYERS, LAYER_NAMES, LAYER_BY_NAME):
        t.clear()
    del INNER[:], INNER_KICAD[:], PHYSICAL[:]
    KICAD_LAYERS[F], KICAD_LAYERS[B] = pcbnew.F_Cu, pcbnew.B_Cu
    LAYER_NAMES[F], LAYER_NAMES[B] = "F.Cu", "B.Cu"
    for k in range(1, nCopper - 1):
        KICAD_LAYERS[k + 1] = getattr(pcbnew, f"In{k}_Cu")
        LAYER_NAMES[k + 1] = f"In{k}.Cu"
        INNER.append(k + 1)
        INNER_KICAD.append(KICAD_LAYERS[k + 1])
    LAYER_BY_NAME.update({n: i for i, n in LAYER_NAMES.items()})
    PHYSICAL.extend([F] + INNER + [B])


def defaultPlaneLayers(nCopper):
    """Inner layers that carry ground on a board with no inner zones of its own, so every
    signal layer sits next to a plane: 4 layers S/G/G/S, 6 layers S/G/S/S/G/S, 8 layers
    S/G/S/G/G/S/G/S; more layers alternate plane and signal pairs the same way."""
    inner = [k + 1 for k in range(1, nCopper - 1)]
    if nCopper <= 4:
        return inner
    if nCopper == 6:
        return [inner[0], inner[3]]
    if nCopper == 8:
        return [inner[0], inner[2], inner[3], inner[5]]
    # S/G/S/G ... G/S/G/S: a plane after every signal layer, mirrored about the middle
    half = len(inner) // 2
    top = [inner[i] for i in range(0, half, 2)]
    return top + [inner[-1 - (l - inner[0])] for l in top]


def planeLayersOf(zones):
    """{inner layer index: nets of the zones on it} for the board's own inner zones."""
    out = {}
    for z in zones:
        for l in INNER:
            if z.IsOnLayer(KICAD_LAYERS[l]):
                out.setdefault(l, set()).add(z.GetNetname())
    return out


def stackOf(board):
    """Set the layer tables from the board's own copper layer count; returns it."""
    n = board.GetCopperLayerCount()
    if len(KICAD_LAYERS) != n:
        setStack(n)
    return n


setStack(4)


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


def polySetToShapely(sps, scale=NM):
    """A KiCad SHAPE_POLY_SET as shapely geometry, in mm (scale=1: in KiCad's nm)."""
    polys = []
    for i in range(sps.OutlineCount()):
        o = sps.Outline(i)
        pts = [(o.CPoint(j).x / scale, o.CPoint(j).y / scale) for j in range(o.PointCount())]
        holes = []
        for h in range(sps.HoleCount(i)):
            hc = sps.Hole(i, h)
            holes.append([(hc.CPoint(j).x / scale, hc.CPoint(j).y / scale) for j in range(hc.PointCount())])
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
    stackOf(board)
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
            # an edge-connector pad (gold finger) is a surface pad too: no hole reaches a plane
            surface = attr in (pcbnew.PAD_ATTRIB_SMD, pcbnew.PAD_ATTRIB_CONN)
            pm = Pad(fp.GetReference(), str(pad.GetNumber()), str(pad.GetNetname()), layers, poly,
                     center, surface, drill, side, len(pads))
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


def existingCopper(board):
    """The board's tracks [(net, layer, p1, p2, width, uuid)] (an arc as two chords) and vias
    [(net, p, dia, drill, uuid)], in mm, plus the uuids of all of them."""
    stackOf(board)
    tracks, vias, uuids = [], [], set()
    mm = lambda v: (v.x / NM, v.y / NM)
    for t in board.GetTracks():
        uid = t.m_Uuid.AsString()
        uuids.add(uid)
        net = t.GetNetname()
        if t.GetClass() == "PCB_VIA":
            vias.append((net, mm(t.GetPosition()), t.GetWidth(pcbnew.F_Cu) / NM, t.GetDrill() / NM, uid))
            continue
        layer = LAYER_BY_NAME.get(t.GetLayerName())
        if layer is None:
            raise ValueError(f"track of {net} on {t.GetLayerName()}, not a copper layer of this board")
        w = t.GetWidth() / NM
        if t.GetClass() == "PCB_ARC":
            a, m, b = mm(t.GetStart()), mm(t.GetMid()), mm(t.GetEnd())
            tracks += [(net, layer, a, m, w, uid), (net, layer, m, b, w, uid)]
        else:
            tracks.append((net, layer, mm(t.GetStart()), mm(t.GetEnd()), w, uid))
    return tracks, vias, uuids


def boardPlanes(board):
    """The board's own planes: its copper zones on the inner layers (teardrops are not
    planes). They are kept as the designer drew them; everything else is routed afresh."""
    stackOf(board)
    zones = [board.GetArea(i) for i in range(board.GetAreaCount())]
    return [z for z in zones if not z.GetIsRuleArea() and not z.IsTeardropArea()
            and z.GetNetname() and any(z.IsOnLayer(l) for l in INNER_KICAD)]


def fillZones(board):
    """Fill every zone the way KiCad itself does: with the board's connectivity rebuilt
    first. Island removal decides from connectivity, so after tracks were added from
    Python a fill without the rebuild keeps or drops pieces differently from KiCad's own
    refill."""
    board.BuildConnectivity()
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())


def innerPlaneFills(board, zones, outline, keepIslands=False):
    """{net: inner copper of that net's planes} as the board fills them (on the ripped
    board: around pads and holes only); a plane that covers the board is None.
    keepIslands: count the pieces nothing of the net touches yet (new fanouts may reach
    them) instead of letting the fill drop them."""
    modes = [(z, z.GetIslandRemovalMode()) for z in zones]
    if keepIslands:
        for z, _ in modes:
            z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_NEVER)
    fillZones(board)
    for z, m in modes:
        z.SetIslandRemovalMode(m)
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


def readLayerNets(druPath):
    """Nets a .kicad_dru lets onto a layer that it otherwise keeps tracks off: {layer name:
    set of nets} from rules of the form (condition "A.Type == 'Track' && A.NetName != 'X'
    && A.NetName != 'Y' ...") (layer "In2.Cu") (constraint disallow track). Rules of any
    other form are left to KiCad's DRC."""
    if not os.path.exists(druPath):
        return {}
    with open(druPath) as f:
        text = f.read()
    out = {}
    for rule in re.split(r"\(rule\s", text)[1:]:
        layer = re.search(r'\(layer\s+"([^"]+)"\)', rule)
        cond = re.search(r'\(condition\s+"([^"]*)"\)', rule)
        if not layer or not cond or not re.search(r"\(constraint\s+disallow\s+track\)", rule):
            continue
        terms = [t.strip() for t in cond.group(1).split("&&")]
        nets = [m.group(1) for t in terms for m in [re.fullmatch(r"A\.NetName\s*!=\s*'([^']+)'", t)] if m]
        if nets and all(t == "A.Type == 'Track'" or re.fullmatch(r"A\.NetName\s*!=\s*'[^']+'", t) for t in terms):
            out.setdefault(layer.group(1), set()).update(nets)
    return out


def readCourtyardClearances(druPath):
    """Clearances a .kicad_dru sets inside a part's courtyard: {ref: mm} from rules with an
    `insideCourtyard('REF')` condition and a `(constraint clearance (min X))` (a BGA's
    fanout rule: one track between two balls)."""
    if not os.path.exists(druPath):
        return {}
    with open(druPath) as f:
        text = f.read()
    out = {}
    for rule in re.split(r"\(rule\s", text)[1:]:
        m = re.search(r"\(constraint\s+clearance\s+\(min\s+([\d.]+)\s*(mm|mil)?\)", rule)
        if not m:
            continue
        mm = float(m.group(1)) * (0.0254 if m.group(2) == "mil" else 1.0)
        for ref in re.findall(r"insideCourtyard\('([^']+)'\)", rule):
            out[ref] = mm
    return out
