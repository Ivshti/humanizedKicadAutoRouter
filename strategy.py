"""Human routing strategy: functional blocks, bundles, layer plan and phase order.

1. Blocks: every multi-pin part (IC, module, connector) anchors a block; passives join
   the anchor they share their most private net with.
2. Connections: per net, a minimum spanning tree over its pads. GND pads each get a
   fanout to the plane instead.
3. Bundles: inter-block signal connections grouped by block pair. Where each bundle runs,
   and on which layer, is decided later by the corridor planner (corridors.py).
4. Phase order: RF (strict), block internals + their GND fanouts, power, bundles.
"""
import math
import re
from collections import defaultdict

from router import Conn

PASSIVE_MAX_PADS = 4
LONG_MM = 15.0
SKIP_REFS = ("FID", "H", "TP")


def isAnchor(fp):
    if fp.ref.startswith(("FID", "H")):
        return False
    return len([p for p in fp.pads if p.net]) > PASSIVE_MAX_PADS


def octile(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)


def buildBlocks(model, planeNets):
    anchors = [fp for fp in model.footprints.values() if isAnchor(fp)]
    block = {fp.ref: fp.ref for fp in anchors}
    netSize = {n: len({p.ref for p in pads}) for n, pads in model.nets.items()}
    pending = [fp for fp in model.footprints.values() if fp.ref not in block]
    for _ in range(3):
        rest = []
        for fp in pending:
            score = defaultdict(float)
            for p in fp.pads:
                if not p.net or p.net in planeNets:
                    continue
                for q in model.nets[p.net]:
                    if q.ref != fp.ref and q.ref in block:
                        d = octile(p.center, q.center)
                        score[block[q.ref]] += 1.0 / netSize[p.net] + 0.2 / (1 + d)
            if score:
                block[fp.ref] = max(score, key=score.get)
            else:
                rest.append(fp)
        pending = rest
    for fp in pending:  # no signal nets in common with anything: nearest anchor
        if anchors:
            block[fp.ref] = min(anchors, key=lambda a: octile(a.pos, fp.pos)).ref
    return block


def mstEdges(pads):
    if len(pads) < 2:
        return []
    inTree = {0}
    edges = []
    best = {i: (octile(pads[0].center, pads[i].center), 0) for i in range(1, len(pads))}
    while best:
        i = min(best, key=lambda k: best[k][0])
        d, j = best.pop(i)
        edges.append((pads[j], pads[i], d))
        inTree.add(i)
        for k in best:
            dk = octile(pads[i].center, pads[k].center)
            if dk < best[k][0]:
                best[k] = (dk, i)
    return edges


def powerEdges(pads, flows, trunkMinAmps=0.3):
    """A power net the way a person lays it: a trunk from the source straight to each
    big consumer (largest first, the first trunk out of a source sized for everything
    that source feeds), then every other pad tapped onto the nearest copper."""
    edges = []
    inTree = []
    flows = sorted(flows, key=lambda f: -f["amps"])
    leaving = defaultdict(float)
    for f in flows:
        leaving[f["source"]] += f["amps"]
    for f in flows:
        src = [p for p in pads if p.ref == f["source"]]
        share = f["amps"] / max(len(f["sinks"]), 1)
        if not src or share < trunkMinAmps:
            continue
        for sinkRef in f["sinks"]:
            dst = [p for p in pads if p.ref == sinkRef]
            if not dst:
                continue
            a, b = min(((p, q) for p in src for q in dst), key=lambda pq: octile(pq[0].center, pq[1].center))
            amps = max(share, leaving[f["source"]])
            leaving[f["source"]] -= share
            edges.append((a, b, octile(a.center, b.center), amps))
            inTree += [p for p in (a, b) if p not in inTree]
    # small sinks: their share of current, if any, rides on the tap that reaches them
    tapAmps = defaultdict(float)
    for f in flows:
        share = f["amps"] / max(len(f["sinks"]), 1)
        if share < trunkMinAmps:
            for sinkRef in f["sinks"]:
                tapAmps[sinkRef] += share
    # pins of one part that sources or sinks a big flow share that flow between them, so
    # the copper joining them is sized for all of it
    partAmps = defaultdict(float)
    for f in flows:
        share = f["amps"] / max(len(f["sinks"]), 1)
        if share >= trunkMinAmps:
            partAmps[f["source"]] = max(partAmps[f["source"]], f["amps"])
            for sinkRef in f["sinks"]:
                partAmps[sinkRef] = max(partAmps[sinkRef], share)
    rest = [p for p in pads if p not in inTree]
    if not inTree:
        inTree, rest = [rest[0]], rest[1:]
    while rest:
        p, q = min(((p, q) for p in rest for q in inTree), key=lambda pq: octile(pq[0].center, pq[1].center))
        amps = partAmps[p.ref] if p.ref == q.ref else tapAmps.pop(p.ref, 0.0)
        edges.append((q, p, octile(p.center, q.center), amps))
        inTree.append(p)
        rest.remove(p)
    return edges


PAIR_SUFFIXES = (("_DP", "_DN"), ("_P", "_N"), ("+", "-"), ("H", "L"))


def pairPartners(nets):
    """Differential pairs by name: X_DP/X_DN, X_P/X_N, X+/X-, and CANH/CANL style."""
    out = {}
    for n in nets:
        for p, q in PAIR_SUFFIXES:
            if n.endswith(p) and len(n) > len(p):
                m = n[:-len(p)] + q
                if m in nets and (p, q) != ("H", "L") or (m in nets and n.upper().startswith("CAN")):
                    out[n], out[m] = m, n
                break
    return out


def linkPairs(conns, partners):
    """Pair each connection of a pair's first net with the connection of its partner that
    joins the same two parts; the second follows the first."""
    byNet = defaultdict(list)
    for c in conns:
        byNet[c.net].append(c)
    n = 0
    for net, other in partners.items():
        if net > other:
            continue
        for c in byNet[net]:
            ends = {c.a.ref, c.b.ref}
            if len(ends) < 2:   # a part's own pass-through pins: nothing to pair
                continue
            mates = [d for d in byNet[other] if d.pairLeader is None and {d.a.ref, d.b.ref} == ends]
            if mates:
                mates[0].pairLeader = c
                c.pairMate = mates[0]
                mates[0].pairMate = c
                n += 1
    return n


# reference prefixes of parts whose GND pads take the pour rather than a via of their own
POUR_PREFIXES = {"J", "P", "CN", "CON", "X", "USB", "SW", "S", "TP", "H", "R", "RN", "L", "FB", "D", "LED", "F"}


def fanoutWanted(pad, padCount, mode):
    """Whether a GND pad gets its own fanout via. "all": every one. "caps-ics":
    capacitors (the via is part of the decoupling loop) and multi-pin parts other than
    connectors; a resistor's, diode's or connector's GND pad takes the pour."""
    if mode == "all":
        return True
    if mode == "none":
        return False
    if mode != "caps-ics":
        raise ValueError(f"fanoutParts must be 'all', 'caps-ics' or 'none', not {mode!r}")
    m = re.match(r"[A-Za-z]+", pad.ref)
    prefix = m.group(0).upper() if m else ""
    if prefix == "C":
        return True
    if prefix in POUR_PREFIXES:
        return False
    return padCount > 3


def plan(model, router, log=print, flows=()):
    planeNets = router.planeNets
    block = buildBlocks(model, planeNets)
    conns = []
    cid = 0

    def add(**kw):
        nonlocal cid
        cid += 1
        c = Conn(cid, **kw)
        conns.append(c)
        return c

    for net, pads in model.nets.items():
        if len(pads) < 2 and net not in planeNets:
            continue
        cls = model.netClass.get(net, "Default")
        kind = "rf" if cls == "RF50" else ("pwr" if router.powerClass(net) else "sig")
        netFlows = [f for f in flows if f["net"] == net]
        edges = powerEdges(pads, netFlows) if netFlows else [(a, b, d, 0.0) for a, b, d in mstEdges(pads)]
        if net in planeNets:
            # pads over the plane each get a fanout via; pads it does not reach (a plane
            # that covers part of the board) are wired to the nearest pad of the net
            over = {p.pid for p in pads if router.onPlane(net, p.center)}
            for p in pads:
                if p.smd and p.pid in over:
                    c = add(net=net, a=p, b=None, kind="gnd", block=block[p.ref], prefLayer=p.side,
                            strict=False)
                    # only a net poured on the outer layers can take a pad without a via
                    c.deferred = net in router.outerPourNets and not fanoutWanted(
                        p, len(router.padsOfRef.get(p.ref, ())), router.k.fanoutParts)
            edges = [e for e in edges if e[0].pid not in over or e[1].pid not in over]
        for a, b, d, amps in edges:
            ba, bb = block[a.ref], block[b.ref]
            c = add(net=net, a=a, b=b, kind=kind, block=ba if ba == bb else "")
            c.order = d
            c.amps = amps
            if kind == "rf":
                c.strict = a.side == b.side or not (a.smd and b.smd)
                c.prefLayer = a.side if a.smd else b.side
            if ba != bb:
                c.bundle = "|".join(sorted((ba, bb)))

    bundles = defaultdict(list)
    for c in conns:
        if c.bundle and c.kind == "sig":
            bundles[c.bundle].append(c)
    for c in conns:
        if router.k.blockLayer == "own" and c.kind in ("sig", "pwr") and not c.bundle \
                and c.a.smd and c.b.smd and c.a.side == c.b.side:
            c.layerLock = c.a.side   # block wiring stays on its parts' side
        if c.kind in ("sig", "pwr") and c.prefLayer < 0 and c.order <= LONG_MM:
            # block internals stay on the side their parts sit on; long runs pick the
            # layer whose main diagonal suits them (router.mainAxis)
            c.prefLayer = c.a.side if c.a.side == c.b.side else -1
    nPairs = linkPairs(conns, pairPartners(set(model.nets)))
    log(f"plan: {len(conns)} connections, {len(set(block.values()))} blocks, {len(bundles)} bundles, "
        f"{nPairs} differential pair links")
    return conns, block, bundles


def phaseOrder(conns):
    """Strategy order: RF, a block's own differential pairs, block internals (shortest
    first) with their GND, power, bundles. A block's pairs go early so both tracks can be
    laid side by side before other wiring takes the room next to their pins; a pair
    between blocks runs with its bundle in its corridor."""
    def key(c):
        if c.kind == "rf":
            return (0, 0, c.order)
        if c.pairMate is not None and not c.bundle:
            return (0, 1, c.order)
        if c.kind in ("sig", "pwr") and c.block:
            return (1, -c.amps, c.order)
        if c.kind == "gnd":
            return (2, 0, 0)
        if c.kind == "pwr":
            return (3, -c.amps, c.order)
        return (4, 0, c.bundle, c.order)
    return followLeaders(sorted(conns, key=key))


def followLeaders(order):
    """The second net of a differential pair is routed right after the first."""
    present = {id(c) for c in order}
    followers = {id(c.pairLeader): c for c in order if c.pairLeader is not None}
    out = []
    for c in order:
        if c.pairLeader is not None and id(c.pairLeader) in present:
            continue
        out.append(c)
        if id(c) in followers:
            out.append(followers[id(c)])
    return out
