"""Autoroute a placed KiCad board from scratch, the way a person would.

    kicad-python tools/humanRouter/run.py <board.kicad_pcb> --out <dir>

Writes <dir>/<name>.kicad_pcb (plus the project, rules and footprint library it needs),
fills the planes, runs DRC and prints a summary. The input board is never modified.
"""
import argparse
import fnmatch
import json
import math
import re
import os
import shutil
import sys
import time
from collections import Counter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import pcbnew
from shapely.geometry import Point

from boardModel import (B, F, INNER_KICAD, LAYER_BY_NAME, LAYER_NAMES, PHYSICAL, boardPlanes,
                        existingCopper, fillZones, innerPlaneFills, loadModel, planeLayersOf,
                        polySetToShapely, readCourtyardClearances, readLayerNets, readSkewRules,
                        ripCopper)
from emit import addPlanes, runDrc, writeItem, writeRoutes
from corridors import Corridors, splitBundles
from flowcheck import squeezes
from router import Knobs, Router, ipcWidth
from strategy import followLeaders, pairPartners, phaseOrder, plan
from tidy import Judge, tidy


SIGNAL_LAYERS = [F, B]   # set from the router's stack in main()
POWER_LAYERS = {}        # plane layer: the power nets the design rules let run on it (main())


def layersFor(c):
    if c.kind == "gnd":
        return [c.a.side]
    if c.layerLock >= 0:
        return [c.layerLock]
    if c.strict:
        return [c.prefLayer]
    if c.kind == "pwr":
        return list(SIGNAL_LAYERS) + [l for l, nets in POWER_LAYERS.items() if c.net in nets]
    return list(SIGNAL_LAYERS)


def winPadsFor(c):
    if c.kind == "gnd":
        return (1.0, 2.5, 5.0)
    return (2.0, 6.0, 25.0)


def viaMmFor(c, k):
    if c.kind == "gnd":
        # a supply-plane fanout would rather share a via of its net a short walk away
        # than set its own beside it; ground fanouts are cheap (the pour joins them anyway)
        return 0.4 if c.net == "GND" else 1.5
    if c.kind == "pwr" and any(c.net in nets for nets in POWER_LAYERS.values()):
        return k.viaMm   # power with an inner layer of its own: a trunk there, drops by via
    # a person keeps power paths on one layer; a tap that carries no current (a flow net's
    # gate hold or sense pin) is priced like a signal
    if c.amps > 1.0 or (c.kind == "pwr" and (c.trunk or c.amps > 0)):
        return k.viaMm * 4
    return k.viaMm


def negotiate(router, queue, log, maxRounds=40, deadline=None, winPads=None):
    """Push and shove at route granularity: a connection that cannot get through may cross
    other routes at a price; those are ripped and go back on the queue. Bounded by the
    deadline and by maxRounds attempts per connection."""
    k = router.k
    rounds = 0
    attempts = Counter()
    while queue:
        if deadline and time.perf_counter() > deadline:
            log("negotiation: deadline reached")
            break
        rounds += 1
        c = queue.pop(0)
        if c.routed and router.connected(c):
            continue
        wp = winPads or winPadsFor(c)
        st, _ = router.route(c, layersFor(c), wp, viaMm=viaMmFor(c, k))
        if st == "ok":
            continue
        attempts[c.cid] += 1
        if attempts[c.cid] > maxRounds:
            continue
        router.k.negotiatePenalty = 4.0 * (1.3 ** min(attempts[c.cid], 12))
        st, payload = router.route(c, layersFor(c), wp, negotiate=True, viaMm=viaMmFor(c, k))
        if st == "conflict":
            res, items, victims = payload
            ripped = []
            for v in victims:
                router.rip(v, ripped)
            # remember the fight: future searches pay to go there again
            for it in items:
                wwin, mask = router.sp.rasterMask(it.geom)
                if mask.size:
                    i0, j0, i1, j1 = wwin
                    for l in it.layers:
                        router.hist[l, i0:i1, j0:j1][mask] += 0.5
            # re-verify after ripping (ripping can only remove obstacles)
            bad = [o for it in items for o in router.check(it)]
            if bad:
                queue.append(c)
            else:
                router.commit(c, res, items)
            # a ripped stitching via is not put back: the field has vias to spare
            queue += [r for r in ripped if r is not c and r.kind != "stitch"]
        elif st == "nudge":
            # only the designer's copper is in the way: nudge it aside (Router.nudge), the
            # routes in the way as well ripped and queued again as for a conflict
            res, items, victims, pieces = payload
            ripped = []
            for v in victims:
                router.rip(v, ripped)

            def place(c=c, res=res, items=items):
                if any(o.route not in pieces for it in items for o in router.check(it)):
                    return None
                router.commit(c, res, items)
                return [c.rid]
            if not router.nudge(pieces, place, log):
                router.nudgeRefused.add((c.cid, frozenset(pieces)))
                queue.append(c)
            queue += [r for r in ripped if r is not c and r.kind != "stitch"]
        elif st != "ok":
            queue.append(c)
    return rounds


def routeAll(router, conns, bundles, block, log, maxRounds=40, deadline=None):
    k = router.k
    # GND pads left to the pour get a via only if the pour misses them (after DRC)
    deferred = sum(1 for c in conns if c.deferred)
    order = phaseOrder([c for c in conns if not c.deferred])
    t0 = time.perf_counter()
    failed = []
    router.corridors = Corridors(router, k.corridorTileMm)
    groups = splitBundles(bundles, block, k.joinMm, k.maxGroup)
    router.corridors.chooseAxes(groups, log)
    # block wiring (a block's own pairs early), GND and power first; bundles last, a pair
    # between blocks with its bundle in its corridor (laid first, a long pair took the room
    # the Nano's power needed: negotiation 59 -> ~150 rounds)
    first = [c for c in order if not (c.bundle and c.kind == "sig")]
    later = [c for c in order if c.bundle and c.kind == "sig"]
    router.reserveEscapes(order, log)
    # the pairs that escaped go first, as a person lays a bus out of a BGA or connector:
    # inside of the turn first; one that cannot be laid is routed net by net in its place
    for c in router.escapeOrder(order):
        router.routePair(c, c.pairMate, layersFor(c), winPadsFor(c), viaMmFor(c, k))
    relaxed = 0
    for c in first:
        st, _ = router.route(c, layersFor(c), winPadsFor(c), viaMm=viaMmFor(c, k))
        if st != "ok" and c.layerLock >= 0:
            # its parts' layer has no way through: the other layer, at its usual price
            c.layerLock = -1
            relaxed += 1
            st, _ = router.route(c, layersFor(c), winPadsFor(c), viaMm=viaMmFor(c, k))
        if st != "ok":
            failed.append(c)
    # block wiring, GND and power are down: plan the channels for what is left, then lay
    # each group's members in order across its width so they run side by side
    router.corridors.plan(groups, log)
    for c in followLeaders(sorted(later, key=lambda c: (c.groupRank, c.groupPos))):
        st, _ = router.route(c, layersFor(c), winPadsFor(c), viaMm=viaMmFor(c, k))
        if st != "ok":
            failed.append(c)
    router.dropFronts()   # every pair has been tried: the room in front of escapes is free
    log(f"pass 1: {len(order) - len(failed)}/{len(order)} routed in {time.perf_counter() - t0:.1f}s, "
        f"{len(failed)} failed: {Counter(c.kind for c in failed)}; {deferred} GND pads left to the pour"
        + (f"; {relaxed} block connections had to leave their parts' layer" if relaxed else ""))
    # GND pads the fanout could not reach are left to the outer pour; they are judged by DRC
    signal = [c for c in conns if not (c.kind == "gnd" and c.net == "GND")]
    before = sum(1 for c in signal if not router.connected(c))
    router.begin()
    rounds = negotiate(router, [c for c in failed if not (c.kind == "gnd" and c.net == "GND")],
                       log, maxRounds, deadline)
    after = sum(1 for c in signal if not router.connected(c))
    if after > before and router.rollback():
        log(f"negotiation left {after} connections apart, pass 1 left {before}: pass 1's result kept")
    router.commitTxn()
    # a ripped route may have been holding other pads together than its own two
    unrouted = reconnect(router, [c for c in conns if not (c.kind == "gnd" and c.net == "GND")],
                         log, deadline)
    locked = [c for c in unrouted if c.layerLock >= 0]
    if locked:
        # block wiring that cannot stay on its parts' layer may use the other one after all
        for c in locked:
            c.layerLock = -1
        negotiate(router, list(locked), log, maxRounds, deadline)
        unrouted = reconnect(router, [c for c in conns if not (c.kind == "gnd" and c.net == "GND")],
                             log, deadline)
        log(f"layer lock: {len(locked)} block connections allowed onto the other layer, "
            f"{sum(1 for c in locked if not router.connected(c))} still unrouted")
    log(f"negotiation: {rounds} rounds, {len(unrouted)} unrouted: "
        + ", ".join(f"{c.net}:{c.a.ref}.{c.a.num}->{(c.b.ref + '.' + c.b.num) if c.b else 'plane'}"
                    for c in unrouted[:30]))
    return unrouted


def finishBoard(board, router, model, keepZones, conns, keepTracks=(), innerGnd=False, log=print,
                openPour=True):
    """Write the routes into the board (copper not in keepZones / keepTracks removed
    first), pour ground on the outer layers that have no ground zone of their own (and
    on the inner ones when innerGnd) and stitch the outer pours to the plane (openPour:
    also a via every stitchPitchMm of open pour). Returns tracks, vias, stitching vias
    added, GND fanouts the pour needs."""
    ripCopper(board, keep=keepZones, keepTracks=keepTracks)
    nT, nV = writeRoutes(board, router)
    if "GND" not in router.planeNets:
        fillZones(board)
        return nT, nV, 0, []
    planes = [(l, "GND", True) for l, kl in ((F, pcbnew.F_Cu), (B, pcbnew.B_Cu))
              if not any(z.GetNetname() == "GND" and z.IsOnLayer(kl) for z in keepZones)]
    if innerGnd:   # the board has no inner planes: ground on the default stack's plane layers
        planes += [(l, "GND", True) for l in sorted(router.planeLayers)]
    # fill with every island kept (the board's own ground zones too), so stitching sees the
    # pieces that would otherwise be removed
    own = [z for z in keepZones if z.GetNetname() == "GND"]
    modes = [z.GetIslandRemovalMode() for z in own]
    for z in own:
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_NEVER)
    added = addPlanes(board, model.outline, planes, removeIslands=False)
    if router.k.stitch:
        nSt, bare, stranded = stitchPours(board, router, conns, openPour=openPour)
        log(f"stitching: {nSt} GND vias; {bare} pieces of pour with no room for a via")
    else:
        nSt, stranded = 0, []
    for z, m in zip(own, modes):
        z.SetIslandRemovalMode(m)
    for z in added:
        board.Remove(z)
    addPlanes(board, model.outline, planes)
    return nT, nV + nSt, nSt, stranded


def staleCopper(board, router):
    """Pieces of the board's own copper that serve nothing: they touch none of their net's
    pads and no zone of their net on an outer layer (a via and a stub on a plane where a
    part used to be, an orphan track). KiCad's DRC does not flag a via on a plane. rids."""
    fills = {}
    for z in board.Zones():
        if z.GetIsRuleArea():
            continue
        for layer, lid in ((F, pcbnew.F_Cu), (B, pcbnew.B_Cu)):
            if z.IsOnLayer(lid):
                g = polySetToShapely(z.GetFilledPolysList(lid))
                if g is not None:
                    key = (z.GetNetname(), layer)
                    fills[key] = g if key not in fills else fills[key].union(g)
    out = []
    for rid in router.padlessPieces():
        net = router.existing[rid][0]
        live = False
        for m in router.existingItems[rid]:
            for layer in ((m.layers[0],) if m.kind == "track" else (F, B)):
                g = fills.get((net, layer))
                if g is not None and m.geom.intersects(g):
                    live = True
        if not live:
            out.append(rid)
    return out


def pourTargets(board, router, net="GND"):
    """{layer: board raster} of the outer pour pieces of `net` that hold a via or a
    through-hole pad of it (stitched to the plane), from the board's current fill, at
    least 0.8 mm from the net's vias."""
    sp = router.sp
    nid = router.netId[net]
    rv = router.rules[net].viaDia / 2
    anchors = [Point(it.pts[0]) for it in sp.items.values() if it.net == nid and it.kind == "via"]
    anchors += [Point(p.center) for p in router.m.nets.get(net, []) if not p.smd]
    out = {}
    for layer, lid in ((F, pcbnew.F_Cu), (B, pcbnew.B_Cu)):
        m = np.zeros((sp.H, sp.W), bool)
        for z in board.Zones():
            if z.GetNetname() != net or not z.IsOnLayer(lid) or z.GetIsRuleArea():
                continue
            fill = polySetToShapely(z.GetFilledPolysList(lid))
            for piece in getattr(fill, "geoms", [fill]) if fill is not None else []:
                near = piece.buffer(rv + 0.05)
                if not any(near.contains(a) for a in anchors):
                    continue
                inner = piece.buffer(-0.05)
                win, mask = sp.rasterMask(inner) if not inner.is_empty else (None, np.zeros(0))
                if mask.size:
                    i0, j0, i1, j1 = win
                    m[i0:i1, j0:j1] |= mask
        out[layer] = m
    # a pad's track ends in open pour, not on a stitching via (that would read as a pad
    # tied to a via, which a person does not draw)
    awayFromVias = sp.maskDist(sp.viaAt, (0, 0, sp.H, sp.W), reach=0, pred=lambda a: a == nid) >= 0.8
    for layer in out:
        out[layer] &= awayFromVias
    return out


def repairUnreached(board, router, conns, log):
    """Pads the pour does not reach, under fanoutParts=none (no via beside a GND pad):
    a passive's or connector's pad gets a short track to the nearest stitched pour on its
    layer; a chip's exposed pad gets a via inside it; a chip's pin gets a fanout that
    prefers the inside of the pin ring. A via beside the pad, at least stitchPadGapMm
    from every pad, is the last resort. A custom-shaped pad (KiCad's thermal spokes
    degenerate on its anchor) gets a solid connection to the pour instead of a track.
    Returns (the connections left for push and shove, pads switched to solid)."""
    targets = pourTargets(board, router)
    done = Counter()
    fanout = []
    solid = 0
    for c in conns:
        fp = board.FindFootprintByReference(c.a.ref)
        pad = fp.FindPadByNumber(c.a.num) if fp is not None else None
        if pad is not None and pad.GetShape() == pcbnew.PAD_SHAPE_CUSTOM and \
                pad.GetLocalZoneConnection() != pcbnew.ZONE_CONNECTION_FULL:
            pad.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
            done["solid connection (custom-shaped pad)"] += 1
            solid += 1
            continue
        if router.isExposedPad(c.a):
            if router.viaInPad(c):
                done["via in the exposed pad"] += 1
                continue
        elif not router.isRingChip(c.a):
            if router.routeToPour(c, targets[c.a.side]):
                done["track to the pour"] += 1
                continue
            c.viaGapMm = router.k.stitchPadGapMm   # last resort: a via beside it, kept clear
            done["via beside the pad (last resort)"] += 1
        else:
            done["via inside the pin ring"] += 1
        c.deferred = False
        fanout.append(c)
    log("pads the pour does not reach: " + (", ".join(f"{n} {how}" for how, n in done.items()) or "none"))
    return fanout, solid


def repairMissedGnd(board, router, missed, log):
    """GND pads the pour misses under fanoutParts "none". gndRepair "report": listed for the
    designer, no copper added; "join": repairUnreached ties them. Returns (connections left
    for push and shove, pads switched to solid)."""
    if router.k.gndRepair == "join":
        return repairUnreached(board, router, missed, log)
    if missed:
        log("pads the pour does not reach, left to the designer: "
            + ", ".join(sorted(f"{c.a.ref}.{c.a.num}" for c in missed)))
    return [], 0


def powerCopper(router, model, log):
    """Power tracks keep the width their class (or their --flows current) sets; with the
    fatten knob each is widened to the widest that fits (Router.fatten). Returns how many
    tracks were widened."""
    if not router.k.fatten:
        log("power copper: class widths (fatten off)")
        return 0
    # a net the designer put in the Rail class is a light rail (a backlight cathode, a
    # reference): it keeps its class width unless a flow says it carries current
    powerNets = {n for n in model.netClass if router.powerClass(n) and model.netClass[n] != "Rail"} \
        | router.flowNets
    n = router.fatten(powerNets)
    log(f"power copper: {n} tracks widened")
    return n


SIDECAR = ".humanrouter.json"   # next to a board: the copper the router drew on it


def itemKey(t):
    """Where a board track or via is: a later run trusts its record only while this holds."""
    if t.GetClass() == "PCB_VIA":
        p = t.GetPosition()
        return ["via", t.GetNetname(), p.x, p.y, t.GetWidth(pcbnew.F_Cu)]
    a, b = t.GetStart(), t.GetEnd()
    return ["track", t.GetNetname(), t.GetLayerName(), a.x, a.y, b.x, b.y, t.GetWidth()]


def readRouterCopper(board, path):
    """uuids of the board's tracks and vias an earlier run drew and nobody has moved since."""
    if not os.path.exists(path):
        return set()
    with open(path) as f:
        rec = json.load(f)
    return {t.m_Uuid.AsString() for t in board.GetTracks()
            if rec.get(t.m_Uuid.AsString()) == itemKey(t)}


def writeRouterCopper(board, path, designer):
    """Record the copper on the board that is not the designer's (designer: uuids)."""
    rec = {t.m_Uuid.AsString(): itemKey(t) for t in board.GetTracks()
           if t.m_Uuid.AsString() not in designer}
    with open(path, "w") as f:
        json.dump(rec, f)
    return len(rec)


def offPlaneFanouts(drc, router):
    """Plane fanouts that carry copper DRC reports unconnected: from each unconnected
    track, via or pad of a partial plane's net, up through the routes it hangs off to the
    fanout whose via should reach the plane but does not (it sits outside what the
    plane's fill really reaches). Their connections."""
    out = []
    for u in drc["unconnected_items"]:
        for it in u["items"]:
            m = re.search(r"\[([^\]]+)\]", it["description"])
            net = m.group(1) if m else None
            if net not in router.planeCells:
                continue
            p = Point(it["pos"]["x"], it["pos"]["y"])
            todo = [r for r in router.routes.values() if r.conn.net == net
                    and any(router.sp.items[i].geom.distance(p) < 0.05 for i in r.items)]
            seen = set()
            while todo:
                r = todo.pop()
                if r.rid in seen:
                    continue
                seen.add(r.rid)
                if r.conn.kind == "gnd" and r.conn.b is None and r.conn not in out:
                    out.append(r.conn)
                todo += [router.routes[q] for q in r.parents if q in router.routes]
    return out


def realPlaneFills(board, router):
    """{net: inner fill} of the partial planes, as the board is filled now."""
    fills = {}
    for z in board.Zones():
        if z.GetIsRuleArea() or z.GetNetname() not in router.planeCells:
            continue
        for kl in INNER_KICAD:
            if z.IsOnLayer(kl):
                g = polySetToShapely(z.GetFilledPolysList(kl))
                if g is not None:
                    n = z.GetNetname()
                    fills[n] = g if n not in fills else fills[n].union(g)
    return fills


def strandedGnd(drc, conns):
    """GND fanout connections whose pad DRC reports as unconnected."""
    byPad = {(c.a.ref, c.a.num): c for c in conns if c.kind == "gnd"}
    out = []
    for u in drc["unconnected_items"]:
        for it in u["items"]:
            m = re.search(r"[Pp]ad (\S+) \[GND\] of (\S+)", it["description"])
            c = m and byPad.get((m.group(2), m.group(1)))
            if c and not c.routed and c not in out:
                out.append(c)
    return out


def fixStarvedThermals(board, drc):
    """A GND pad whose thermal spokes are blocked gets a solid pour connection instead."""
    n = 0
    for v in drc["violations"]:
        if v["type"] != "starved_thermal":
            continue
        for it in v["items"]:
            if it["description"].startswith("Zone"):  # the other side of the report
                continue
            m = re.search(r"[Pp]ad (\S+) \[[^]]*\] of (\S+)", it["description"])
            if not m:
                print(f"thermals: cannot tell which pad from {it['description']!r}", file=sys.stderr)
                continue
            fp = board.FindFootprintByReference(m.group(2))
            for pad in fp.Pads():
                if pad.GetNumber() == m.group(1):
                    pad.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
                    n += 1
    return n


def gndMissed(drc):
    """GND pads KiCad's DRC reports apart from the rest of ground: ["REF.pad", ...]."""
    return sorted({m.group(2) + "." + m.group(1) for u in drc["unconnected_items"] for it in u["items"]
                   for m in [re.search(r"[Pp]ad (\S+) \[GND\] of (\S+)", it["description"])] if m})


def stitchPours(board, router, conns, net="GND", openPour=True):
    """Stitch the outer pours of `net` (filled, islands kept) to the plane: every piece
    gets stitchPerIsland vias or through-hole pads of the net, so none is fed only through
    pad thermals, and (openPour) open pour gets a via every stitchPitchMm. Vias keep stitchPadGapMm
    of bare board to every pad and stay out of rule areas. Returns (vias added, pieces
    that could not get even one, fanout connections whose pad sits on such a piece)."""
    k, sp = router.k, router.sp
    nid = router.netId[net]
    rule = router.rules[net]
    rv = rule.viaDia / 2
    ok = router.stitchMask(net)
    anchors = [it.pts[0] for it in sp.items.values() if it.net == nid and it.kind == "via"]
    anchors += [p.center for p in router.m.nets.get(net, []) if not p.smd]
    pieces = []
    for layer, lid in ((F, pcbnew.F_Cu), (B, pcbnew.B_Cu)):
        for z in board.Zones():
            if z.GetNetname() != net or not z.IsOnLayer(lid) or z.GetIsRuleArea():
                continue
            fill = polySetToShapely(z.GetFilledPolysList(lid))
            if fill is not None:
                pieces += [(layer, g) for g in getattr(fill, "geoms", [fill])]
    added, bare, starved = 0, 0, []
    fanouts = [c for c in conns if c.kind == "gnd" and c.net == net and not c.routed]

    netInfo = board.GetNetsByName()[net]

    def place(cands):
        nonlocal added
        pt = router.placePlaneVia(cands, net)
        if pt is None:
            return None
        # the routes are already written: this one goes onto the board now
        writeItem(board, netInfo, router.viaItem(net, pt, rule, 0), free=True)
        anchors.append(pt)
        i, j = sp.toCell(*pt)
        r = int(0.9 / sp.g)
        ok[max(i - r, 0):i + r + 1, max(j - r, 0):j + r + 1] = False
        added += 1
        return pt
    inPour = np.zeros_like(ok)
    for layer, piece in pieces:
        room = piece.buffer(-(rv + 0.1))
        if room.is_empty:
            continue
        win, mask = sp.rasterMask(room)
        if mask.size:
            i0, j0, i1, j1 = win
            inPour[i0:i1, j0:j1] |= mask
    relaxed = None   # the fanout gap instead of the stitching gap, for a piece short of vias
    for layer, piece in pieces:
        near = piece.buffer(rv + 0.05)
        have = [a for a in anchors if near.contains(Point(a))]
        room = piece.buffer(-(rv + 0.1))
        for gapPass in (0, 1):
            if len(have) >= k.stitchPerIsland or room.is_empty:
                break
            if gapPass == 1:
                if relaxed is None:
                    # judged afresh: the vias placed so far are on the board already
                    relaxed = router.stitchMask(net, padGap=k.fanoutViaGapMm)
                mask_ok = relaxed
            else:
                mask_ok = ok
            win, mask = sp.rasterMask(room)
            i0, j0, i1, j1 = win
            ii, jj = np.nonzero(mask & mask_ok[i0:i1, j0:j1])
            cands = [sp.toMm(i + i0, j + j0) for i, j in zip(ii, jj)]
            for _ in range(k.stitchPerIsland - len(have)):
                if not cands:
                    break
                # as far as possible from what the piece already has: spread out
                ref = have or [cands[len(cands) // 2]]
                cands.sort(key=lambda c: -min(math.dist(c, h) for h in ref))
                pt = place(cands[:40])
                if pt is None:
                    break
                have.append(pt)
                cands = [c for c in cands if math.dist(c, pt) > rule.viaDrill + 0.6]
                if relaxed is not None:
                    i, j = sp.toCell(*pt)
                    r = int(0.9 / sp.g)
                    relaxed[max(i - r, 0):i + r + 1, max(j - r, 0):j + r + 1] = False
        if not have:
            bare += piece.area >= 1.0   # slivers between pads do not count
            starved += [c for c in fanouts if c not in starved and layer in c.a.layers
                        and piece.intersects(c.a.poly)]
    if not openPour:
        return added, bare, starved
    # open pour: one via per grid node that has none of its own yet
    p = k.stitchPitchMm
    minx, miny, maxx, maxy = router.m.outline.bounds
    y = miny + p / 2
    while y < maxy:
        x = minx + p / 2
        while x < maxx:
            if not any(math.dist(a, (x, y)) < p / 2 for a in anchors):
                place(router.nearestOk(ok & inPour, x, y, p / 3))
            x += p
        y += p
    return added, bare, starved


def audit(router, log, phase):
    """HR_AUDIT=1: check the model after each phase; HR_PROBE="x,y;x,y": also list what the
    model holds at those points (mm)."""
    if not os.environ.get("HR_AUDIT"):
        return
    router.audit(log, phase)
    for net in filter(None, os.environ.get("HR_PROBE_NET", "").split(";")):
        for rid, r in sorted(router.routes.items()):
            if r.conn.net != net:
                continue
            ends = [(it.kind, it.layers[0] if it.kind == "track" else "*", it.pts)
                    for it in (router.sp.items[i] for i in r.items)]
            log(f"probe {phase} route {rid} {net} {r.conn.a.ref}.{r.conn.a.num}->"
                f"{r.conn.b.ref + '.' + r.conn.b.num if r.conn.b else 'plane'} nodes {r.nodes} "
                f"parents {sorted(r.parents)} children {sorted(router.children.get(rid, ()))} items {ends}")
    for xy in filter(None, os.environ.get("HR_PROBE", "").split(";")):
        x, y = map(float, xy.split(","))
        for it in router.sp.query(router.allLayers, (x - 0.05, y - 0.05, x + 0.05, y + 0.05)):
            if it.geom.buffer(1e-3).contains(Point(x, y)):
                r = router.routes.get(it.route)
                owner = f"{r.conn.net} {r.conn.a.ref}.{r.conn.a.num}->" \
                        f"{r.conn.b.ref + '.' + r.conn.b.num if r.conn.b else 'plane'}" if r else "no route"
                log(f"probe {phase} ({x},{y}): {it.kind} net {router.netName.get(it.net, it.net)} "
                    f"layers {it.layers} pts {it.pts} route {it.route} ({owner}) tag {it.tag!r}")


def reconnect(router, conns, log, deadline=None):
    """Connections the tidying passes left apart are routed again (push and shove if
    need be); returns what is still apart."""
    apart = [c for c in conns if not router.connected(c)]
    if apart:
        # a finished connection is only traded away if the total gets better
        router.begin()
        negotiate(router, list(apart), log, maxRounds=10, deadline=deadline)
        after = [c for c in conns if not router.connected(c)]
        if len(after) >= len(apart) and router.rollback():
            log(f"reconnect: no better ({len(apart)} -> {len(after)} apart); put back as it was")
        router.commitTxn()
        apart = [c for c in conns if not router.connected(c)]
        log(f"reconnect: {len(apart)} connections still apart")
    return apart


def cleanup(router, log, deadline=None, rounds=2):
    """Second pass, as a person tidies a board: worst detours first, each leaf route ripped
    and searched again against the finished board."""
    k = router.k
    for rnd in range(rounds):
        ranked = []
        for rid, r in router.routes.items():
            c = r.conn
            if c.kind not in ("sig", "pwr", "rf") or c.b is None:
                continue
            cost, nVia = router.routeCost(rid)
            ideal = math.dist(c.a.center, c.b.center) + 1e-3
            ranked.append((cost / ideal, rid))
        ranked.sort(reverse=True)
        better = 0
        for badness, rid in ranked:
            if badness < 1.25:
                break
            if deadline and time.perf_counter() > deadline:
                break
            c = router.routes[rid].conn if rid in router.routes else None
            if c is None:
                continue
            if router.improve(rid, layersFor(c), winPadsFor(c), viaMmFor(c, k)):
                better += 1
        log(f"cleanup round {rnd + 1}: {better} routes improved of {sum(1 for b, _ in ranked if b >= 1.25)} candidates")
        audit(router, log, f"cleanup {rnd + 1}")
        if not better:
            break
    # vias last: with the board finished, a detour that was blocked earlier may now be
    # the better trade for a via pair
    withVias = sorted(((router.routeCost(rid)[1], rid) for rid, r in router.routes.items()
                       if r.conn.kind in ("sig", "pwr") and r.conn.b is not None), reverse=True)
    before = sum(n for n, _ in withVias)
    for n, rid in withVias:
        if not n or (deadline and time.perf_counter() > deadline):
            break
        c = router.routes[rid].conn if rid in router.routes else None
        if c is not None:
            router.improve(rid, layersFor(c), winPadsFor(c), viaMmFor(c, k) * k.viaCleanupMult)
    # a signal that hops a track with two vias: the track makes room instead
    unhopped = 0
    for rid in sorted(router.routes):
        if deadline and time.perf_counter() > deadline:
            break
        r = router.routes.get(rid)
        if r is not None and r.conn.kind in ("sig", "pwr") and \
                router.unhop(rid, layersFor, winPadsFor, lambda c: viaMmFor(c, k)):
            unhopped += 1
    after = sum(router.routeCost(rid)[1] for rid, r in router.routes.items() if r.conn.kind in ("sig", "pwr"))
    log(f"via cleanup: {before} -> {after} signal/power vias ({unhopped} hops undone by moving what they crossed)")
    audit(router, log, "via cleanup")
    # straighten: every route searched once more with corners priced high, kept when it
    # has fewer of them and is no longer
    bendy = sorted(((router.routeBends(rid), rid) for rid, r in router.routes.items()
                    if r.conn.kind in ("sig", "pwr") and r.conn.b is not None), reverse=True)
    before = sum(n for n, _ in bendy)
    fixed = 0
    for n, rid in bendy:
        if n < 2 or (deadline and time.perf_counter() > deadline):
            break
        c = router.routes[rid].conn if rid in router.routes else None
        if c is not None and router.improve(rid, layersFor(c), winPadsFor(c), viaMmFor(c, k), straighten=True):
            fixed += 1
    after = sum(router.routeBends(rid) for rid, r in router.routes.items() if r.conn.kind in ("sig", "pwr"))
    log(f"straighten: {fixed} routes, {before} -> {after} corners")
    audit(router, log, "straighten")
    log(f"via tidying: {router.tidyVias()} vias moved in line with their pads")
    log(f"corners: {router.chamferCorners()} right angles cut into two 45-degree bends")
    audit(router, log, "via tidying")
    # differential pairs that ended up apart: route both nets again, together
    pairs = {id(r.conn): r.conn for r in router.routes.values() if r.conn.pairLeader is not None}
    for follow in pairs.values():
        lead = follow.pairLeader
        was = router.coupling(lead, follow)
        if was < 0.5 and router.repairPair(lead, follow, layersFor(lead), winPadsFor(lead), viaMmFor(lead, k)):
            log(f"pair {lead.net}/{follow.net}: coupled {was:.0%} -> {router.coupling(lead, follow):.0%}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("board")
    ap.add_argument("--out", required=True)
    ap.add_argument("--minutes", type=float, default=25.0)
    ap.add_argument("--flows", help="JSON {flows: [{net, source, sinks, amps}]} sizing power copper")
    ap.add_argument("--knob", action="append", default=[], metavar="NAME=VALUE",
                    help="override a routing knob (see Knobs in router.py), e.g. viaMm=2.5")
    ap.add_argument("--keep-copper", action="store_true",
                    help="keep every track, via and zone already on the board and route only "
                         "what is missing")
    ap.add_argument("--route", choices=("all", "blocks"), default="all",
                    help="blocks: only the wiring inside functional blocks and GND fanouts; the "
                         "links between blocks are left to you")
    ap.add_argument("--nets", help="comma-separated net names or globs: route only these")
    ap.add_argument("--drop-stale", action="store_true",
                    help="with --keep-copper: remove the board's copper that touches none of its "
                         "net's pads (left where a part used to be); without it, it is only listed")
    ap.add_argument("--tidy", choices=("on", "off", "report", "only"), default="on",
                    help="clean the copper up at the end, like KiCad's Cleanup Tracks & Vias (on, "
                         "the default); report: list what it would change and change nothing; "
                         "only: route nothing, just clean up the board's copper")
    ap.add_argument("--stitch-only", action="store_true",
                    help="route nothing: keep the board's copper, add return vias beside its signal "
                         "vias and stitch its outer ground pours to the plane")
    ap.add_argument("--seed", type=int, default=0,
                    help="run number: the same board, options and seed give the same result; another "
                         "seed tries the connections in a slightly different order (default 0)")
    args = ap.parse_args()
    if os.environ.get("PYTHONHASHSEED") != "0":
        # Python orders sets of names differently in every process unless told otherwise:
        # start again with that order fixed, so a run can be repeated exactly
        os.execve(sys.executable, [sys.executable] + sys.argv, {**os.environ, "PYTHONHASHSEED": "0"})
    if args.stitch_only or args.tidy == "only":
        args.keep_copper = True
    t0 = time.perf_counter()
    deadline = t0 + args.minutes * 60

    def log(msg):
        print(f"[{time.perf_counter() - t0:7.1f}s] {msg}", flush=True)

    src = os.path.abspath(args.board)
    srcDir = os.path.dirname(src)
    name = os.path.splitext(os.path.basename(src))[0]
    os.makedirs(args.out, exist_ok=True)
    for ext in (".kicad_pro", ".kicad_dru"):
        if os.path.exists(os.path.join(srcDir, name + ext)):
            shutil.copy(os.path.join(srcDir, name + ext), os.path.join(args.out, name + ext))
    # the footprint library table points back at the source board's libraries
    libTable = os.path.join(srcDir, "fp-lib-table")
    if os.path.exists(libTable):
        rel = os.path.relpath(srcDir, os.path.abspath(args.out))
        with open(libTable) as f:
            text = f.read().replace("${KIPRJMOD}", "${KIPRJMOD}/" + rel)
        with open(os.path.join(args.out, "fp-lib-table"), "w") as f:
            f.write(text)
    outBoard = os.path.join(args.out, name + ".kicad_pcb")

    board = pcbnew.LoadBoard(src)
    boardZones = boardPlanes(board)
    ripCopper(board, keep=boardZones)
    model = loadModel(board, os.path.join(srcDir, name + ".kicad_pro"))
    model.planes = innerPlaneFills(board, boardZones, model.outline) if boardZones else {"GND": None}
    # an inner layer with a zone of the board's own is a plane; without any, the default stack
    model.planeLayers = planeLayersOf(boardZones) if boardZones else None
    log(f"model: {len(model.pads)} pads, {len(model.nets)} nets, {len(model.keepouts)} keepouts")
    log("planes: " + ", ".join(f"{n} ({'whole board' if f is None else f'{f.area:.0f} mm2'})"
                               for n, f in model.planes.items())
        + ("" if boardZones else " (none on the board: ground on the default stack's plane layers)"))
    knobs = Knobs()
    for kv in args.knob:
        key, value = kv.split("=", 1)
        if not hasattr(knobs, key):
            raise SystemExit(f"unknown knob {key!r}")
        cur = getattr(knobs, key)
        setattr(knobs, key, (value.lower() in ("1", "true", "yes")) if isinstance(cur, bool) else type(cur)(value))
        log(f"knob {key} = {getattr(knobs, key)}")
    if knobs.gndRepair not in ("report", "join"):
        raise SystemExit(f"gndRepair must be 'report' or 'join', not {knobs.gndRepair!r}")
    knobs.seed = args.seed or knobs.seed
    router = Router(model, knobs, log)
    SIGNAL_LAYERS[:] = router.signalLayers
    log("stack: " + " / ".join(
        f"{LAYER_NAMES[l]} " + ("plane " + "+".join(sorted(router.planeLayers[l])) if l in router.planeLayers
                                else "signal") for l in PHYSICAL))
    # a plane layer the design rules open to some power nets carries them, the plane
    # poured round them (the Nano's In2)
    POWER_LAYERS.clear()
    for lname, nets in readLayerNets(os.path.join(srcDir, name + ".kicad_dru")).items():
        l = LAYER_BY_NAME.get(lname)
        extra = {n for n in nets if n in model.nets and router.powerClass(n)} - router.planeLayers.get(l, set())
        if l in router.planeLayers and extra:
            POWER_LAYERS[l] = extra
            log(f"power on {lname} (design rules): {', '.join(sorted(extra))}")
    router.powerLayers = dict(POWER_LAYERS)
    courtyards = readCourtyardClearances(os.path.join(srcDir, name + ".kicad_dru"))
    if courtyards:
        router.setCourtyardClearances(courtyards)
        log("clearance inside courtyards (design rules): "
            + ", ".join(f"{ref} {mm:g} mm" for ref, mm in sorted(courtyards.items())))
    log(f"space: {router.sp.W}x{router.sp.H} cells @ {router.k.grid} mm")
    keepZones, keepTracks, designer = boardZones, set(), set()
    if args.keep_copper:
        # the output is the board as it is plus the new copper; its own copper is fixed
        board = pcbnew.LoadBoard(src)
        tracks, vias, keepTracks = existingCopper(board)
        soft = readRouterCopper(board, os.path.join(srcDir, name + SIDECAR))
        designer = keepTracks - soft
        keepZones = [z for z in board.Zones() if not z.GetIsRuleArea()]
        if boardZones:
            # the planes as the board's own copper leaves them: its vias cut the fill
            fills = innerPlaneFills(board, boardPlanes(board), model.outline, keepIslands=True)
            router.setPlaneFills(fills)
            for net, fill in fills.items():
                pieces = sorted((g for g in getattr(fill, "geoms", [fill]) if g.area >= 1.0),
                                key=lambda g: -g.area) if fill is not None else []
                if len(pieces) > 1:
                    where = "; ".join(f"{g.area:.0f} mm2 at x {g.bounds[0]:.1f}-{g.bounds[2]:.1f}, "
                                      f"y {g.bounds[1]:.1f}-{g.bounds[3]:.1f}" for g in pieces)
                    log(f"WARNING: the {net} plane is in {len(pieces)} pieces ({where}): the "
                        f"board's own copper cuts it. The router treats it as one plane, so pads "
                        f"it ties to different pieces stay apart; join the pieces on the board")
        log(f"existing copper kept: {len(tracks)} tracks, {len(vias)} vias, {len(keepZones)} zones "
            f"({router.loadExisting(tracks, vias, soft)} items; {len(soft)} drawn by an earlier run, "
            f"which may be moved)")
        stale = staleCopper(board, router)
        if stale:
            where = []
            for rid in stale:
                its = router.existingItems[rid]
                p = its[0].pts[0]
                where.append(f"{router.existing[rid][0]} at ({p[0]:.2f}, {p[1]:.2f}): "
                             f"{sum(1 for i in its if i.kind == 'track')} tracks, "
                             f"{sum(1 for i in its if i.kind == 'via')} vias")
            if args.drop_stale:
                for rid in stale:
                    keepTracks -= router.dropExisting(rid)
            log(f"copper touching no pad of its net ({'removed' if args.drop_stale else 'kept; --drop-stale removes it'}): "
                + "; ".join(where))
    if args.tidy == "only":
        # route nothing, pour nothing: clean the board's own copper up and check it
        if args.drop_stale:
            ripCopper(board, keep=keepZones, keepTracks=keepTracks)
        judge = Judge(os.path.join(args.out, "tidy"), name, os.path.join(srcDir, name + ".kicad_pro"),
                      os.path.join(srcDir, name + ".kicad_dru"))
        tidy(board, judge, log)
        board.Save(outBoard)
        drc = runDrc(outBoard, os.path.join(args.out, "drc.json"))
        viol = Counter(v["type"] for v in drc["violations"])
        log(f"DRC: {sum(viol.values())} violations {dict(viol)}, {len(drc['unconnected_items'])} unconnected")
        return
    flows = []
    if args.flows:
        with open(args.flows) as f:
            flows = json.load(f)["flows"]
    router.flowNets = {f["net"] for f in flows}
    for f in flows:
        if "minWidthMm" in f:
            router.flowFloor[f["net"]] = max(router.flowFloor.get(f["net"], 0.0), float(f["minWidthMm"]))
    conns, block, bundles = plan(model, router, log, flows)
    router.allConns = conns
    router.inputNets = {c.net for c in conns if c.input}
    if router.k.nudge:
        log(f"nudge: {router.markMovable()} pieces of the designer's copper may be nudged aside")
    total = len(conns)
    if args.route == "blocks":
        conns = [c for c in conns if c.kind == "gnd" or not c.bundle]
    if args.nets:
        pats = [p.strip() for p in args.nets.split(",") if p.strip()]
        conns = [c for c in conns if any(fnmatch.fnmatchcase(c.net, p) for p in pats)]
    if len(conns) < total:
        keep = {id(c) for c in conns}
        bundles = {b: [c for c in m if id(c) in keep] for b, m in bundles.items()}
        bundles = {b: m for b, m in bundles.items() if m}
        log(f"routing {len(conns)} of {total} connections; the rest are left to you")
    if args.keep_copper:
        done = sum(1 for c in conns if router.connected(c))
        log(f"{done} of them are already made by the copper on the board")
    if args.stitch_only:
        conns, unrouted = [], []
        log("stitch only: nothing is routed")
    else:
        for c in sorted((c for c in conns if c.amps > 0), key=lambda c: -c.amps)[:6]:
            log(f"  current: {c.net} {c.a.ref}->{c.b.ref} {c.amps:.2f} A -> {router.ruleForConn(c).width:.2f} mm")
        # negotiation may use most of the budget; cleanup and planes need the rest
        unrouted = routeAll(router, conns, bundles, block, log,
                            deadline=t0 + args.minutes * 60 * 0.75)
        audit(router, log, "negotiation")
        cleanup(router, log, deadline=deadline)
        unrouted = reconnect(router, [c for c in conns if not (c.kind == "gnd" and c.net == "GND")],
                             log, deadline)
        if unrouted:   # named from the final state: the tidying passes may have traded nets
            log(f"unrouted after tidying: {len(unrouted)}: "
                + ", ".join(f"{c.net}:{c.a.ref}.{c.a.num}->{(c.b.ref + '.' + c.b.num) if c.b else 'plane'}"
                            for c in unrouted[:30]))
        skewRules = readSkewRules(os.path.join(srcDir, name + ".kicad_dru"))

        def skewBudget(net):
            base = net[:-1]  # KiCad's diff-pair base name: the net without its P/N/+/- suffix
            for pat, mm in skewRules:
                if fnmatch.fnmatchcase(base, pat) or fnmatch.fnmatchcase(net, pat):
                    return mm, True
            return router.k.pairSkewMm, False
        # pairs this run routed completely; one with copper of its own on the board is the
        # designer's to match
        mine = {c.net for c in conns}
        drawn = {n for n, _ in router.existing.values()}
        pairs = {a: b for a, b in pairPartners(set(model.nets)).items()
                 if {a, b} <= mine and not {a, b} & drawn}
        router.matchPairs(pairs, skewBudget, log)
        audit(router, log, "length matching")
        log(f"search stats: {router.stats}")
        viaBy = Counter()
        for r in router.routes.values():
            c = r.conn
            n = sum(1 for i in r.items if router.sp.items[i].kind == "via")
            if c.kind in ("sig", "pwr") and n:
                where = "bundle" if c.bundle else "block"
                side = "same side" if c.b is not None and c.a.side == c.b.side and c.a.smd and c.b.smd else "cross/THT"
                viaBy[f"{c.kind} {where} {side}"] += n
            else:
                viaBy[c.kind] += n
        log(f"vias by kind: {dict(viaBy)}")
        if powerCopper(router, model, log):
            audit(router, log, "power copper")
    if "GND" in router.planeNets and router.k.stitch:
        # a routing run stitches around its own vias; --stitch-only around everyone's
        log(f"return vias: {router.returnVias(ownOnly=not args.stitch_only)} GND vias beside signal vias")
    finish = lambda: finishBoard(board, router, model, keepZones, conns, keepTracks - router.droppedUuids,
                                 not boardZones, log, openPour=args.stitch_only or not args.keep_copper)
    nT, nV, nIsl, islandPads = finish()
    board.Save(outBoard)
    log(f"wrote {outBoard}: {nT} tracks, {nV} vias")
    drc = runDrc(outBoard, os.path.join(args.out, "drc.json"))
    for _ in range(2):
        # GND pads whose piece of pour ended up cut off: fan them out to the plane after
        # all, pushing other routes aside, and write the board again
        found = strandedGnd(drc, conns)
        stranded = found + [c for c in islandPads if not c.routed and c not in found]
        offPlane = offPlaneFanouts(drc, router)
        if offPlane:
            # route them again where the plane's fill really is
            router.setPlaneFills({**{n: None for n in router.planeNets if n not in router.planeCells},
                                  **realPlaneFills(board, router)})
            ripped = []
            for c in offPlane:
                if c.rid in router.routes:
                    router.rip(c.rid, ripped)   # and what hangs off it
            log(f"plane fanouts outside the plane's fill: {len(offPlane)} routed again "
                f"({len(ripped)} connections with them)")
            stranded += [c for c in offPlane + ripped if c not in stranded]
        if router.k.fanoutParts == "none":
            # those pads take the pour by design; one it misses is reported, or with
            # gndRepair "join" joined to the pour (a via beside it only if nothing else works)
            missed = [c for c in stranded if c.deferred]
            fan, solid = repairMissedGnd(board, router, missed, log)
            stranded = [c for c in stranded if not c.deferred] + fan
            repaired = [c for c in missed if router.connected(c)]
        else:
            repaired, solid = [], 0
        if not stranded and not repaired and not solid:
            break
        if stranded:
            negotiate(router, list(stranded), log, maxRounds=10, winPads=(2.0, 6.0, 15.0))
            log(f"stranded GND pads: {sum(1 for c in stranded if router.connected(c))}/{len(stranded)} "
                f"fanned out to the plane")
            audit(router, log, "stranded GND repair")
        fixed = sum(1 for c in stranded + repaired if router.connected(c)) + solid
        if not fixed:
            break
        nT, nV, nIsl, islandPads = finish()
        board.Save(outBoard)
        drc = runDrc(outBoard, os.path.join(args.out, "drc.json"))
    for _ in range(3):  # a pad switched to solid can reveal the next one
        solid = fixStarvedThermals(board, drc)
        if not solid:
            break
        fillZones(board)
        board.Save(outBoard)
        log(f"thermals: {solid} GND pads switched to solid pour connection")
        drc = runDrc(outBoard, os.path.join(args.out, "drc.json"))
    if gndMissed(drc):
        log(f"GND pads the pour does not reach: {', '.join(gndMissed(drc))}")
    if args.tidy != "off":
        judge = Judge(os.path.join(args.out, "tidy"), name, os.path.join(srcDir, name + ".kicad_pro"),
                      os.path.join(srcDir, name + ".kicad_dru"))
        # under --nets only those nets' copper is cleaned, and under --keep-copper only
        # what this router drew (the designer's copper is theirs)
        scope = {n for n in model.nets if any(fnmatch.fnmatchcase(n, p) for p in pats)} if args.nets else None
        tidy(board, judge, log, report=args.tidy == "report", nets=scope, fixed=designer)
        if args.tidy != "report":
            board.Save(outBoard)
            drc = runDrc(outBoard, os.path.join(args.out, "drc.json"))
    viol = Counter(v["type"] for v in drc["violations"])
    log(f"DRC: {sum(viol.values())} violations {dict(viol)}, {len(drc['unconnected_items'])} unconnected")
    if gndMissed(drc):   # judged on the board as written, after the clean-up
        log(f"WARNING GND pads cut off from ground on the finished board: {', '.join(gndMissed(drc))}")
    if flows:
        want = lambda net, amps: max(router.rules[net].width, ipcWidth(amps, router.k),
                                     router.flowFloor.get(net, 0.0))
        bad = squeezes(board, flows, want, router.k.neckMm)
        for f, sink, w, thin in bad:
            where = f"{f['net']} {f['source']} -> {sink} ({f['amps']} A, trunk {w:.2f} mm)"
            if thin is None:
                log(f"WARNING current path {where}: no copper path")
                continue
            log(f"WARNING current path {where} runs through thinner copper: " +
                ", ".join(f"{L} mm at {tw} mm on {layer} near {xy}" for tw, L, layer, xy in thin))
        log(f"current paths: {len(bad)} of the flows squeezed")
    if router.droppedUuids:
        log(f"moved: {len(router.droppedUuids)} tracks and vias an earlier run drew were ripped to make room")
    log(f"router copper recorded: {writeRouterCopper(board, os.path.join(args.out, name + SIDECAR), designer)} "
        f"items in {name + SIDECAR}")
    summary = dict(minutes=(time.perf_counter() - t0) / 60, tracks=nT, vias=nV,
                   violations=dict(viol), unconnected=len(drc["unconnected_items"]),
                   unroutedConns=len(unrouted), leftToYou=total - len(conns))
    with open(os.path.join(args.out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=1)


if __name__ == "__main__":
    main()
