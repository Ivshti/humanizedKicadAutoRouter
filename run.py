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

from boardModel import B, F, IN1, IN2, boardPlanes, innerPlaneFills, loadModel, readSkewRules, ripCopper
from emit import addPlanes, floatingIslands, runDrc, writeItem, writeRoutes
from corridors import Corridors, splitBundles
from router import Knobs, Router
from strategy import followLeaders, pairPartners, phaseOrder, plan


def layersFor(c):
    if c.kind == "gnd":
        return [c.a.side]
    if c.strict:
        return [c.prefLayer]
    return [F, B]


def winPadsFor(c):
    if c.kind == "gnd":
        return (1.0, 2.5, 5.0)
    return (2.0, 6.0, 25.0)


def viaMmFor(c, k):
    if c.kind == "gnd":
        return 0.4
    if c.amps > 1.0:   # a person keeps high-current paths on one layer
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
        elif st != "ok":
            queue.append(c)
    return rounds


def routeAll(router, conns, bundles, block, log, maxRounds=40, deadline=None):
    k = router.k
    order = phaseOrder(conns)
    t0 = time.perf_counter()
    failed = []
    router.corridors = Corridors(router, k.corridorTileMm)
    groups = splitBundles(bundles, block, k.joinMm, k.maxGroup)
    router.corridors.chooseAxes(groups, log)
    first = [c for c in order if not (c.bundle and c.kind == "sig")]
    later = [c for c in order if c.bundle and c.kind == "sig"]
    for c in first:
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
    log(f"pass 1: {len(order) - len(failed)}/{len(order)} routed in {time.perf_counter() - t0:.1f}s, "
        f"{len(failed)} failed: {Counter(c.kind for c in failed)}")
    # GND pads the fanout could not reach are left to the outer pour; they are judged by DRC
    rounds = negotiate(router, [c for c in failed if not (c.kind == "gnd" and c.net == "GND")],
                       log, maxRounds, deadline)
    # a ripped route may have been holding other pads together than its own two
    unrouted = reconnect(router, [c for c in conns if not (c.kind == "gnd" and c.net == "GND")],
                         log, deadline)
    log(f"negotiation: {rounds} rounds, {len(unrouted)} unrouted: "
        + ", ".join(f"{c.net}:{c.a.ref}.{c.a.num}->{(c.b.ref + '.' + c.b.num) if c.b else 'plane'}"
                    for c in unrouted[:30]))
    return unrouted


def finishBoard(board, router, model, boardZones, conns):
    """Write the routes into the (ripped) board, pour ground on the outer layers (and on
    the inner ones when the board has no planes of its own) and keep every piece of outer
    pour that can be given a via. Returns tracks, vias, island vias added."""
    ripCopper(board, keep=boardZones)
    nT, nV = writeRoutes(board, router)
    if "GND" not in router.planeNets:
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        return nT, nV, 0, []
    planes = [(F, "GND", True), (B, "GND", True)]
    if not boardZones:
        planes += [(IN1, "GND", True), (IN2, "GND", True)]
    added = addPlanes(board, model.outline, planes, removeIslands=False)
    nIsl, stranded = islandVias(board, router, conns)
    for z in added:
        board.Remove(z)
    addPlanes(board, model.outline, planes)
    return nT, nV + nIsl, nIsl, stranded


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


def islandVias(board, router, conns, net="GND"):
    """Pieces of the outer GND pours that reach no via (only surface pads, or nothing)
    would be cut off from the ground: each one with room for a via gets one down to the
    inner planes. Returns (vias added, fanout connections of pads on pieces with no
    room), the latter to be routed with push and shove."""
    nid = router.netId[net]
    netInfo = board.GetNetsByName()[net]
    rule = router.rules[net]
    added, stranded = 0, []
    fanouts = [c for c in conns if c.kind == "gnd" and c.net == net and not c.routed]
    for layer in (F, B):
        anchors = [it.geom for it in router.sp.items.values()
                   if it.net == nid and (layer in it.layers)
                   and (it.kind in ("track", "via") or (it.kind == "pad" and not it.smd))]
        for piece in floatingIslands(board, layer, net, anchors):
            room = piece.buffer(-(rule.viaDia / 2 + 0.05))
            pt = None
            if not room.is_empty:
                minx, miny, maxx, maxy = room.bounds
                grid = [(x, y) for x in np.arange(minx, maxx, 0.3) for y in np.arange(miny, maxy, 0.3)]
                pts = [(room.representative_point().x, room.representative_point().y)]
                pts += [p for p in grid if room.contains(Point(p))][:40]
                pt = router.placePlaneVia(pts, net)
            if pt is not None:
                via = router.viaItem(net, pt, rule, 0)
                writeItem(board, netInfo, via)
                added += 1
            else:
                stranded += [c for c in fanouts if c not in stranded and layer in c.a.layers
                             and piece.intersects(c.a.poly)]
    return added, stranded


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
        for it in router.sp.query(range(4), (x - 0.05, y - 0.05, x + 0.05, y + 0.05)):
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
        negotiate(router, apart, log, maxRounds=10, deadline=deadline)
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
    after = sum(router.routeCost(rid)[1] for rid, r in router.routes.items() if r.conn.kind in ("sig", "pwr"))
    log(f"via cleanup: {before} -> {after} signal/power vias")
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
    args = ap.parse_args()
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
    log(f"model: {len(model.pads)} pads, {len(model.nets)} nets, {len(model.keepouts)} keepouts")
    log("planes: " + ", ".join(f"{n} ({'whole board' if f is None else f'{f.area:.0f} mm2'})"
                               for n, f in model.planes.items())
        + ("" if boardZones else " (none on the board: ground on both inner layers)"))
    knobs = Knobs()
    for kv in args.knob:
        key, value = kv.split("=", 1)
        if not hasattr(knobs, key):
            raise SystemExit(f"unknown knob {key!r}")
        cur = getattr(knobs, key)
        setattr(knobs, key, (value.lower() in ("1", "true", "yes")) if isinstance(cur, bool) else type(cur)(value))
        log(f"knob {key} = {getattr(knobs, key)}")
    router = Router(model, knobs, log)
    log(f"space: {router.sp.W}x{router.sp.H} cells @ {router.k.grid} mm")
    flows = []
    if args.flows:
        with open(args.flows) as f:
            flows = json.load(f)["flows"]
    router.flowNets = {f["net"] for f in flows}
    conns, block, bundles = plan(model, router, log, flows)
    for c in sorted((c for c in conns if c.amps > 0), key=lambda c: -c.amps)[:6]:
        log(f"  current: {c.net} {c.a.ref}->{c.b.ref} {c.amps:.2f} A -> {router.ruleForConn(c).width:.2f} mm")
    # negotiation may use most of the budget; cleanup and planes need the rest
    unrouted = routeAll(router, conns, bundles, block, log,
                        deadline=t0 + args.minutes * 60 * 0.75)
    audit(router, log, "negotiation")
    cleanup(router, log, deadline=deadline)
    reconnect(router, [c for c in conns if not (c.kind == "gnd" and c.net == "GND")], log, deadline)
    skewRules = readSkewRules(os.path.join(srcDir, name + ".kicad_dru"))

    def skewBudget(net):
        base = net[:-1]  # KiCad's diff-pair base name: the net without its P/N/+/- suffix
        for pat, mm in skewRules:
            if fnmatch.fnmatchcase(base, pat) or fnmatch.fnmatchcase(net, pat):
                return mm, True
        return router.k.pairSkewMm, False
    router.matchPairs(pairPartners(set(model.nets)), skewBudget, log)
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
    powerNets = {n for n in model.netClass if router.powerClass(n)} | router.flowNets
    log(f"power copper: {router.fatten(powerNets)} tracks widened")
    audit(router, log, "power copper")
    if "GND" in router.planeNets:
        log(f"return vias: {router.returnVias()} GND vias beside signal vias")
        log(f"stitching: {router.stitch()} GND vias")
    nT, nV, nIsl, islandPads = finishBoard(board, router, model, boardZones, conns)
    log(f"pour islands: {nIsl} GND vias added so islands stay in the pour")
    board.Save(outBoard)
    log(f"wrote {outBoard}: {nT} tracks, {nV} vias")
    drc = runDrc(outBoard, os.path.join(args.out, "drc.json"))
    for _ in range(2):
        # GND pads whose piece of pour ended up cut off: fan them out to the plane after
        # all, pushing other routes aside, and write the board again
        stranded = strandedGnd(drc, conns) + [c for c in islandPads if not c.routed]
        if not stranded:
            break
        negotiate(router, list(stranded), log, maxRounds=10, winPads=(2.0, 6.0, 15.0))
        fixed = sum(1 for c in stranded if router.connected(c))
        log(f"stranded GND pads: {fixed}/{len(stranded)} fanned out to the plane")
        if not fixed:
            break
        nT, nV, nIsl, islandPads = finishBoard(board, router, model, boardZones, conns)
        board.Save(outBoard)
        drc = runDrc(outBoard, os.path.join(args.out, "drc.json"))
    for _ in range(3):  # a pad switched to solid can reveal the next one
        solid = fixStarvedThermals(board, drc)
        if not solid:
            break
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        board.Save(outBoard)
        log(f"thermals: {solid} GND pads switched to solid pour connection")
        drc = runDrc(outBoard, os.path.join(args.out, "drc.json"))
    viol = Counter(v["type"] for v in drc["violations"])
    log(f"DRC: {sum(viol.values())} violations {dict(viol)}, {len(drc['unconnected_items'])} unconnected")
    summary = dict(minutes=(time.perf_counter() - t0) / 60, tracks=nT, vias=nV,
                   violations=dict(viol), unconnected=len(drc["unconnected_items"]),
                   unroutedConns=len(unrouted))
    with open(os.path.join(args.out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=1)


if __name__ == "__main__":
    main()
