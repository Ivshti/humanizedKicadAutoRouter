"""Autoroute a placed KiCad board from scratch, the way a person would.

    kicad-python tools/humanRouter/run.py <board.kicad_pcb> --out <dir>

Writes <dir>/<name>.kicad_pcb (plus the project, rules and footprint library it needs),
fills the planes, runs DRC and prints a summary. The input board is never modified.
"""
import argparse
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

from boardModel import B, F, IN1, IN2, loadModel, ripCopper
from emit import addPlanes, runDrc, writeRoutes
from corridors import Corridors, splitBundles
from router import Knobs, Router
from strategy import phaseOrder, plan


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


def planCorridors(router, bundles, block, log):
    k = router.k
    router.corridors = Corridors(router, k.corridorTileMm)
    groups = splitBundles(bundles, block, k.joinMm, k.maxGroup)
    router.corridors.plan(groups, log)


def routeAll(router, conns, bundles, block, log, maxRounds=40, deadline=None):
    k = router.k
    order = phaseOrder(conns)
    t0 = time.perf_counter()
    failed = []
    for i, c in enumerate(order):
        if c.bundle and c.kind == "sig" and router.corridors is None:
            # block wiring, GND and power are down: plan the channels for what is left
            planCorridors(router, bundles, block, log)
        st, _ = router.route(c, layersFor(c), winPadsFor(c), viaMm=viaMmFor(c, k))
        if st != "ok":
            failed.append(c)
    log(f"pass 1: {len(order) - len(failed)}/{len(order)} routed in {time.perf_counter() - t0:.1f}s, "
        f"{len(failed)} failed: {Counter(c.kind for c in failed)}")
    # negotiation: a failed connection may cross other routes at a price; those are ripped
    # and go back on the queue (the push-and-shove step, at route granularity)
    # GND pads the fanout could not reach are left to the pour; they are judged by DRC
    queue = [c for c in failed if c.kind != "gnd"]
    rounds = 0
    attempts = Counter()
    while queue and rounds < maxRounds * max(1, len(failed)):
        if deadline and time.perf_counter() > deadline:
            log("negotiation: deadline reached")
            break
        rounds += 1
        c = queue.pop(0)
        if c.routed and router.connected(c):
            continue
        st, _ = router.route(c, layersFor(c), winPadsFor(c), viaMm=viaMmFor(c, k))
        if st == "ok":
            continue
        attempts[c.cid] += 1
        if attempts[c.cid] > maxRounds:
            continue
        router.k.negotiatePenalty = 4.0 * (1.3 ** min(attempts[c.cid], 12))
        st, payload = router.route(c, layersFor(c), winPadsFor(c), negotiate=True,
                                   viaMm=viaMmFor(c, k))
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
            for r in ripped:
                if r is not c:
                    queue.append(r)
        elif st != "ok":
            queue.append(c)
    unrouted = [c for c in conns if not router.connected(c)]
    log(f"negotiation: {rounds} rounds, {len(unrouted)} unrouted: "
        + ", ".join(f"{c.net}:{c.a.ref}.{c.a.num}->{(c.b.ref + '.' + c.b.num) if c.b else 'plane'}"
                    for c in unrouted[:30]))
    return unrouted


def fixStarvedThermals(board, drc):
    """A GND pad whose thermal spokes are blocked gets a solid pour connection instead."""
    n = 0
    for v in drc["violations"]:
        if v["type"] != "starved_thermal":
            continue
        for it in v["items"]:
            m = re.match(r"Pad (\S+) \[.*\] of (\S+) on", it["description"])
            if not m:
                continue
            fp = board.FindFootprintByReference(m.group(2))
            for pad in fp.Pads():
                if pad.GetNumber() == m.group(1):
                    pad.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
                    n += 1
    return n


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("board")
    ap.add_argument("--out", required=True)
    ap.add_argument("--minutes", type=float, default=25.0)
    ap.add_argument("--flows", help="JSON {flows: [{net, source, sinks, amps}]} sizing power copper")
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
    ripCopper(board)
    model = loadModel(board, os.path.join(srcDir, name + ".kicad_pro"))
    log(f"model: {len(model.pads)} pads, {len(model.nets)} nets, {len(model.keepouts)} keepouts")
    router = Router(model, Knobs(), log)
    log(f"space: {router.sp.W}x{router.sp.H} cells @ {router.k.grid} mm")
    flows = []
    if args.flows:
        with open(args.flows) as f:
            flows = json.load(f)["flows"]
    router.flowNets = {f["net"] for f in flows}
    conns, block, bundles = plan(model, router, log, flows)
    for c in sorted((c for c in conns if c.amps > 0), key=lambda c: -c.amps)[:6]:
        log(f"  current: {c.net} {c.a.ref}->{c.b.ref} {c.amps:.2f} A -> {router.ruleForConn(c).width:.2f} mm")
    unrouted = routeAll(router, conns, bundles, block, log, deadline=deadline)
    cleanup(router, log, deadline=deadline)
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
    powerNets = {n for n, cls in model.netClass.items() if cls in router.k.powerWidth} | router.flowNets
    log(f"power copper: {router.fatten(powerNets)} tracks widened")
    nStitch = router.stitch()
    log(f"stitching: {nStitch} GND vias")
    nT, nV = writeRoutes(board, router)
    addPlanes(board, model.outline, [(F, "GND", True), (IN1, "GND", True), (IN2, "GND", True),
                                     (B, "GND", True)])
    board.Save(outBoard)
    log(f"wrote {outBoard}: {nT} tracks, {nV} vias")
    drc = runDrc(outBoard, os.path.join(args.out, "drc.json"))
    solid = fixStarvedThermals(board, drc)
    if solid:
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
