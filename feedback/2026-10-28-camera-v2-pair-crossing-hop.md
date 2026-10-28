# Feedback: camera v2, a pair that must cross another hops twice (2026-10-28)

`feedback/2026-10-28-camera-v2/camera.kicad_pcb` is the placed, unrouted board (its `.kicad_pro`, `.kicad_dru`,
`camera.flows.json` and lands beside it); `routed-seed8.kicad_pcb` is the router's best result. The board lives in
the main repo at `hardware/camera-v2/pcb/board/`.

    $KPY run.py feedback/2026-10-28-camera-v2/camera.kicad_pcb --out /tmp/cam --flows feedback/2026-10-28-camera-v2/camera.flows.json --seed 8

## What happens

Seeds 0-9: every connection routed, DRC clean except **one `skew_out_of_range` per run**, 2.7-4.6 mm, always on
CSI_D0 or CSI_D1. The track lengths of the two nets agree within 0.1 mm; the skew is **vias**: one net of the pair
has 3 vias, its partner 1, and KiCad counts the board's thickness per via (`use_height_for_length_calcs`).
`matchPairs` measures `netLength` without vias, so it reports the pair matched (`skew 0.59 -> 0.25`) while DRC says
3 mm. Seed 8, CSI_D0_P leaves J2 on B, goes up to F at (1.38, -8.1), back down to B at (1.38, -4.9) and up again at (-0.38, -3.15) (lens-axis frame,
KiCad = +100,+100); CSI_D0_N has its one via at (0.33, -8.28). All three pairs end up 17-20 % coupled.

## Why it is a crossing

The sensor receptacle J2 is on B.Cu; its signal row (y -9.375) has, west to east, D1 (pins 20/21, x 0.1/0.5), D0
(23/24, x 1.3/1.7), CLK (26/27, x 2.5/2.9). The chokes L2/L3/L4 on F.Cu, fixed by the HDMI pin map, sit west to east
D0 (x -0.79), D1 (0.71), CLK (2.21). So D0 and D1 swap order once between J2 and the chokes. Every pair needs one
via anyway (B to F). A person routes it with no extra via: D1 stays on B down to just before L3 and vias up there; D0
vias up at J2 and crosses over D1 on F (or the mirror). The router instead takes both pairs to F within 1-2 mm of J2
and then hops one wire of D0 back to B and up again to get past D1.

## Asks

1. Pair layer planning: when two pairs must cross and both change layer anyway, choose where each changes layer so
   the crossing costs no via (one pair keeps the source layer past the crossing).
2. A pair's two nets should never get different via counts; if they do, treat it as uncoupled and re-route the pair
   together.
3. `matchPairs` (and the "skew a -> b" log) should count via length the way KiCad does when the board's
   `use_height_for_length_calcs` is on, so the router's "matched" and DRC agree.

## Addendum: r3 (same day; board at main-repo commit 6f153161, hardware/camera-v2/pcb/board/)

The pairs are now 0.14 mm / 0.41 mm gap (the chokes' pin pitch). Seeds 0-12, default knobs (`viaMm` 5 and 10
gave identical results):

- Lane 0 always lands one of two ways: the clean crossing (one via per wire) with **0.99 mm unmatched**, where the
  log says `length match CSI_D0_N/CSI_D0_P: skew 0.99 -> 0.99 mm (budget 0.50)`, i.e. `lengthen` found no room
  (seeds 3, 5, 8, 9); or the double hop of D0_P (4.4 mm by DRC, counted with via barrels). The board now carries
  a lane-0 rule at 1.0 mm so it can ship; the ask stands: meander room for a crossing pair (or plan the crossing
  so both wires come out equal).
- **3.3 V necked below its pad**: seed 5, V3V3 leaves J1 pin 15 (pad 0.30 x 2.00) on 0.15 mm for ~0.5 mm past the
  pad end, between pins 14 and 16 whose tracks also leave straight out. A 0.30 neck fits there. `neckMm` should
  floor at the pad's own width, not the class's signal width.
- **Vias against pads**: with no rule, escape vias landed 0.017-0.027 mm from same-net pad edges (C3/C4/R1/R2);
  with the board's `physical_clearance` 0.05 mm via-to-pad rule they come out 0.05-0.09 mm, and with 0.10 mm
  DRC fails (the router does not read the rule). Read `physical_clearance` via-to-pad from the .kicad_dru.
- **Pairs over plane clearance**: some CSI tracks pass over the inner ground's clearance holes round a
  neighbouring pair's via (lane 1 P over lane 0 P's vias in r2). Price plane holes under pair tracks.
- **No gap setting for a signal's escape via** (r3 final, main-repo commit 4ce0971e): a 0.1 mm via-to-pad rule at
  the two fine-pitch connectors (`B.memberOfFootprint('J1') || ...('J2')`) left 5 violations at 0.06 mm in every
  seed (J2 pins 1, 7, 21; J1 pins 16, 19). `viaGapMm` applies to fanouts only. The board shipped at 0.05 mm.
- Seed 8 of the final placement matched every pair with one via per wire (lane 0 at 0.61 mm by KiCad), so the
  crossing can come out clean; seeds 0-4 still produced the double hop (1.3 mm) or V2V8/V1V1 detours of 60-70 mm.
