# Feedback: re-routing hops locally mostly rebuilt them (2026-10-25)

- Repo: AmbireTech/secure-signer, `board/wallet.kicad_pcb`, commit `7930030`.
- Router: `under-part-jumps` at `3c165b3`, with `viaPadGapMm=0.2`, `hopMaxMm=6`, and
  `--knob jumps=0` via the board repo's `routeGauge.py`.

## What was tried

For each of 8 layer hops on the owner's board, the hop was ripped locally: both vias,
the run between them, and up to 4 mm of the net on the other layer on each side. A
track longer than that was cut at 4 mm, so the router had to join its cut end. Then
the router filled the gaps (8 minutes; it took about 3).

## Result per net

| Net | Outcome |
|---|---|
| VBUS_SENSE | hop gone; route goes round a cap; the net's detour ratio is now 1.9 |
| BTN | the same hop, both vias moved 0.8–1.5 mm (one now clears R_EN's pad) |
| AIRGAP | the identical hop: same two vias, same positions |
| FB_BOOST | the same hop, one via 0.16 mm further from U2's pin (that part is good) |
| CHG_TS, CHG_ISET | new hops a few mm away, and 5 vias crowding Q_CHG; rejected |
| CELL_N_SENSE (north) | hop gone, but the replacement is an 18-segment wiggle round the MCU fanout; rejected |
| CELL_N_SENSE (south) | 3 vias became 2 plus a detour west round Q_REV and a dangling end; rejected |

A second run that ripped only the 5 accepted hops routed BTN and CELL_N_SENSE
differently again, so results depend on what else is ripped.

## Asked of the router

1. **Count a hop as a cost while routing, not only in tidy.** `hopMaxMm` only re-lays
   a hop afterwards "when that fits". Here it never fitted, and negotiation went
   back to the same two-via crossing. Negotiation should price the via pair against
   ripping the crossed net, or against a short re-order.
2. **Joining a cut track end** produced the wiggles: the CELL_N_SENSE route ran 15 mm
   to reach the cut end instead of joining the piece where it was nearest. If the
   piece is fixed copper, any point on it is a valid target.
3. **Results depend on the rip set.** Ripping 8 hops vs 5 gave different BTN and
   CELL_N_SENSE routes. Not a bug as such, but it makes "rip one, check, keep" unreliable.

## Not the router's

The board repo's `routeDefects.py` missed a hop when the via that branches came
first in the walk (fixed in `7930030`). Earlier "hop gone" counts for BTN and
AIRGAP were wrong because of it.
