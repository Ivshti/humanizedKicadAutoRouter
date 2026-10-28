# Feedback: how the owner fixed what the router and the quality pass left (2026-10-25)

- Repo: AmbireTech/secure-signer.
- Before: `7930030`, after the automated quality pass.
- After: `a547dda`, the owner's hand pass.
- Measured by diffing the two boards' copper per net, plus `routeDefects.py`.

## What the owner did, in numbers

| Net / area | Vias | Length | How |
|---|---|---|---|
| CHG_TS (charger cluster) | 5 → 2 | 55.2 → 54.3 mm | left the cluster on the parts' layer: a loop round the west side of the 2N7002DW, no hop through it |
| CHG_ISET | 2 → 2 | 15.2 → 9.9 mm | re-laid straight; one hop remains |
| CELL_N_SENSE | 6 → 4 | 53.2 → 50.4 mm | front trunk carried to the cap and one via dropped right at C_CNS's pad, 0.05 mm from it |
| BTN | 2 → 1 | 52.3 → 54.8 mm | a hop traded for 2.5 mm on the back |
| MCU camera fanout (CAM_D0–D3, HREF, PCLK) | same | same | vias slid out from the pins, to 0.15+ mm from other nets' pins |
| C_U9B (0402) | — | — | turned 90° so RADIO_TXD_MCU jumps straight between its pads |
| Pads the pour missed | — | — | each tied by a short track to a GND via beside it, neighbouring pads chained; never a track into fill. Unconnected 4 → 1 |

## Lessons for the router

1. **A via pays for more than a few mm of track in a cluster.** The owner went round
   the charger FET on the parts' layer instead of crossing it with via pairs: 3 fewer
   vias and a shorter net. Inside a block, price a via pair above a 3–5 mm detour on
   the same layer.
2. **Drop the layer change at the destination.** Carry a signal on its trunk layer to
   the part and change layer once, right at the pad, instead of hopping on the way.
3. **Calibrate `viaPadGapMm` by net.** The owner's own new vias sit 0.05 mm from their
   own net's pad (a fanout via) and 0.15–0.19 mm from other nets' pads. Suggestion:
   a via must not overlap a pad of its own net and must keep 0.15 mm from other nets'
   pads. Today's single 0.2 mm, own net included, is stricter than the owner in one
   direction and looser in the other.
4. **Starved ground pads: a short tie to a via, by design.** This is the owner's habit:
   a tie of at most about 1.5 mm to the nearest GND via, a new via if needed, and two
   neighbouring pads may share one. It refines `gndRepair`: tie to a via, never end a
   track in fill, and never tie a pad the router's own unjudged routes cut off.
5. **A jump must fit the pad gap.** C_U9B's pads are 0.40 mm apart. A 0.15 mm track
   with 0.15 mm clearance on each side needs 0.45, and the owner's jump now breaks
   clearance (0.10 mm). A jump should check the gap against width plus two clearances,
   and neck to the minimum width (0.10 mm here) when that makes it fit.
6. **Turning a two-pad part to open a straight jump** is a placement move the owner
   uses; the router's placement-aware passes could try it.
