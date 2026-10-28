# Feedback: the Ambire signer's joystick nets (2026-10-22)

From the board's owner, after looking at the routed board: **"some of the joystick
nets have horrible twists and turns."** Those nets were ripped and left to be routed
by hand.

## Where

- Repo AmbireTech/secure-signer, `board/wallet.kicad_pcb`.
  - Commit `87ec893` has the routes as the router drew them.
  - Commit `8beaf8d` has them ripped.
- The board is 4 layers, 48 × 87 mm, with parts on B.Cu.
- The joystick (ALPS SKRHAAE010, `SW_JOY`) is on F.Cu at (24.0, 80.8). Its four
  direction pins each go to one resistor on B.Cu (`R_JA`–`R_JD`), placed under or
  next to it.
- Runs, in this order, each with `--keep-copper --knob fanoutParts=none`:
  1. USB pair + GND;
  2. power + GND/V3V3;
  3. signals + GND/V3V3;
  4. leftovers with `--drop-stale`.

## Numbers

"Shortest" is the minimum spanning tree over the net's pad centres.

| Net | Pads | Shortest | Copper | Ratio | Vias | Bends |
|---|---|---|---|---|---|---|
| JOY_C | 2 | 0.5 mm | 5.7 mm | 11× | 1 | 7 |
| JOY_A | 2 | 10.9 mm | 42.8 mm | 3.9× | 3 | 10 |
| JOY_B | 2 | 4.3 mm | 5.5 mm | 1.3× | 1 | 5 |
| JOY_CEN | 2 | 22.1 mm | 26.8 mm | 1.2× | 1 | 9 |
| JOY_DIR | 6 | 54.4 mm | 87.4 mm | 1.6× | 7 | 42 |

JOY_C's pads are 0.5 mm apart in plan but on opposite faces. One via beside the pad
is the whole route. The router drew 5.7 mm and 7 bends instead.

## Ripped for the same reason

- **SW_USB:** the buck's switch node, 2 pads, went through 2 vias at 1.7×.
  A switch node should stay on one layer, as short as possible.
- **USB_DP / USB_DN:** 4 vias on one line of the pair and 2 on the other.
- **RADIO_TX:** 4 vias on an 8.8 mm, 2-pad net.
- **CHG_TS:** 11 vias at 2.6× overall.

## The router's blind spot here

- After the cell clip moved over the boost converter, BAT (clip pad → LTC3539 VIN
  pin) and PWR_EN (→ the boost's enable pin) failed in negotiation:
  - `BAT:BT_P.1->U2.4`
  - `BAT:U2.4->C_IN_BOOST.1`
  - `PWR_EN:R_EN.1->U2.5`
- **Vias in same-net pads:** the only via site for BAT is inside the 9 × 5 mm
  BAT clip pad on the other face. The router never places a via inside a pad,
  even one of the via's own net.
- **Edge lane:** PWR_EN needs a lane between the clip pad and the board edge.
  That lane is 0.6 mm wide.

## What would have caught it

- Score each finished connection against its pads' straight-line distance and its
  layer change, with the owner's acceptance test: near-shortest, no detour, hairpin
  or extra via.
- Rip and retry the worst connections, or report them as left to the user, instead
  of keeping them.
- The tidy pass's "worst detours" step did not catch a 11× two-pad net.
