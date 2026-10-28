# Feedback: the Nano bar-display board was finished by hand and by KRT, not by us (2026-10-26)

From the board's owner: **"i don't like that the router we own is being superseded by ai hand
routing; unacceptable. you need to tell the router to fix its shit and identify why it cannot
route."** And on the 3V3 that was laid around the router: **"truly atrocious"**.

- Board: `hardware/nano/board/nano.kicad_pcb` after the 3.9" RGB display swap (malaka-chrono
  `909334c7` placement + partial route, owner hand routing `f0856364`, finishing pass `c692d340`).
- In `c692d340` the last 18 airwires were closed by KiCadRoutingTools' `route.py` and by Claude
  laying copper with pcbnew scripts, because humanRouter's output was rejected. That is the
  failure this report is about.
- Repro boards next to this file, in `2026-10-26-nano-bar39/`:
  - `case-final-opens/`: `c692d340` (six opens: SYS, CHG_CE, CHG_STAT, LED_DIN, V3V3 at
    U10 pin 3, and the owner's unfinished V3V3 stub at U6).
  - `case-before-today/`: the board before the finishing pass (24 opens on 13 nets).
  - `runA-route.log`: the run on `case-final-opens` quoted below.
- Runs:
  `run.py <board> --out D --keep-copper --nets "SYS,CHG_CE,CHG_STAT,LED_DIN,V3V3" --flows boards/nano.flows.json --knob fanoutParts=none --minutes 10`
  (run A), plus single-net runs of the same with `--minutes 2-3 --tidy off`.

## What the router did on these nets

Earlier today the same open nets (`case-before-today`, 12 nets, 4 seeds, 8 min each) were
judged per net against the owner's rule (length ≤ 1.35x the gap, ≤ 1 extra via). The other
router, KRT, got routes the router did not:

| net | humanRouter, 4 seeds | KRT got |
|---|---|---|
| ESP_EN | 0.6 mm added for a 42.6 mm gap: not routed | 45 mm, 3 vias |
| IMU_INT1 | 2.1x the gap | 1.3x, 2 vias |
| MODEM_TX | 2.5x, or 3 vias on a top-to-top link | 1 via |
| V3V3 | 2.0x, 8-12 vias | 1.5x, 9 vias over 9 islands |
| CHG_CE / CHG_STAT / LED_DIN | 4-8 vias each | (rejected too) |

KRT's numbers are not a target: it ignores the patch and runs the charger pair 1.1 mm past
the GNSS feed. They show the space exists.

## Why it cannot route: causes found, each with the run that shows it

### 1. Module bodies ban vias over far too much board (main cause of the long detours)

`bodyViaKeep` (router.py `loadStrategyRasters`, applied at 607) bans every via of every net
under every body ≥ 40 mm², on both faces. On this board that covers:
- ESP32 module `U4`: the whole courtyard, x -15..26 mm.
- The patch `ANT1`: 29 x 29 mm.
- The modem `U7`: 28 x 28 mm.
- The touch FPC `J_CTP`: on the bottom, directly under the expander `U16`.

The owner's own boards do not follow that rule:

| under | owner, before the swap | owner, `f0856364` |
|---|---|---|
| ESP32 module body | 26 signal vias | 30 signal vias |
| modem body | 2 | 2 |
| patch courtyard | 0 | 1 (none within 4 mm of the feed) |

The patch ban is right. The module ban is not. With it, every net to the ESP32's east pins has
to place its via in the 4 mm slot between the module body and the patch, where the display
bus's 20 vias already sit.

Proof, LED_DIN alone on `case-final-opens`:
- as is: **114.5 mm**, 2 vias, around the far end of the display bus;
- `--knob bodyMinAreaMm2=100000` (the patch ban stays): **41.8 mm**, 4 vias;
- the owner did it in 48 mm with 2 vias on the old board.

The main diagonals are not the cause: `againstCost=0` gives the identical 114.5 mm, and
`topDiagonal` flipped gives 121 mm.

**Asked:** under a module, vias of other nets are allowed outside the module's own pads and
exposed pad (with clearance). Keep the hard ban only for antennas (and their feed), and for
bodies the board's owner marks. A connector body on the far face (the FPC under U16) is not
a reason to ban vias either.

### 2. Power is never routed on the inner layer the rules allow

`layersFor()` (run.py:35) returns only F.Cu and B.Cu. This board's `.kicad_dru` allows SYS,
SYS_SW, BAT, V3V3, VMDM, VBUS, CHG_IN and V5_12 on In2. That is where the owner runs power;
design.md says so too. So:
- **V3V3** is laid as a pad-to-pad tree of 0.2 mm tracks on the outer layers. It crosses the
  display bus twice and hangs branches off a strap pull-up. The owner called the result
  atrocious.
- **V3V3 to U10 pin 3** can only reach the pin past U10's GND pin. The router's route there
  (run A) cut U10 pin 2 off the pour, exactly as the hand route did.

**Asked:** route power nets the `.kicad_dru` allows on In2 there, as a trunk with short drops
by via, priced so a drop is cheaper than an outer-layer detour. Keep the In2 GND pour joined
around them, as the owner does.

### 3. A zero-current tap of a power net is priced as power

SYS's link to the slide switch `SW1` holds the load switch's gate. It carries nothing (the
flow is U1 → Q_SW only). Because SYS is in the Power class, the tap still:
- gets the class's 0.8 mm vias;
- pays 4x per via (`viaMmFor`: `c.kind == "pwr"`).

Alone, it routes as **155 mm with 5 vias and 31 corners** for a ~63 mm gap; it starts by going
east to x = 52 mm. In run A it was one of the two nets left unrouted.

**Asked:** a tap with no current is a signal: signal via size, signal via price, signal
width (the class width as the floor is fine).

### 4. The last reconnect pass trades nets and the log names the wrong ones

In run A:
- LED_DIN was routed through every phase up to via tidying (probe at 505.8 s).
- At 607 s the reconnect pass ripped LED_DIN and CHG_STAT to fit SYS and CHG_CE.
- The log still says `2 unrouted: SYS, CHG_CE` (written at 457 s), and `summary.json` repeats
  it. KiCad's DRC says LED_DIN and CHG_STAT are the open ones.
- What reconnect laid instead is poor: CHG_CE 111.7 mm with 10 vias, SYS 155.8 mm with 5.

On top of that:
- Negotiation spent 430 of 607 s (48 rounds) on five nets.
- Pass 1 had routed 50 of 51 connections in 21 s, so negotiation ended worse than pass 1.

**Asked:** never rip a finished connection in reconnect unless the total gets better. Name
the open connections from the final state. If negotiation ends with more failures than
pass 1, keep pass 1's result.

### 5. Tidy edits copper outside `--nets`, under `--keep-copper`

Run A was asked for five nets. Tidy (on by default) removed:
- the owner's GNSS_nRESET leg (45.1→45.8, 15.5; plus 0.8 mm on B.Cu at 58.4, 38.6);
- a VBCKP piece (0.9 mm);
- the owner's ESP_EN joints at C_EN;
- I2C_SCL, MODEM_TX, USB_DP/DN and LCD_B4 crumbs;
- 11 existing V3V3 items.

The four-seed runs also gave back GNSS_nRESET, VBCKP and GND ("designer's copper removed") and
re-laid the USB pair (66 % coupled). All of these nets were outside the request. In
`c692d340` the crumbs had to be put back by hand.

**Asked:** under `--keep-copper`, tidy touches only the router's own copper (the sidecar),
and under `--nets`, only those nets.

### 6. "Pads the pour does not reach: none" while a pad is cut off

In run A the router reported no unreached pads, yet DRC splits GND: U10 pin 2 sits in a
1.5 mm² B.Cu pour piece with no via. Its V3V3 leg to pin 3 runs 0.17 mm off the pin column.
Round 4 counts SMD-only pour pieces as islands. That check either ran before the last change
or misses pieces this small.

**Asked:** run the island check on the board as written (after tidy). A route that strands a
GND pad is a failed route.

### 7. Long nets that must cross the patch

With the module bans off, CHG_CE still needs 79.5 mm and 10 vias for a 41.5 mm gap. Its path
is boxed in:
- it must cross the patch north to south;
- the owner's bottom-layer traffic under the patch runs east-west;
- the top layer and vias are banned there (correctly).

This is mostly placement: the expander `U16` sits north, the charger south. That belongs to
the placement session. The router should still report it, rather than lay 10 vias:
"CHG_CE/CHG_STAT cross the patch; no layer free" would have said where the problem was.

### 8. A crossing forced by pin order

LCD_DISP and LCD_nRESET left the expander on pins 11/12 in the opposite order to their
pull-downs at the display connector. Every route crossed, and the router looped LCD_DISP
around the expander, which boxed in two GND pads. Swapping the two expander pins (P6/P7 are
interchangeable GPIOs) removed the crossing.

**Asked (lower priority):** when two nets on interchangeable pins of one part must cross,
say so in the log, with the swap that would avoid it.

### 9. The easy route round the patch was never tried (owner: "fundamental lapse")

The owner routed CHG_CE and CHG_STAT by hand on the bottom layer, round the patch
(`wip/nano-owner-wip-2026-10-26`):
- east out of the expander at y ≈ 16;
- down the east edge at x ≈ 76;
- back west at y ≈ 53;
- 2 vias each.

On the board the router saw (`case-final-opens`) that corridor was empty, yet the router laid
CHG_CE as 100 mm with 8 vias:
- west under the expander;
- under the patch on the bottom;
- a top-layer hop at the patch's east edge (x 58.5), back west, and another hop at its west
  edge (x 28.8).

`corridorOut=1` gives the identical route, so the corridor penalty is not why. Causes:
- **Its own via ban under J_CTP.** The touch connector sits on the bottom directly under U16,
  and its body bans every via within reach of the expander's east pins. The route's first via
  lands at (43.65, 19.2), past U16's south end.
- **Two east-west bottom barriers under the patch**, with top copper and vias both banned
  between them. Every north-south crossing then costs a hop at the patch's edge, and the
  search pays it twice instead of leaving the patch altogether.
- **The top-side exit east of U16 was walled** by MODEM_RESET, Q_RST, MODEM_RX and
  GNSS_nRESET at x 49-51. That was ours (earlier passes), and the owner cleared it.

**Asked:** when a connection's straight line crosses an antenna body where both layers are
closed, also cost the path round the body's nearest edge and take it when it is cheaper.
Corridor planning should consider the board-edge band as a corridor.

## Not router defects (for the record)

- The hand-laid I2C_SCL touch link, the upright C_EXP and the solid connection on R_BLSET's
  GND pad were placement and part-level fixes.
- The 3V3 "loops" and jogs in `c692d340` came from KRT and from the hand smoothing, not from
  this router.

## Order of work, by what it would have saved on this board

1. Body via ban (1): LED_DIN 114 → 42 mm; every ESP32 east-pin net.
2. In2 power (2): V3V3 and SYS done the owner's way.
3. Zero-current power taps (3).
4. Reconnect trading nets and stale names (4).
5. Tidy scope (5).
6. Island check after tidy (6).
