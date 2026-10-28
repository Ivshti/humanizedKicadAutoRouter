# Feedback: the owner's review of the c6d9adf Nano run (2026-10-25)

The owner's verdict on `hardware/nano/autoroute/nano.kicad_pcb` from `c6d9adf`, compared
with the hand board `18b487c3`: **"otherwise doesn't look half bad"**. Fixed by c6d9adf: no
0.15 mm necks or 2 mm blobs on P12_IN / V12N, no GND stub or via on the RJ45's pin 7, GND
vias 551 → 403 (hand: 314), DRC 0 / 0.

The owner's raw edits are kept next to this file:
`2026-10-25-nano-owner-cleanup-18b487c3.diff.txt` (the hand board's cleanup) and
`2026-10-25-nano-owner-fixes-oct14-autoroute.diff.txt` (fixes to the Oct 14 autoroute).

## Still open, as the owner listed it

1. **Vias, more than twice the hand count: 168 vs 75** (Oct 14: 181). Worst nets, by
   hand → router: GNSS_RX 1→9, BAT 1→7, I2C_SDA 2→8, GNSS_TX 1→7, MODEM_RX 0→6,
   USB_DP 0→5, MODEM_RESET_N / MODEM_PWRKEY / MODEM_RXD_1V8 0→4, CANL / TFT_BL / SYS /
   MODEM_STATUS 2→6, USB CC1/CC2 and the LED links Net-D1..D5-1 0→2. The owner's own
   cleanup shows the preference: SIM_IO_M taken on an 18 mm longer top-layer loop to
   drop two vias. *Price a via pair above a several-mm same-layer detour.*
2. **The V12N via went back to where the owner had moved it from.** It sits at
   (81.08, 57.65), between the RJ45's pin field and the P12_IN trunk at x 80.25. The
   owner had put it clear, at (84.08, 58.97). *Keep vias of other power nets out of a
   connector's pin field and off the side of an input trunk.*
3. **The diode-to-TVS link on P12_IN: 4 mm at 0.2 mm** (hand: 0.8 mm). The flow sizes the
   path connector → diode → regulator; the SMBJ24A's clamp path got signal width. *A TVS
   or other protection part on a power net gets the trunk width.*
4. **flowcheck flagged 5 squeezed paths:** the modem buck-boost's and the 3.3 V buck's
   switch nodes at 0.3 mm on 0.8 / 0.5 mm trunks, and VBUS at 0.4 mm on 0.5 mm.
   *Switch nodes into inductors are the worst place to neck; reroute or widen until the
   check passes.*
5. **Long detours:** ESP_EN 128 mm vs 63 by hand, MODEM_RESET_N 52 vs 28, BAT 117 vs 88.
6. **The 12 V input at 0.5 mm**, where the owner uses 0.8. *Consider a floor for nets off
   the car connector, where transients matter more than the IPC average.*
7. (Screenshots) **V-shaped right-angle corners:** BL_EN leaves the 100 k pull-down
   diagonally, steps a 0.19 mm flat and turns back diagonally around the resistor's GND
   pad. SW12 runs diagonally out of the 12 V buck's pin and diagonally back to the
   bootstrap cap. A person cuts such a corner into two 45° bends.

The owner also asked for a comparison with the malakaRouter3 pass
(`hardware/nano/autoroute/malakaRouter3/`, Oct 12), which "LOOKS better".

## What was found

- **4 was mostly the check's own mistake.** It counted a neck into a pin as squeezed when
  the pin's *long* side was wider than the trunk. The TPS63802's pins are 0.9 × 0.3 mm,
  so the 0.3 mm neck (0.45 mm past the pin's end) is how a person lands there too; the
  hand board runs 0.35 mm for 1.3 mm. VBUS's 0.4 mm is a neck into the TPS2121's
  0.4 mm pin. Judged by the pin's narrow side, none of the five is a squeeze.
- **5:** cleanup re-routed every detour against the finished board, but still with the
  bundle's corridor plan in force (3× the cost outside the corridor). ESP_EN ran 118 mm
  on the bottom inside its corridor for a 53 mm straight-line distance.
- **1, the hops that stay:** they cross power trunks (SYS_SW, VBUS, V3V3). Unhop only
  moved routes with nothing attached, and a trunk has taps.
- **2:** with P12_IN at 0.8 mm the via still landed 0.45 mm beside it, under the diode.
  Nothing kept another net's via off the side of a trunk.
- **3 with the first fix in place:** the TVS tap then grew to 2 mm. It runs pad to pad
  (diode to TVS), so no copper capped it at the trunk's width.
- **malakaRouter3:** worse on everything measured. 186 signal vias; 0.15 mm necks on the
  2.6 A BAT, SYS and modem switch-node paths and on P12_IN; 174 right-angle bends (hand 8,
  this router 12–14); 4.4 bends per 10 mm (2.6); 225 jogs (46); CAN 4 % coupled with
  10 mm skew. It looks cleaner because 80 % of its copper is on the top layer (2254 mm
  top, 585 bottom), so the bottom reads as an unbroken ground pour. The hand board is the
  other way round (1298 top, 1702 bottom).

## What changed in the router

- **Protection parts (3):** a two-pin part between a flow net and ground that is not a
  capacitor, resistor or inductor (a TVS, an ESD diode) gets its tap sized like the
  trunk.
- **Taps never wider than the trunk:** a tap is capped by the widest copper of the net
  at its pads, not only by the copper it lands on.
- **Width floor (6):** a flow may say `"minWidthMm"`; its trunk is never narrower.
  `boards/nano.flows.json` sets 0.8 mm on P12_IN and V12N.
- **Input trunks (2):** a flow whose source is a connector is an input trunk. Another
  power net's via within 1 mm of it costs what a via out of place costs (5×), in the
  search and in via tidying. The via-tidying step had put the V12N via back in line
  with the diode pad, beside the trunk. A hard keep-out (any net's via, or power vias
  only) left 10–44 connections unrouted: it boxed in V12N itself and the power around
  the USB-C charger, where VBUS runs. Laying input trunks first did not help either.
- **Unhop (1):** a crossed route comes up together with the taps hanging off it (up to
  ten routes); everything is put back if the result is not better.
- **Tidy free of the plan (5):** cleanup, via cleanup and unhop route without the
  corridor plan.
- **Corners (7):** after via tidying, every right angle (square, a V of two diagonals,
  or two diagonals with a flat under 0.4 mm) is cut into two 45° bends, as deep into
  the corner as clearance allows.
- **flowcheck (4):** a neck is judged by the pin's narrow side, and by how far it
  reaches past the pin (it may be drawn in several pieces).
- **Via price (1), tried and not adopted:** `viaMm` 10 instead of 5 gave 155 signal vias
  against 146 for the same code at 5, and the CAN pair fell apart (3 % coupled, 29 mm
  skew).

## Results (scratch runs on 18b487c3, seed 0; final code = R7)

| | hand | c6d9adf | R7 |
|---|---|---|---|
| signal vias | 75 | 168 | 163 |
| on one-side nets | 40 | 97 | 99 |
| GND vias | 314 | 403 | 394 |
| DRC | 0 / 0 | 0 / 0 | 0 / 0 |
| current paths squeezed | 5 | 2 (against the 0.8 mm floor) | 0 |
| right-angle bends | 8 | 12 | 10 |
| ESP_EN mm | 63 | 128 | 67 |
| MODEM_RESET_N mm | 28 | 52 | 53 |
| BAT mm | 88 | 117 | 108 |

Current paths in R7: P12_IN 0.8 mm end to end, its TVS tap 0.8 mm, V12N 0.8 mm with its via
at (83.05, 55.95), off the diode's far end and clear of the trunk. SYS 2.0 mm, BAT
0.8 mm, VMDM 0.55 mm; each pin neck at most 0.5 mm past its pin. BL_EN and SW12 (the
screenshots): the V's point became a straight beside the resistor's GND pad. SW12 keeps
a 0.12 mm step that the 12 V buck's pin clearance forces.

**Vias are not fixed.** Over four runs of nearly the same code the count went 146, 152,
155 and 163. R5 and R7 differ only after routing, yet their negotiation ran 145 and 81
rounds, so a run is not exactly repeatable. Unhop undoes 3–8 hops a run. What is left:
- short top-side signals hop under a crossing (MODEM_RX 0 → 6, MODEM_RESET_N and
  MODEM_PWRKEY 0 → 4);
- bundles planned onto the bottom layer between two top-side blocks still take two
  vias per member.

Next: plan corridors on the parts' own side unless the top is full, and let unhop take
a whole bundle at once.

MODEM_RESET_N's detour (53 vs 28 mm) and USB's skew (11 mm in R7, 0.2–0.8 mm in other
runs: USB_DP branches to the ESD part) remain open.
