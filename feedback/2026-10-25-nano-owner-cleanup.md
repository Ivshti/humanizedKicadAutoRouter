# Feedback: the Nano against the owner's hand board, and the owner's fixes to an autoroute (2026-10-25)

Two inputs from the owner:

- **Hand board** `18b487c3` (`hardware/nano/board/nano.kicad_pcb`), the owner's cleanup of
  the hand-routed Nano. SIM_IO_M re-laid on the top layer with no vias: a 35 mm loop
  instead of 17 mm and two hops. MODEM_TX, CHG_CE and VMDM straightened. Collinear
  micro-jogs merged on CHG_STAT and SYS_SW by nudging the ground via they dodged. DRC
  unchanged, 0 / 0.
- **Owner's hand fixes to the Oct 14 autoroute** (`hardware/nano/autoroute/`, git-ignored):
  - P12_IN (0.75 A): 6.7 mm at 0.15 mm beside the RJ45's pin 8, then a 2 mm stub into
    the SS36 reverse-polarity diode. Made 0.8 mm end to end.
  - V12N: 28 mm at 0.15 mm widened to 0.5 mm, its 2 mm diode stub cut to 0.8 mm, and its
    via moved out from between the RJ45 pins and P12_IN.
  - The RJ45's GND pin 7 lost its stub and via (the pour takes it), and another GND via
    between the diode and the connector went too.
  - V3V3 trunk 0.8 → 1.3 mm. SYS's 2 mm stub cut to 0.8 mm.

  The owner's rule: **a trunk keeps one width; no long neck, and no stub fatter than its
  trunk.**

## What the router (50bb483) still did on 18b487c3

Same board, `--flows boards/nano.flows.json`. The current path of each flow, traced
through the copper from source to sink along the widest path:

```
P12_IN  J_12V->D_REV12 0.75 A   0.2 x 6.7 mm, then 2.0 x 1.8 mm      <- same neck, same stub
V12N    D_REV12->U12   0.75 A   1.6 | via | 2.0 x 12.9, 0.2 x 11.7 B
SYS     U1->Q_SW       2.6 A    0.2 x 1.3 | via | 1.3
BAT     J_BAT->U1      2.6 A    ... 0.2 x 1.7 at the charger
VMDM    U11->U7        2.0 A    0.3 x 5.2 out of the TPS63802 pin
V3V3    U2->U4         0.5 A    0.2 along most of 44 mm
```

So "power floors at class width" (50bb483) did **not** cover the necking. The class width
on this board is 0.2 mm. Pin 7 and its stub were already fixed (the pour takes it).

Signal vias: 161 by the router, 75 by hand. On nets whose pads are all on one side:
101 vs 40. Two-via hops on short nets, each crossing a power track:

```
Net-D1-1..D5-1 (LED resistor -> LED, ~6 mm)   hop SYS_SW (a tap of it)
CC1, CC2 (USB-C -> 5.1 k, 2.3 mm by hand)     hop the VBUS trunk
SD_CS, SD_SCK                                 hop V3V3
```

## Causes

1. With `--flows`, a power connection was searched at `max(class width, IPC width)`:
   0.2 mm for anything up to about 1 A here. The power width (Power 0.5, Rail 0.3) was
   used only without flows.
2. `fatten` widened every track on its own to the widest that fit. It grew short stubs
   on big pads to 2 mm and left 0.2 mm wherever the room ran out.
3. String pulling (`pullTight`): a shortcut that starts in a neck at a fine-pitch pin
   took the neck's width along its whole length (VMDM: 5.2 mm at the pin's 0.3 mm).
4. Nothing checked the finished board's current paths. KiCad's DRC cannot see a neck.

## What changed in the router

- A power connection carrying a planned current is searched at `max(power width, IPC
  width)`.
- `fatten` works per run (a route's stretch on one layer). Each run gets one width, the
  widest that fits along all of it. No run is wider than the route's longest run. A tap
  is no wider than the copper it joins. A run landing on a narrower pad necks to the
  pad's size for neckMm past its edge only.
- A shortcut out of a neck may keep the neck's width only within neckMm of that pad.
- After tidy, `flowcheck.py` follows every flow of 0.3 A or more and logs a WARNING for
  any stretch of its path thinner than the trunk (necks onto narrow pads excepted).
- A new "unhop" step after via cleanup: a route between two pads on one side that hops
  layers is ripped with the routes it crosses. The hopper is laid on its own side with no
  via, then the others are laid again. Kept only if every one routes, with fewer vias and
  less cost.

## Result on 18b487c3 (seed 0)

DRC 0 / 0. Current paths:

```
P12_IN  0.5 x 8.4 mm, no neck, no stub
V12N    0.5 x 0.7 F | via | 0.5 x 26.6 B
SYS     0.74 at the charger pin | via | 0.8
BAT     0.8 throughout, 0.2 x 0.5 mm into the charger's pin
VMDM    0.55 x 16.8 mm, no neck
V3V3    0.3 (Rail width) throughout
```

Left as warnings: switch nodes (L1/L2 of the modem buck-boost and the 3.3 V buck) at
0.3 mm for 0.7–0.9 mm out of their pins (0.35 mm on the hand board). VBUS has 0.85 mm at
0.4 mm.

## Still open

- **Vias: 168 by the router vs 75 by hand** (one-side nets 97 vs 40). Unhop undid 2
  hops. The hops that remain cross power trunks with taps hanging off them (SYS_SW,
  VBUS, V3V3), and unhop only moves routes with nothing attached. Next step: rip a
  crossed trunk together with its taps.
- **Bundles sent to the bottom layer.** Corridors put bundle members between two top-side
  blocks on the bottom (80 "sig bundle same side" vias). MODEM_RX: 6 vias, 22.6 mm on the
  bottom under eight top-side nets; 0 vias on the hand board.
- **Width of light rails.** The owner widened V3V3 to 1.3 mm. The Rail class's 0.3 mm
  stays wherever a whole run cannot widen.
