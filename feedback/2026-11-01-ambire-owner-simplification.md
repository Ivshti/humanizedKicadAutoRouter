# Feedback: the owner's simplification pass on the Ambire v4 board (2026-11-01)

- Repo: malaka-chrono `hardware/ambire-hw` (the Ambire Ice device board, v4).
- Before: `82717f7` (the board after the last routing/quality passes), copy in
  `2026-11-01-ambire-owner-simplification/before-82717f7.kicad_pcb`.
- After: `c843049`, the owner's hand pass, `after-c843049.kicad_pcb` beside it
  (load either as `wallet.kicad_pcb` next to the `wallet.kicad_pro` in that folder,
  or KiCad applies default rules).
- Measured by diffing copper per net (vias, track length, segments) and by listing, for
  every via pair the owner removed, what lay on the other layer between the two vias.

## What the owner did, in numbers

| Net | Vias | Length | Segments |
|---|---|---|---|
| CELL_N_SENSE | 6 → 2 | 53.5 → 44.1 mm | 21 → 19 |
| JOY_DIR | 5 → 1 | 89.8 → 107.8 mm | 35 → 23 |
| I2C_SCL | 6 → 2 | 35.5 → 35.4 mm | 19 → 13 |
| AIRGAP | 5 → 2 | 75.0 → 67.3 mm | 35 → 28 |
| RADIO_OFF | 4 → 2 | 32.5 → 25.9 mm | 23 → 17 |
| CTP_INT | 4 → 2 | 15.2 → 14.9 mm | 9 → 7 |
| VBUS_SENSE | 4 → 2 | 76.0 → 70.6 mm | 28 → 20 |
| GND (stitching) | 275 → 274 | | |

Seven signal nets lost 22 vias; six of the seven also got shorter. JOY_DIR (a front
ladder of five resistors to one MCU pin on the back) traded 4 vias for 18 mm of track:
one layer change, at the destination.

## What was "in the way" of each hop the owner removed

Listed from the before board: for each via pair with a short run between them, the
nets whose copper sat on the other layer inside a 0.9 mm corridor between the vias.

| Net, via pair | Run | On the other layer |
|---|---|---|
| CTP_INT (30.48, 20.68)–(31.45, 20.68) | 1.0 mm on F.Cu | I2C_SCL track, a GND stitching via |
| CTP_INT (38.64, 19.86)–(37.73, 20.68) | 1.1 mm on F.Cu | I2C_SDA track |
| CTP_INT (37.73, 20.68)–(31.45, 20.68) | 6.3 mm on B.Cu | BUZZER track, a GND via |
| I2C_SCL (36.48, 22.85)–(37.4, 21.93) | 1.3 mm on F.Cu | V3V3 track + via, a CTP_RST via |
| I2C_SCL (33.61, 16.51)–(34.62, 15.5) | 1.4 mm on F.Cu | I2C_SDA track, a GND via |
| I2C_SCL (28.48, 27.45)–(30.95, 22.18) | 6.3 mm on F.Cu | V3V3, PWR_HOLD, CTP_RST, IMU_INT pads/tracks |
| I2C_SCL (36.48, 22.85)–(30.95, 22.18) | 5.8 mm on B.Cu | BUZZER track, CTP_RST track + via |
| CELL_N_SENSE (29.88, 37.17)–(26.98, 37.17) | 2.9 mm on F.Cu | V3V3 pad + track, a GND pad |
| CELL_N_SENSE (29.88, 37.17)–(33.08, 39.1) | 4.0 mm on B.Cu | CELL_MID track + via, CHG_HI track, a GND via |
| RADIO_OFF (10.65, 41.6)–(6.86, 45.49) | 5.6 mm on F.Cu | V3V3 pad/track/via, BL_A, BL_K, a GND via |
| AIRGAP (7.72, 49.7)–(9.82, 53.88) | 5.3 mm on F.Cu | RADIO_3V3, RADIO_EN, a GND via |
| VBUS_SENSE (3.12, 57.38)–(3.55, 61.73) | 6.2 mm on B.Cu | AIRGAP track, USB_MCU_DN track |
| JOY_DIR (43.12, 42.42)–(39.12, 38.95) | 5.4 mm on B.Cu | CHG_STAT track, a GND via |

In 9 of the 13 the obstacle included **plane-net copper**: a V3V3 track or a GND
stitching via. In two (the 1.0 mm CTP_INT hop the owner screenshotted, and the I2C_SCL
hop at (36.5, 22.9)) that plane copper was most or all of what the hop crossed.

## Lessons for the router

1. **Plane-net copper is not an obstacle; it is movable.** V3V3 is a plane (In1) and GND
   is a plane (In2) plus the outer pours. A V3V3 *track* on an outer layer is only a
   pad-to-via fanout, and a GND stitching via is free: either can be re-dropped a
   millimetre away at no cost. The router hopped CTP_INT through two vias to cross one
   I2C_SCL track and a GND stitching via, and I2C_SCL through two vias to cross a V3V3
   fanout. The owner's words: "jumping over a 3V3 trace given that the one is
   completely flexible". Rule: before paying a via pair, try moving the plane-net
   copper in the way (slide the stitching via, re-drop the fanout via on the other side
   of the signal); treat only *signal* copper and pads as fixed.
2. **Short hops are never worth it.** Every hop under 1.5 mm the owner removed cost two
   vias to cross one track; each was solved on the hop's own layer by nudging what it
   crossed or by going around it. `routeDefects.py` only lists a hop when its own layer
   already has a way, so it missed these; it now also lists hops whose only obstacles
   are plane-net tracks and free stitching vias.
3. **One layer change, at the destination** (again, as on 2026-10-25): JOY_DIR's five
   front pads now run on the front to the MCU and drop once at its pin; 4 vias for 18 mm
   of track. The router's cost model still prices a via below a few mm of detour.
4. **Net length is not the objective.** Six of seven nets got shorter anyway, but
   JOY_DIR got 20 % longer and that was the right trade for the owner.
