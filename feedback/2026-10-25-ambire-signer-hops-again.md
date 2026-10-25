# Feedback: layer hops again, and a ground track into the pour (2026-10-25)

From the board's owner, with a screenshot: **"what do you have to say for yourself …
what does the router have to say about this stupidity."**
- Repo: AmbireTech/secure-signer, `board/wallet.kicad_pcb`.
- The router finished the chin around the owner's hand routing: commit `f5cf3e3`, on top
  of `10dc1f8`.
- Run: `routeGauge.py` with `--knob nudge=1` on `under-part-jumps`, then a GND-only pass
  with `--knob gndRepair=join`.

## The screenshot

**1. A pointless hop.** RADIO_RXD_MCU drops through a via at (17.25, 37.9), runs 1.7 mm
on F.Cu, and comes straight back up at (15.53, 37.9). The net continues on B.Cu at both
ends. The 0402 cap it hops around, C_VDD11A, could have been passed on B.Cu, or
the crossing re-ordered.

**2. A ground track into the pour.** A 1.06 mm GND track runs from C_VDD11A pin 2 at
(16.3, 38.15) to (17.05, 37.4), where it ends in fill. That is `repairUnreached`'s
"track to the pour". It only happened because we ran `gndRepair=join`, so the choice to
enable it was ours. But it shows exactly what the owner calls slop. The owner fixes such
a pad by hand.

## Hops are not rare: 16 new ones from this one pass

Found by `scripts/routeDefects.py --since 10dc1f8` in the board repo:

| Net | Vias at | Run between | Layer |
|---|---|---|---|
| VBUS_SENSE | (2.48, 31.48) / (3.7, 26.15) | 5.8 mm | B.Cu |
| RADIO_RXD_MCU | (15.53, 34.12) / (15.53, 37.9) | 3.8 mm | B.Cu |
| RADIO_RXD_MCU | (17.25, 37.9) / (15.53, 37.9) | 1.7 mm | F.Cu |
| CELL_N_SENSE | (26.5, 36.88) / (23.07, 36.88) | 3.9 mm | F.Cu |
| CELL_N_SENSE | (9.45, 37.27) / (12.22, 37.9) | 3.0 mm | B.Cu |
| CELL_N_SENSE | (23.8, 38.25) / (23.07, 36.88) | 1.7 mm | B.Cu |
| CELL_N_SENSE | (7.88, 62.88) / (6.42, 62.88) | 1.4 mm | F.Cu |
| CELL_N_SENSE | (7.88, 62.88) / (8.43, 61.6) | 1.3 mm | B.Cu |
| CHG_TS | (33.73, 48.8) / (34.55, 53.27) | 5.0 mm | F.Cu |
| CHG_TS | (34.4, 60.92) / (34.0, 58.08) | 3.0 mm | F.Cu |
| V3V3_USB | (44.85, 56.2) / (47.38, 60.95) | 5.8 mm | F.Cu |
| BAT | (36.42, 59.83) / (38.1, 59.83) | 1.7 mm | F.Cu |
| AIRGAP | (12.53, 67.38) / (11.12, 63.95) | 4.0 mm | F.Cu |
| FB_BOOST | (39.15, 68.7) / (38.83, 67.12) | 1.7 mm | B.Cu |
| V_BOOST | (36.12, 69.05) / (36.25, 72.47) | 3.5 mm | B.Cu |

CELL_N_SENSE alone makes five hops on one net.

## Asked of the router (again, now more urgent)

1. The hop tidy class from the 2026-10-23 report. Re-lay each via pair's short run
   on the layer the net continues on, and drop both vias when DRC allows. Without it,
   every finished route needs a manual sweep.
2. Treat a two-via crossing as congestion in negotiation. A net that would hop twice
   within a few mm should pay for it, so the router tries ripping the crossed net first.
3. Retire "track to the pour" from `gndRepair=join`, or make it a separate opt-in.
   This owner wants a pad the pour misses reported, never a track laid into fill.
