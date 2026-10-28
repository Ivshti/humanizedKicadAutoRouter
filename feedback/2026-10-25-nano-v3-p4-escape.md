# Feedback: Nano v3 first pass, ESP32-P4 walled in (2026-10-25)

The owner's verdict on the Nano v3 runs (`hardware/nano-v3`, commit `34f9eb72` has the run below as
`autoroute/`): **"routing on the nano v3 is a fucking disaster ... and the part placement is an absolute
disaster"**. The project is paused. Most of the blame is the placement (a greedy packer, written in that session,
seated each part at the free spot nearest its partners' pads); the router's part is how it handled a fine-pitch
QFN with no room to escape. The owner's pointer: **malakaRouter3 has good BGA escape tools, and so does
KiCadRoutingTools** (`projects/KiCadRoutingTools/qfn_fanout.py`, `bga_fanout.py`, `place_fanout_clearance.py`).

## The board

- 73.9 x 71.4 mm (run p2, with a 9 mm chin) and 73.9 x 60.9 mm (run p4, no chin; killed before it finished),
  4 layers, 1.6 mm, **every part on F.Cu**: B.Cu is glued to a display, so nothing can go under the chip.
- ESP32-P4NRW32X, QFN-104 10 x 10 mm, **0.35 mm pitch, 0.2 mm pads**, nine-square exposed pad; around it ~40
  decoupling caps (0402/0603) for its 16 supply pins, the flash (WSON-8), crystal, core buck.
- Parts under the display's folded ribbon tails are height-limited (1.3-1.4 mm), not forbidden.
- Run: `run.py ... --minutes 20 --knob fanoutParts=none`.

## Results

| run | placement | unconnected | of which P4-local nets (P4_*, DSI, CSI, SD, SDIO) | V3V3 | tracks | vias |
|---|---|---|---|---|---|---|
| p2 | greedy, decaps touching the P4's pad ring | 89 | 39 | 10 | 1495 | 533 |
| p3 | p2 + a 2 mm part-free fan-out ring round the P4 (decaps just outside it) | 125 | 48 | 20 | 1289 | 478 |

The fan-out ring made it worse: with the decaps 2 mm out, the router spent the ring on rail tracks to them.

## What the owner's screenshot shows (p2, the P4's south-west corner)

1. **V3V3 as wide (rail-class) tracks threading the decap field** between 0402s, from cap to cap, instead of one
   short via per decap pad to the plane. The rail's trunks took the corridors the QFN's signal pins needed.
2. **The QFN's west pins 53-59 never escaped** (ratsnest lines straight through the copper): their pads face a
   wall of decaps 0.6-1 mm away, and nothing reserved an escape channel for them.
3. **Supply nets of the chip (P4_VDD_MIPI, P4_VDD_PSRAM) run from cap to cap on the outer layer with a via at
   each end**, each its own little tree, beside GND vias at almost every GND pad.

## Asks

- **Escape a fine-pitch QFN/BGA before anything else**, the way malakaRouter3 / KiCadRoutingTools' `qfn_fanout`
  do: every signal pin gets its stub and via (or its outer-layer exit) planned first, and decaps are routed into
  what is left, not the other way round.
- **A placement gauge that says "this chip cannot escape" early**: count each side's pins that have a free
  corridor within N mm and report before spending 20 minutes routing.
- **Decap hookup on single-sided boards**: pad -> short neck -> via to the plane (the owner's rule on the Ambire
  signer: one via per 3.3 V pad, never trunks or daisy chains between plane vias). Rail-class width only on the
  trunk from the regulator, never cap to cap.
