# Feedback: GND tracks on a pour-and-stitch board (2026-10-23)

From the board's owner: **"the autorouter keeps adding gnd traces which is complete
slop."** On this board GND is the outer pours plus free stitching vias to a solid In2
plane. A GND track is wanted only where a part asks for a short path to the plane
(switching loops, exposed pads, RF ground, ESD clamp). The board keeps that list, and
the designer draws those tracks. The router should add **no GND tracks** here.

## Where

- Repo AmbireTech/secure-signer, `board/wallet.kicad_pcb`.
  - Input: commit `75ccdd6`, with the MCU USB pair thinned to 0.15 mm.
  - Our filtered result: commit `3731480`. The router's GND tracks were stripped
    before commit; its output is not in git.
- Run: `--keep-copper --knob fanoutParts=none --knob nudge=1`, full finish (all nets
  left), branch `under-part-jumps`.

## What it added

The full run added 23 GND vias and 3 GND tracks. Before routing, the pour already
reached every GND pad these tracks touch. Each track was redundant on the board as it
came in.

1. **A pad-to-via tie at R_VSET pin 2** (the charger's voltage-set divider): a 0.54 mm
   B.Cu track, then 0.61 mm on to a new via at (30.125, 66.475). This is exactly the
   "via beside the pad" the owner forbids. It is just reached by a track first.
2. **A 0.8 mm B.Cu bridge** at (24.475, 69.5) to (25.275, 69.5), joining two GND ties
   the designer had drawn (the ESD clamp's and the USB switch's). Both were already
   connected.

Earlier passes left similar tracks, and the owner has been removing them by hand.

## Likely mechanism

In `run.py`, `repairUnreached()` handles pads that DRC reports as unreached
mid-run. A passive's pad gets `routeToPour` (a track), with a via beside it as the
last resort. The pad is checked while the run's own, not yet judged, signal copper is
on the board. That copper cuts the pour off a pad for a moment. On this run 14 of
the routed nets were later rejected and ripped. The GND repair for them stayed. It is
also our only source of GND tracks.

## Asked of the router

- Under `fanoutParts=none`, never route a GND **track**. A pad the pour misses is a
  finding: list it in the result as "unreached GND pad", and do not tie it.
  - If a knob must keep the old behaviour, it must be opt-in. The default
    for `fanoutParts=none` is no tracks.
- Free stitching vias are fine. They may never be joined to a pad by a track.
- Never bridge two existing pieces of the designer's GND copper with a new track.
- A test for it: a board where a signal route clips the pour off a 0402's GND pad.
  The router must leave that pad unreached and report it, and add no track and no
  via beside it.
