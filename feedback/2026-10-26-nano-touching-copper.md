# Feedback: `--keep-copper` re-routes connections the board already makes (2026-10-26)

Board: the Nano (`hardware/nano/board/nano.kicad_pcb`) with four new 0402s placed for the level-shifter
bypass caps and the modem TX pull-up, plus the touch-reset resistor and ESD clamp. Repro case next to this file in
`2026-10-26-nano-touching-copper/` (`nano.kicad_pcb` as placed, KiCad DRC: 0 violations, 7 unconnected, all on the
new parts; `run.log` is the run below).

```
run.py nano.kicad_pcb --out D --keep-copper --nets "V3V3,MODEM_TX,CTP_RST" --flows boards/nano.flows.json \
  --knob fanoutParts=none --knob stitch=false --minutes 5 --tidy off
```

## What it did

DRC 0 / 0 afterwards, and the seven new connections were made. But it also laid V3V3 copper for four links the board
already had, all far from the new parts:

| link it routed again | how the board already joins it |
|---|---|
| U6 pin 2 - pin 3 (IMU, 0.5 mm pitch) | a 0.2 track ending at (19.61, 49.50), in the gap between the two pads, its width overlapping both |
| U5 pin 7 - pin 8 (GNSS receiver, bottom) | a 0.35 track from pin 8 whose width overlaps pin 7's edge |
| R_SDA 2 - R_SCL 2 | (same kind: joined by overlap, not by a track end inside a pad) |
| R_SD3 2 - R_SDCMD 2 | (same) |

and a 0.4 mm stub from the V3V3 via at (50.5, 13.1) up to the 0.35 track it already touches by its rim.
All of it is redundant copper: the result has loops on V3V3 and a via 0.6 mm from an existing one (C_STAT3V3 was
taken to a new via at (46.75, 12.05) instead of to U17 pin 6, 1 mm away). It was removed by hand from the result.

## Cause

`router.py loadExisting()` joins a piece of existing copper to a pad (or to another piece) only when one of the
track's **end points** (or the via's centre) lies inside the other item's shape:

```python
anchors = [(it.layers[0], q) for q in it.pts] if it.kind == "track" else [(l, it.pts[0]) for l in it.layers]
...
if ... not o.geom.buffer(1e-3).contains(pt): continue
```

KiCad joins on **shape overlap**: a track whose width covers the edge of a pad, or a via whose rim overlaps a track,
is connected (that is why KiCad's DRC reports these nets complete). So `components()` sees the U6 / U5 pads as
separate, `connected()` says the connection is not made ("28 of them are already made" counted only the others),
and the connection is routed again.

**Asked:** in `loadExisting`, join an item to a pad / item of its net when their geometries overlap (`geom.intersects`
on the layer, the way KiCad's connectivity does), not only when an end point is inside. A quick cross-check that
would have caught it: before routing, compare the set of connections the model thinks are open with KiCad's own
unconnected list for the board as loaded (`kicad-cli pcb drc`); any connection the model calls open that KiCad calls
made is a model bug, and should be logged rather than routed.

## Also seen

- With stitching on (default), the same run added 13 return vias and 14 stitching vias across the whole board on a
  three-net job. `--nets` should limit stitching to the vias of the nets asked for (`--knob stitch=false` was the
  workaround).
