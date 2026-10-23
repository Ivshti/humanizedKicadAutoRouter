# Feedback: re-routing single nets on a finished board (2026-10-23)

The board was fully routed by hand: AmbireTech/secure-signer `board/wallet.kicad_pcb`,
commit `c359382`, DRC clean. The owner asked for the detoured nets to be re-laid where
the router does better. We ripped one net at a time and ran
`--keep-copper --knob fanoutParts=none`. A net's new route was kept only if it was
shorter, had no more vias and no craft finding, and left KiCad's DRC clean.

## 1. Routes wall a ground pad's pour off from every via

Two routes came out clearly better on length and vias, but each cut a small piece of
B.Cu GND pour, holding a GND pad, off from every stitching via:

| Net | Owner's route | Router's route | Pad cut off | Pour piece |
|---|---|---|---|---|
| CHG_STAT | 90 mm, 6 vias | 60 mm, 2 vias | `R_TMR` pin 2 (the charger timer resistor) | 1.6 mm² at x 28.7–30.1, y 66.3–68.0 |
| CHG_HI | 55 mm, 4 vias | 41 mm, 4 vias | `C_VDD2` pin 2 (an MCU decoupling cap) | 1.0 mm² at x 28.1–29.4, y 36.6–38.7 |

DRC reports these as an unconnected pour. `--stitch-only` found "1 piece of pour with
no room for a via", so neither route can be repaired afterwards.

**Asked of the router:** a route must not leave a GND pad's piece of outer pour without
a via, an existing one or room for a new one. The thermal-corridor cost protects the
pad's spokes, but not the piece of pour those spokes land on. Check it per route when
it is accepted, with the pour fill raster the stitching pass already builds.

## 2. Crash in `writeRouterCopper` after tidy

Single-net runs of FB_BOOST and CHG_ISET, with and without `--knob nudge=1`, finished
routing and tidy, then died in `writeRouterCopper`:

```
File "run.py", line 371, in writeRouterCopper
  rec = {t.m_Uuid.AsString(): itemKey(t) for t in board.GetTracks()
TypeError: 'swig_runtime_data5.SwigPyObject' object is not iterable
```

- The board had already been saved and DRC-checked, so only the sidecar was lost.
- Tidy had reverted kept-back edits in those runs; `revert()` re-adds removed tracks
  with `board.Add`. Other runs that also reverted did not crash, so the trigger is
  narrower than that.
- It is not tied to `nudge`: it reproduces without it. Every crash so far was on a net
  in the charger corner that the router then left unconnected (FB_BOOST, CHG_ISET).

Repro: rip FB_BOOST from that commit, then run
`run.py … --keep-copper --knob fanoutParts=none`.
