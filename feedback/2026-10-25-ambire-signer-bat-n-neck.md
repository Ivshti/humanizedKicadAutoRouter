# Feedback: the cell's return current ran through a 0.15 mm neck (2026-10-25)

The owner, with a screenshot: **"what the fuck"**.

- Repo: AmbireTech/secure-signer.
- Made by the chin pass, `f5cf3e3` (`routeGauge.py` on `under-part-jumps`, `nudge=1`).
- Survived until the owner fixed it by hand in `931c2a9`.

## What the router laid (B.Cu, BAT_N, Power class, 0.8 mm)

```
Q_REV (AO3416 source side)  (2.1, 61.75) -> (3.4, 63.05) -> (3.4, 63.61)   0.8 mm
                            (3.4, 63.61) -> (5.0, 65.21) = R_CNS.1           0.15 mm   <- neck
R_CNS.1 (100 k cell-sense)  (5.0, 65.21) -> (5.0, 66.28) -> (4.17, 67.1)
                                         -> (4.17, 70.67) = BT_N clip        0.8 mm
```

The cell-sense resistor's pad became a waypoint on the cell's return path. The tap
into it is legitimately thin (microamps), but the path from the clip to the
reverse-polarity FET ran through that tap. So the whole cell current, up to the
boost's input current, crossed 1.6 mm of 0.15 mm track.

KiCad's DRC cannot see this; nothing breaks a rule. Three review passes missed it
too, until the owner saw it.

## Where to look in the router

- `route()` near `okLand`: a current-carrying connection may only land on its own end
  pads or on routes at least as wide. But `srcItems = [...] or srcItems` falls back
  to *every* item of the component when none qualifies. One way here: the clip's
  wide route lands on the tap's pad R_CNS.1, and the thin tap is then the only link
  on to Q_REV.
- The tidy passes (overlap, straighten, dangling) re-join copper after routing. A
  join there can also put a tap in series.

## Asked of the router

1. **Never fall back to landing current on a tap.** If no wide item qualifies, route
   to the end pad, or fail the connection loudly.
2. **A final check after tidy:** for every flow net, no current path between its flow
   pads may pass a piece narrower than the flow needs. The board repo's
   `routeDefects.py` "neck" check (`9bfc185`) is a cheap proxy: a track under 60 % of
   its class with wide copper of the net at both ends and not buried in it. It finds
   this case, and one more on the board now: V_BOOST, 0.15 mm from the LTC3539's
   output pin to its via, feeding a 0.4 mm trunk.
