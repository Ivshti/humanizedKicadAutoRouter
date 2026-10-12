# humanizedKicadAutoRouter

An autorouter for KiCad boards that works the way a person routes. It does
functional blocks first, then power sized for its current, then block-to-block
buses in planned corridors. It runs headless, writes a routed copy of your
board, and only counts the result as done once KiCad's own DRC agrees.

On a dense 88 × 70 mm 4-layer board (ESP32 module, LTE modem, GNSS receiver and
patch antenna, chargers, 162 parts) it routes the fully ripped board in 6–10
minutes on an M3 Max. That board took about 3 hours by hand. The result has no
DRC errors or warnings and no unconnected items.

Your input board is never modified.

---

## Contents

- [What you need](#what-you-need)
- [Quick start](#quick-start)
- [Adding it to your project](#adding-it-to-your-project)
- [Preparing your board](#preparing-your-board)
- [Telling it how much current flows (optional)](#telling-it-how-much-current-flows-optional)
- [What you get](#what-you-get)
- [How it routes](#how-it-routes)
- [House rules it follows](#house-rules-it-follows)
- [Reviewing the result](#reviewing-the-result)
- [Limits today](#limits-today)
- [Open items](#open-items)

---

## What you need

- **KiCad 10** (developed against 10.0.4). It provides both the board library
  (`pcbnew`) and `kicad-cli`, which runs the final DRC.
- **A Python that can `import pcbnew`**, with `numpy` and `shapely`:
  - **macOS:** KiCad ships one that already has everything:
    ```sh
    KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
    ```
  - **Linux:** the system `python3` that KiCad's packages install `pcbnew`
    into. Add the rest with `pip install numpy shapely` (or your distro's
    packages).
- **A C compiler** reachable as `cc` (Xcode command line tools, or gcc/clang).
  The fast search core is compiled automatically on first run into `build/`.
- **`kicad-cli`** on your `PATH`.
- *Optional, for PNG previews:* `rsvg-convert` (from librsvg).

No KiCad window is needed. Nothing talks to a running KiCad.

---

## Quick start

```sh
git clone git@github.com:Ivshti/humanizedKicadAutoRouter.git
cd humanizedKicadAutoRouter

$KPY run.py path/to/myboard.kicad_pcb --out routed/
```

Then open `routed/myboard.kicad_pcb` in KiCad.

Options:

| option | meaning |
|---|---|
| `--out DIR` | where the routed copy goes (required) |
| `--flows FILE` | how much current each power net carries; see below |
| `--minutes N` | time budget, default 25; it stops negotiating when this runs out |
| `--knob NAME=VALUE` | override one routing setting, e.g. `--knob viaMm=3`; repeatable. The settings and their defaults are listed in `Knobs` at the top of `router.py`. Useful ones: `viaMm` (what a via costs, in mm of track; 5), `topDiagonal` (`/`, `\` or `auto`), `pairSkewMm` |

Progress is printed as it goes. The last line is the DRC result, for example
`DRC: 0 violations {}, 0 unconnected`.

---

## Adding it to your project

The router is a folder of scripts, so pick whichever way of keeping it suits you:

**As a git submodule** (pinned to a version, updated when you choose):

```sh
git submodule add git@github.com:Ivshti/humanizedKicadAutoRouter.git tools/humanRouter
$KPY tools/humanRouter/run.py hardware/board/board.kicad_pcb --out hardware/board/autoroute
```

**As a plain clone next to your projects**, if you route boards from
several repositories:

```sh
git clone git@github.com:Ivshti/humanizedKicadAutoRouter.git ~/tools/humanRouter
$KPY ~/tools/humanRouter/run.py board.kicad_pcb --out autoroute
```

**In a script or CI job**, run it like any command. Read `summary.json` from
the output folder to decide pass or fail: `violations` should be empty,
`unconnected` should be 0.

```sh
$KPY tools/humanRouter/run.py board.kicad_pcb --out autoroute --minutes 20
python3 -c "import json,sys; s=json.load(open('autoroute/summary.json')); sys.exit(bool(s['violations']) or s['unconnected'] > 0)"
```

Tips:

- **Route into a separate folder, not over your board.** The output folder
  gets its own copy of your project, design rules and footprint-library table
  (pointed back at your libraries), so it opens in KiCad as-is.
- If you keep routed copies out of version control, add the output folder to
  your `.gitignore`.
- Runs are not bit-for-bit repeatable. Two runs of the same board can differ
  slightly and take a minute more or less. If one run leaves something
  unconnected, run it again.

---

## Preparing your board

The router uses what is already in your KiCad project. Before the first run,
check the following.

1. **Placement is final.** It routes; it never moves parts.
2. **The `.kicad_pro` sits next to the board.** Net classes, their track
   widths, clearances and via sizes, and the net-class patterns all come from
   it. A `.kicad_dru` with custom rules is copied along and is honoured by the
   final DRC.
3. **A 4-layer stack-up.** Layers are outer top, two inner, outer bottom.
   Signals and power are routed on the outer layers only.
4. **Planes.** Copper zones you have drawn on the inner layers are kept exactly
   as they are: a solid ground plane, or a ground plane with a power pour on
   the other inner layer, for example. Pads of a plane's net over the plane get
   a fanout via to it; pads outside a partial plane are wired to the net like
   any other. If the board has no inner zones, both inner layers become solid
   ground. Zones on the outer layers are removed with the rest of the copper
   and poured again as ground; a power pour you had there comes back as
   overlapping wide tracks (step 9 below).
5. **Ground is the net called `GND`.** It gets the outer pours, return vias and
   the stitching via field.
6. **Net class names it recognises:**

   | net class | treated as |
   |---|---|
   | `Power` | power: routed early, wide (0.5 mm or the current it carries) |
   | `Rail` | lighter power: routed with power, 0.3 mm |
   | `RF50` | RF: routed first, on its own layer, no vias |
   | `Default` | signal, at 0.2 mm |
   | any other class | signal, at that class's track width |

   A board with no `Power` or `Rail` class still gets its rails routed as
   power: a net in the default class whose name is a voltage or a rail
   (`+5V`, `V3_3`, `VBAT`, `VBUS_C`, `M2_3v3`, `VCC_IO`) is treated as `Rail`.
   Control lines named after a rail (`VBUS_EN`, `PWR_FAIL`, `BAT_DET`) stay
   signals. To use other class names, rename your classes or edit
   `powerWidth` in `router.py`.
7. **Keep-out areas** you have drawn (board-level or inside footprints) are
   obeyed.
8. **Antennas:** a footprint whose reference starts with `ANT` owns its whole
   courtyard on its side of the board. No other copper goes under it.
9. **Big modules and connectors** (courtyard of 40 mm² or more): no foreign
   copper goes under their bodies on their side. Their own pins can still
   escape outward.
10. **Fiducials (`FID…`) and mounting holes (`H…`)** are left alone.
11. **Differential pairs** are found by name: `X_DP`/`X_DN`, `X_P`/`X_N`,
    `X+`/`X-`, and `CANH`/`CANL`. The pair gap is the net class's diff-pair
    gap from the `.kicad_pro`. Their lengths are matched to the skew budget
    of a `(constraint skew (max …))` rule in the `.kicad_dru` whose condition
    names the pair with `A.inDiffPair('…')`; a pair without such a rule is
    matched to 0.5 mm (`--knob pairSkewMm=…`).

Any existing tracks, vias and outer-layer copper pours are removed before
routing. The router always starts from a clean board, plus your inner planes.

---

## Telling it how much current flows (optional)

Without a flows file, power nets are routed at their net-class width. With one,
each power net is laid out as a person would. A trunk runs from the source to
each big consumer, sized by IPC-2221 for the current it actually carries. Small
loads are tapped off it, and tracks neck down only at the pins.

```json
{
  "flows": [
    { "net": "VBUS", "source": "J1",  "sinks": ["U14"], "amps": 1.1 },
    { "net": "V3V3", "source": "U2",  "sinks": ["U4", "U7"], "amps": 0.8 }
  ]
}
```

- `net`: the net name.
- `source`: the reference of the part that feeds it (connector, regulator).
- `sinks`: the parts that draw from it.
- `amps`: the total current on that path; it is split evenly between the sinks.

`boards/nano.flows.json` is a full real example. Pass yours with
`--flows myboard.flows.json`.

---

## What you get

In the `--out` folder:

| file | what it is |
|---|---|
| `<board>.kicad_pcb` | the routed board, planes filled |
| `<board>.kicad_pro`, `.kicad_dru`, `fp-lib-table` | copies so the folder opens in KiCad |
| `drc.json` | KiCad's DRC report for the routed board (errors and warnings) |
| `summary.json` | run time, track and via counts, DRC violations by type, unconnected count |

---

## How it routes

It follows the order a person uses:

1. **Rip.** Every track, via and pour is removed; keep-outs stay.
2. **Find the functional blocks.** Every multi-pin part (IC, module, connector)
   anchors a block, and each passive joins the block it is most tied to.
   Wiring between two blocks becomes a bundle.
3. **RF first, then each block's own wiring.** Short, local, on the side the
   parts sit on. Each ground pad gets its fanout via to the plane.
4. **Power.** Biggest current first, as trunks from source to consumers.
   High-current paths avoid changing layers.
5. **Pick each layer's direction, then plan corridors.** Before the first
   track, it decides which diagonal the top layer's long runs follow and gives
   the bottom the other one, so the two layers run crosswise and long buses
   can live on either. Every bundle is planned both ways on the empty board
   and the cheaper assignment wins (`--knob topDiagonal=/` or `\` to set it
   yourself). Then, with the blocks and power down, it decides where each
   bundle will run and on which layer, on a coarse map of the space left:
   - Running along the other layer's diagonal costs extra, so a bundle picks
     the layer whose direction suits it and changes layer only where its path
     turns the other way.
   - Bundles that can share a channel are drawn into it, side by side.
   - A bundle that would cross other wiring on the same layer is sent to the
     other layer instead, because every such crossing costs a pair of vias.
6. **Route the bundles** inside their corridors, one member after the other in
   order across the bundle, so they lie side by side instead of crossing. A
   member's vias belong next to its own pins or where its corridor planned a
   layer change; a via anywhere else is priced like three, so mid-route hops
   are a last resort. The second net of a differential pair follows the first
   at the pair gap and changes layer right beside it.
7. **Push and shove.** A connection that cannot get through may route across
   others at a price. The ones it crossed are ripped and routed again, and the
   board remembers where the fights were.
8. **Tidy up.** The worst detours are re-routed against the finished board.
   Then every route with vias is retried with vias priced much higher. A new
   route is kept only if it is better.
9. **Length matching.** The shorter net of each differential pair gets
   serpentines on its longest straight runs until the pair is within its skew
   budget. Each bump is checked against the finished board before it is
   placed, so a pair in a tight spot may end up matched less closely.
10. **Power copper.** Every power track is widened to the widest that fits, so
    neighbouring tracks of one net overlap into a small pour, the way a person
    lays overlapping thick tracks. A track that ends on a pad grows no wider
    than that pad.
11. **Return vias.** Every signal via without a ground via within 1.5 mm gets
    one beside it, so the return current can change planes where the signal
    changes layers.
12. **Planes and stitching.** Ground is poured on the outer layers (and on the
    inner ones when the board has no planes of its own) and stitched with a
    via grid. A piece of the outer pour that nothing connects would be removed
    and leave a hole in the ground; each one with room for a via gets one. A
    ground pad whose thermal spokes end up blocked is given a solid connection.
13. **Check and repair.** KiCad's DRC runs on the saved board. A ground pad
    that the pour could not reach is then given a fanout via after all, pushing
    other wiring aside, and the board is written and checked again. The last
    DRC result is what is reported.

Throughout, signal tracks are drawn to run one clearance away from existing
wiring. Tracks pack together the way a person nudges them, and the space left
over stays in one piece for the ground pour.

All tracks are horizontal, vertical or 45°.

---

## House rules it follows

- Inner layers carry nothing but your planes (ground, if you drew none).
- No copper of another net under the antenna or under module and connector
  bodies on their side.
- No via touches any SMD pad.
- Pad-specific clearances (for example around fiducials) and pads with
  different shapes per layer are respected.
- Every track and via is checked against exact geometry before it is
  placed. KiCad's DRC then has the final word.

---

## Reviewing the result

```sh
./render.sh routed/myboard.kicad_pcb previews/         # top, bottom, both, inner-2 as PNG
$KPY metrics.py my-hand-routed.kicad_pcb routed/myboard.kicad_pcb
$KPY analyze.py my-hand-routed.kicad_pcb routed/myboard.kicad_pcb
```

`metrics.py` prints track length per layer, via counts (ground vs.
signal/power) and how many segments are not 0/45/90°.

`analyze.py` looks at the things a reviewer looks at:

- **direction:** track length per layer by direction, for all tracks and for
  long ones. It shows whether each layer has a main direction.
- **vias:** signal vias, and how many have no ground via within 1.5 mm.
- **reference:** signal track length that does not run over the neighbouring
  inner ground plane, for example over a slot between via clearances.
- **pour gaps:** areas on the outer layers with no copper at all, larger than
  1 mm², i.e. places the ground pour could not reach.

All three are handy for comparing against a hand-routed version of the same
board.

---

## Limits today

Numbers are from the reference board, compared with the same board routed by hand.

- **More vias than a person uses.** About 160–180 signal/power vias against 77
  by hand. Most are escape vias next to the pins, two per long bundle member.
  Your hand-routed board keeps more long runs on top. Mid-route hops have come
  down but are still several times the hand-routed count.
- **Differential pairs** are coupled where there is room, and pairs that end
  up apart are re-routed together. On the reference board USB comes out about
  half to two-thirds coupled, like the hand-routed board. The long CAN pair
  varies from run to run (20–55%). Lengths are matched by serpentines; a pair
  with no straight run left to bend can stay a millimetre or two apart.
- **4-layer boards with a `GND` net.** Two-layer and six-layer boards need
  changes.
- **Run time** is 6–10 minutes on the reference board. Most of it is push and
  shove, and it depends on how crowded the board is.
- **Memory.** Everything is rasterised at 0.025 mm over the whole board, so
  memory grows with board area: about 2.2 GB peak on the 88 × 70 mm reference
  board, roughly four times that on a 133 × 81 mm one. Route one board at a
  time.
- **Runs are not deterministic.** Most of the variation comes from push and
  shove.
- **Never moves parts.** If a placement is unroutable, it reports what is left
  unconnected.

---

## Open items

- **Component placement.** The router takes placement as given. Placing parts
  the way a person would, with blocks grouped and connectors and modules
  where they must be, is the next big piece. It would let the tool start from a
  schematic, not a placed board.
