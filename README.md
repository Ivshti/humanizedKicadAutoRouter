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
| `--knob NAME=VALUE` | override one routing setting, e.g. `--knob viaMm=3`; repeatable. The settings and their defaults are listed in `Knobs` at the top of `router.py`; the useful ones are in the table below |
| `--keep-copper` | keep every track, via and zone already on the board and route only what is missing; see [Routing part of a board](#routing-part-of-a-board) |
| `--route blocks` | route only the wiring inside functional blocks and the ground fanouts; the links between blocks are left to you |
| `--nets A,B*` | route only these nets (names or wildcards) |
| `--stitch-only` | route nothing: keep the board's copper, add return vias beside its signal vias and stitch its outer ground pours to the plane |

Knobs worth knowing:

| knob | default | what it does |
|---|---|---|
| `viaMm` | 5 | what a via costs, in mm of track |
| `topDiagonal` | `auto` | the top layer's main diagonal: `/`, `\` or `auto` |
| `pairSkewMm` | 1.5 | length mismatch allowed in a differential pair with no skew rule of its own |
| `signalWidth` | 0 | signal track width; 0 uses your Default net class |
| `neckMm` | 0.5 | a power track narrows only this close to a pad of its own that is narrower than the track; anywhere else it keeps its width or the connection is reported unrouted |
| `fanoutParts` | `caps-ics` | which ground pads get their own via: `all`; `caps-ics` (capacitors and multi-pin parts other than connectors; a resistor's, diode's or connector's ground pad takes the pour, and gets a via only when the pour misses it); or `none` (every ground pad takes the pour, stitching feeds the pour, and a pad the pour misses is reported, not given a via). Pads of other plane nets (a supply plane) always get their via, since nothing pours them on the outer layers |
| `fanoutViaGapMm` | 0.25 | bare board kept between a ground fanout via and any pad edge, its own pad included |
| `fanoutInsideMult` | 3 | how much more a fanout pays where its via does not belong: inside a small part's outline (so the via lands on the far side from its other pads), or in the band just outside a chip's ring of pins, where the wiring to its other pins arrives (so the via goes inside the ring, under the body) |
| `thermalHard` | `true` | other nets may not cross the path of a ground pad's thermal spokes (except within 0.3 mm of their own pads); `false` makes it only a cost (`thermalPenalty`) |
| `stitch` | `true` | stitch the outer ground pours to the plane, and add return vias |
| `stitchPitchMm` | 4 | spacing of the stitching grid in open pour |
| `stitchPerIsland` | 2 | vias (or through-hole ground pads) every piece of outer pour gets, so none hangs on pad thermals alone |
| `stitchPadGapMm` | 0.5 | bare board between a stitching or return via and any pad |
| `blockLayer` | `free` | `own`: wiring inside a block stays on its parts' layer, and the other outer layer is left for the links between blocks. A connection that cannot be routed that way may use the other layer |

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
    matched to 1.5 mm (`--knob pairSkewMm=…`).

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
   parts sit on. Ground pads of capacitors and chips get a fanout via to the
   plane, with bare board between the via and every pad: on the far side of a
   small part, and inside the pin ring of a chip, out of the way of the wiring
   that arrives at its pins. Other ground pads take the pour.
4. **Power.** Biggest current first, as trunks from source to consumers.
   High-current paths avoid changing layers. A power track keeps its width all
   the way into its pad; it narrows only to land on a pad narrower than itself.
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
   are a last resort. A differential pair is drawn as one: a single search
   wide enough for both tracks at the pair gap, then split into the two
   tracks, which run side by side, turn together and splay apart just enough
   at every layer change for their two vias. Where the two pads are on the
   wrong sides for that (the pair would have to cross), the second net
   follows the first at the pair gap instead.
7. **Push and shove.** A connection that cannot get through may route across
   others at a price. The ones it crossed are ripped and routed again, and the
   board remembers where the fights were.
8. **Tidy up.** The worst detours are re-routed against the finished board.
   Then every route with vias is retried with vias priced much higher. A new
   route is kept only if it is better. Then every route is retried once more
   with corners priced high and kept when it has fewer of them and is no
   longer. Finally a via that sits a short stub away from a pad is moved so
   the stub leaves the pad's centre in a straight line, along the pad's long
   axis when it can. Anything a tidying pass left apart is routed again.
9. **Length matching.** The shorter net of each differential pair gets a few
   tall trapezoid bumps on its longest straight runs until the pair is within
   its skew budget. Each bump is checked against the finished board before it
   is placed, so a pair in a tight spot may end up matched less closely. A
   pair with no skew rule of its own that is off by several times the default
   budget took two different paths; it is reported and left alone rather than
   buried under bumps.
10. **Power copper.** Every power track is widened to the widest that fits, so
    neighbouring tracks of one net overlap into a small pour, the way a person
    lays overlapping thick tracks. A track that ends on a pad grows no wider
    than that pad.
11. **Return vias.** Every signal via without a ground via within 1.5 mm gets
    one within 2 mm, so the return current can change planes where the signal
    changes layers.
12. **Planes and stitching.** Ground is poured on the outer layers (and on the
    inner ones when the board has no planes of its own). Then the pour itself
    is stitched to the plane: every piece of it gets two vias (a through-hole
    ground pad counts as one), so none hangs on pad thermals alone, and open
    pour gets a via every 4 mm. Stitching vias keep 0.5 mm of bare board to
    every pad and stay out of keep-out areas. A ground pad whose thermal spokes
    end up blocked is given a solid connection.
13. **Check and repair.** KiCad's DRC runs on the saved board. A ground pad
    that the pour could not reach is then given a fanout via after all, pushing
    other wiring aside, and the board is written and checked again. The last
    DRC result is what is reported.

Throughout, signal tracks are drawn to run one clearance away from existing
wiring. Tracks pack together the way a person nudges them, and the space left
over stays in one piece for the ground pour.

All tracks are horizontal, vertical or 45°. Each piece of a route is pulled
straight after the search: a stair-step the grid produced becomes one straight
run plus one 45° leg, with the bend placed where the route already turns, and
every track ends on the centre of its pad, entering along the pad's long axis.
Signal tracks use the width of your Default net class.

---

## House rules it follows

- Inner layers carry nothing but your planes (ground, if you drew none).
- No copper of another net under the antenna or under module and connector
  bodies on their side.
- No via touches any SMD pad, and a ground fanout via keeps 0.25 mm of bare
  board to every pad.
- Nothing of another net crosses the path of a ground pad's thermal spokes, so
  the pour reaches the pad.
- Pad-specific clearances (for example around fiducials) and pads with
  different shapes per layer are respected.
- Every track and via is checked against exact geometry before it is
  placed. KiCad's DRC then has the final word.

---

## Routing part of a board

You can let the router finish a board you have started, or do the tedious
part of one you will finish yourself:

```sh
$KPY run.py myboard.kicad_pcb --out routed/ --keep-copper --route blocks --knob blockLayer=own
```

- `--keep-copper` keeps everything you drew: tracks, vias and zones. Nothing
  new may cross it, and push and shove never moves it. New tracks may join it.
  A connection your copper already makes is not routed again.
- `--route blocks` routes only the short wiring inside each functional block
  (an IC or module with its passives) and the ground fanouts. The buses
  between blocks are left for you.
- `--nets` limits the run to the nets you name.
- With `--keep-copper`, ground is poured only on outer layers that have no
  ground zone of your own; your zones and the new ones are stitched as above.
  Pairs that already have copper of yours are not length-matched.
- `--stitch-only` routes nothing at all: it adds return vias beside your
  signal vias and stitches your outer ground pours, then reports the ground
  pads the pour does not reach.
- The DRC report then counts what was left for you as unconnected. The log
  says how many connections that is.

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
- **tidiness:** how the tracks read: bends per 10 mm of track, jogs (a run
  that shifts sideways and carries on within 1 mm), steps (any piece under
  0.5 mm between two bends) and the share of track ends on SMD pads that
  land on the pad's centre.

All three are handy for comparing against a hand-routed version of the same
board.

For placement rather than routing, `analyze.py myboard.kicad_pcb --facing`
(optionally followed by `U4,U5`) scores how well each part's signal pins face
the parts they connect to, from -1 to 1, and what the score would be with the
part rotated 90, 180 or 270 degrees. A part that would score clearly better
rotated is flagged. It agrees with a hand decision on the reference signer
board: a secure element rotated so its I²C pins face the microcontroller went
from -0.49 to +0.57.

When a result has an unconnected item or a dangling track, run with
`HR_AUDIT=1` in the environment: after each phase the router checks its own
model for track ends that touch nothing, or only wiring it does not know it
depends on, and prints what it finds. `HR_PROBE="x,y;x,y"` (mm) lists what the
model holds at those points, and `HR_PROBE_NET=NAME` lists every route of a
net, after each phase.

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
  board, about 3.2 GB on a 133 × 81 mm one. Route one board at a time.
- **Runs are not deterministic.** Most of the variation comes from push and
  shove.
- **Never moves parts.** If a placement is unroutable, it reports what is left
  unconnected.
- **Inner layers carry only planes.** A board that routes power on an inner
  layer, rather than pouring it, gets that power routed on the outer layers.

---

## Open items

- **Component placement.** The router takes placement as given. Placing parts
  the way a person would, with blocks grouped and connectors and modules
  where they must be, is the next big piece. It would let the tool start from a
  schematic, not a placed board.
