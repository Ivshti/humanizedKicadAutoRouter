# Feedback: pointless layer hops (2026-10-23)

From the board's owner, with screenshots: **"big fails … another utter fail … I saw a lot
of these completely pointless layer jumps."** A hop is a net that drops through a via,
runs a few mm on the other layer, and comes straight back through a second via.
- **Where:** AmbireTech/secure-signer, `board/wallet.kicad_pcb`.
- **Router route:** commit `a4b7bbb`, the router's full route of proto 1.
- **Before the owner's clean-up:** commit `601dd92`.

A short hop here is two vias of one net joined by a single run of at most 6 mm on one
layer, with no pad on it, where the net continues on the other layer at both ends.

| Commit | Short hops |
|---|---|
| `a4b7bbb` (router's route) | 41 |
| `601dd92` (after hand passes) | 21 |
| `0141109` (owner's clean-up) | 11, all crossings |

## The three kinds, with the owner's examples

**1. Stale: the obstacle left and the hop stayed.**
- CHG_OFF went F.Cu → B.Cu for 4.2 mm at y 48.45 → F.Cu. When the router drew it, the
  F.Cu line between the vias was blocked by the MCU USB pair, so it was a real crossing.
- The pair was re-routed later and nothing blocked the line any more. The hop stayed
  until the owner removed it by hand.
- Nothing ever revisits a crossing once the net it crossed has moved, whether the router
  ripped that net in the same run or a later run or the designer moved it.

**2. Born pointless: nothing blocked it even on the board as committed.**
- CELL_SENSE: 1.0 mm on F.Cu at (36.0–37.0, 66.05).
- USB_DP: 2.05 mm at (23.5, 68.5–70.5).
- VBUS: 1.8 mm at (26.5–28.3, 80.7).

Most likely these are kind 1 inside a single run: push and shove ripped the crossed net
after the hop was laid.

**3. Crossing instead of reordering.**
- BTN_B and BTN_N both leave the same SOT-23 transistor (Q_BTN). BTN_N was laid straight
  and BTN_B crossed it with two vias and 1.4 mm of F.Cu.
- The owner's fix: BTN_B straight on B.Cu, BTN_N a little way around it. No vias.
- Most of the 41 hops are this kind: one net crossing exactly one other net. Examples:
  RADIO_TX × RADIO_RX, VBUS × USB_PG, SW_USB × USB_PG, JOY_B × V3V3_USB,
  RADIO_3V3 × VBUS_SENSE, CHG_STAT × VBUS_SENSE, CHG_TS × V_BOOST.
- A legal crossing never registers as congestion, so negotiation never rips the
  crossed net. Two vias cost about 10 mm at viaMm=5, which is cheaper than any detour
  the search can find while the other net stays put.

## Asked of the router

1. **A tidy class for hops.** For each via pair as defined above, try to lay the run on
   the layer the net continues on, straight or by the shortest legal path, and drop both
   vias. Keep it when DRC is no worse, like the other tidy classes. Run it at the end of
   every route and in `--tidy only`, so it catches hops left stale by later edits.
2. **Treat a two-via crossing as congestion.** When a route crosses one other net with a
   via pair, rip the crossed net and re-lay both, and keep whichever is cheaper. It
   matters most when the two nets leave the same part (BTN_B and BTN_N from Q_BTN;
   RADIO_TX and RADIO_RX), where the right answer is to swap their exit order. That is
   the same problem as pair polarity at a part's pins.
3. **A `hop` craft finding** in the audit and in `routeGauge`, so these are counted and
   a quality filter can reject a net that has one.
