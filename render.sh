#!/bin/sh
# Render a routed board's copper to PNGs for review: top, bottom, and both.
#   tools/humanRouter/render.sh <board.kicad_pcb> <outDir> [widthPx]
set -e
board="$1"; out="$2"; w="${3:-2400}"
mkdir -p "$out"
for spec in "top:F.Cu,Edge.Cuts" "bottom:B.Cu,Edge.Cuts" "both:F.Cu,B.Cu,Edge.Cuts" "in2:In2.Cu,Edge.Cuts"; do
  name="${spec%%:*}"; layers="${spec#*:}"
  kicad-cli pcb export svg --layers "$layers" --mode-single --page-size-mode 2 \
    --exclude-drawing-sheet -o "$out/$name.svg" "$board" >/dev/null
  rsvg-convert -w "$w" -b white "$out/$name.svg" -o "$out/$name.png"
done
echo "$out"
