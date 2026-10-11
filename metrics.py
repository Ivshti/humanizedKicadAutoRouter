"""Copper statistics for comparing routed boards.

    kicad-python tools/humanRouter/metrics.py <a.kicad_pcb> [<b.kicad_pcb> ...]
"""
import math
import sys
from collections import Counter

import pcbnew


def stats(path):
    b = pcbnew.LoadBoard(path)
    length = Counter()
    segs = Counter()
    vias = Counter()
    angles = Counter()
    for t in b.GetTracks():
        if t.GetClass() == "PCB_VIA":
            vias["GND" if t.GetNetname() == "GND" else "signal/power"] += 1
            continue
        s, e = t.GetStart(), t.GetEnd()
        L = math.hypot(e.x - s.x, e.y - s.y) / 1e6
        layer = b.GetLayerName(t.GetLayer())
        length[layer] += L
        segs[layer] += 1
        a = round(math.degrees(math.atan2(e.y - s.y, e.x - s.x))) % 180
        angles["octilinear" if a % 45 == 0 else "any-angle"] += 1
    return dict(lengthMm={k: round(v) for k, v in length.items()}, segments=dict(segs),
                vias=dict(vias), angles=dict(angles))


if __name__ == "__main__":
    for p in sys.argv[1:]:
        print(p)
        for k, v in stats(p).items():
            print(f"  {k}: {v}")
