"""Inspect a Nav2 occupancy grid PGM/YAML at the ramp area."""
import sys
import yaml
from pathlib import Path
import numpy as np
from PIL import Image


def inspect(yaml_path):
    yaml_path = Path(yaml_path)
    meta = yaml.safe_load(yaml_path.read_text())
    pgm_path = yaml_path.parent / meta['image']
    print(f"YAML: {yaml_path}")
    print(f"  image: {meta['image']}")
    print(f"  resolution: {meta['resolution']} m/pix")
    print(f"  origin: {meta['origin']}")
    print(f"  negate: {meta['negate']}  free_thresh: {meta['free_thresh']}  occupied_thresh: {meta['occupied_thresh']}")

    img = np.array(Image.open(pgm_path).convert('L'))
    h, w = img.shape
    print(f"  PGM size: {w} cols × {h} rows")

    # Convert world (X, Y) to pixel (col, row).
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]

    def w2p(x, y):
        col = int((x - ox) / res)
        row = h - 1 - int((y - oy) / res)
        return col, row

    # Ramp area:
    #   ramp_north_entry: x ∈ [-2, +2], y ∈ [19.6, 20.0]
    #   ramp_north:       x ∈ [-2, +2], y ∈ [15.08, 19.54]
    #   ramp_inner:       x ∈ [-2, +2], y ∈ [7.0, 15.10]
    # Combined ramp box: x ∈ [-2.1, 2.1], y ∈ [6.9, 20.2]
    c0, r0 = w2p(-2.1, 20.2)   # top-left (max y, min x)
    c1, r1 = w2p(2.1, 6.9)     # bottom-right (min y, max x)
    rmin, rmax = sorted([r0, r1])
    cmin, cmax = sorted([c0, c1])
    print(f"\nRamp area in pixels: rows {rmin}-{rmax}, cols {cmin}-{cmax}")

    sub = img[rmin:rmax+1, cmin:cmax+1]
    n = sub.size
    n_free   = int((sub > 250).sum())   # white/free
    n_occ    = int((sub < 50).sum())    # black/occupied
    n_unkn   = n - n_free - n_occ
    print(f"  Pixels in ramp area: {n}")
    print(f"    free  (>250): {n_free:6d} ({100*n_free/n:5.1f}%)")
    print(f"    occup (<50):  {n_occ:6d}  ({100*n_occ/n:5.1f}%)")
    print(f"    unknown:      {n_unkn:6d} ({100*n_unkn/n:5.1f}%)")


if __name__ == '__main__':
    inspect(sys.argv[1])
