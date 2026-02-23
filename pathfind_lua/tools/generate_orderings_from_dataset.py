#!/usr/bin/env python3
"""
generate_orderings_from_dataset.py

Generate a bench-compatible orderings file for pathfind_lua from an existing
CMaNGOS-style dataset (mmaps/ + maps/), by sampling tiles that exist on disk.

This avoids depending on an external orderings.lua/json file.

Output format matches bench.lua expectations:
  ["orderings"] = {
    { 0, {0}, { {x1,y1,x2,y2}, ... } },
  }
"""

from __future__ import annotations

import argparse
import random
import re
from pathlib import Path
from typing import List, Tuple


MMTILE_RE = re.compile(r"^(?P<map>\d{3})(?P<tx>\d{2})(?P<ty>\d{2})\.mmtile$")
MAP_RE = re.compile(r"^(?P<map>\d{3})(?P<tx>\d{2})(?P<ty>\d{2})\.map$")

BLOCK_SIZE = 533.33333
TILE_ORIGIN = 32.0


def tile_center_world_xy(tx: int, ty: int) -> Tuple[float, float]:
    # Invert PathFinder.worldToTile:
    #   tx = floor(32 - x/BLOCK)
    # Choose x such that 32 - x/BLOCK is in (tx, tx+1); use +0.5.
    x = (TILE_ORIGIN - (tx + 0.5)) * BLOCK_SIZE
    y = (TILE_ORIGIN - (ty + 0.5)) * BLOCK_SIZE
    return x, y


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", required=True, help="Directory containing mmaps/ and maps/")
    ap.add_argument("--map-id", type=int, default=0)
    ap.add_argument("--count", type=int, default=50, help="Number of pairs to generate")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--require-terrain", action="store_true", help="Only sample tiles that have both .mmtile and .map")
    ap.add_argument("--out", required=True, help="Output orderings .lua path")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    data_dir = Path(args.data_dir)
    mmaps = data_dir / "mmaps"
    maps = data_dir / "maps"
    if not mmaps.is_dir() or not maps.is_dir():
        raise SystemExit("data-dir must contain mmaps/ and maps/")

    mid = int(args.map_id)
    want = f"{mid:03d}"

    nav_tiles: List[Tuple[int, int]] = []
    for p in mmaps.iterdir():
        if not p.is_file():
            continue
        m = MMTILE_RE.match(p.name)
        if not m or m.group("map") != want:
            continue
        nav_tiles.append((int(m.group("tx")), int(m.group("ty"))))

    if not nav_tiles:
        raise SystemExit(f"no .mmtile files found for map {mid}")

    terrain_set = set()
    for p in maps.iterdir():
        if not p.is_file():
            continue
        m = MAP_RE.match(p.name)
        if not m or m.group("map") != want:
            continue
        terrain_set.add((int(m.group("tx")), int(m.group("ty"))))

    if args.require_terrain:
        nav_tiles = [t for t in nav_tiles if t in terrain_set]
        if not nav_tiles:
            raise SystemExit("no tiles with both nav+terrain found")

    rnd = random.Random(int(args.seed))
    rnd.shuffle(nav_tiles)
    picks = nav_tiles[: max(1, min(int(args.count), len(nav_tiles)))]

    pairs = []
    for (tx, ty) in picks:
        x, y = tile_center_world_xy(tx, ty)
        # Small delta staying inside the same tile (<< BLOCK_SIZE)
        x2, y2 = x + 10.0, y + 10.0
        pairs.append((x, y, x2, y2))

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    lines.append('["orderings"] = {')
    lines.append("  {")
    lines.append("    0,")
    lines.append("    { 0 },")
    lines.append("    {")
    for (x1, y1, x2, y2) in pairs:
        lines.append(f"      {{ {int(x1)}, {int(y1)}, {int(x2)}, {int(y2)} }},")
    lines.append("    },")
    lines.append("  },")
    lines.append("}")
    out.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")
    print(f"Wrote {len(pairs)} pairs to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

