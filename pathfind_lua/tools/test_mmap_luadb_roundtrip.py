#!/usr/bin/env python3
"""
test_mmap_luadb_roundtrip.py

Self-test for the sharded Lua blob format produced by convert_mmaps_to_lua.py:
  - Generates a tiny fake dataset (mmaps/maps) with one map and a few tiles
  - Runs convert_mmaps_to_lua.py with a manifest restricting to those files
  - Parses the generated Lua shard file (data.lua) to extract serialize_index/data
  - Performs index lookup and raw-deflate decompression
  - Verifies recovered bytes match the originals

This does NOT require Lua. It's meant to sanity-check the converter/index logic.
"""

from __future__ import annotations

import os
import re
import tempfile
from pathlib import Path
from typing import Dict, Tuple

import zlib


def decode_adaptint(data: bytes, offset: int) -> Tuple[int, int]:
    stx = 0
    acu = 1
    while True:
        if offset >= len(data):
            raise ValueError("adaptint OOB")
        v = data[offset]
        stx += acu * (v // 2)
        offset += 1
        acu *= 128
        if (v % 2) == 0:
            break
    return stx, offset


def search_index(index: bytes, blob: bytes, item: int) -> bytes | None:
    cofs = 0
    while True:
        idx, cofs = decode_adaptint(index, cofs)
        if idx == 0:
            return None
        ofs, cofs = decode_adaptint(index, cofs)
        ln1, cofs = decode_adaptint(index, cofs)
        rlink, cofs = decode_adaptint(index, cofs)
        if idx == item:
            start = ofs - 1
            end = start + ln1 + 1
            return blob[start:end]
        if idx < item:
            cofs += rlink


def lua_unescape_short_string(lit: str) -> bytes:
    # Input is the inside of quotes, after removing the opening/closing quote.
    out = bytearray()
    i = 0
    n = len(lit)
    while i < n:
        ch = lit[i]
        if ch != "\\":
            out.append(ord(ch))
            i += 1
            continue
        # backslash escape
        if i + 1 >= n:
            raise ValueError("dangling backslash")
        nxt = lit[i + 1]
        if nxt == "\n":
            # Lua line continuation: backslash + newline => nothing
            i += 2
            continue
        if nxt == "n":
            out.append(10)
            i += 2
            continue
        if nxt == "r":
            out.append(13)
            i += 2
            continue
        if nxt == "\\":
            out.append(92)
            i += 2
            continue
        if nxt == '"':
            out.append(34)
            i += 2
            continue
        if nxt.isdigit():
            # \000 style (exactly 3 digits in our generator)
            if i + 4 > n:
                raise ValueError("short \\ddd escape")
            ddd = lit[i + 1 : i + 4]
            out.append(int(ddd, 10))
            i += 4
            continue
        # Unknown escape
        raise ValueError(f"unknown escape: \\{nxt!r}")
    return bytes(out)


def extract_lua_string_assignment(src: str, key: str) -> bytes:
    # Find: key = "....",
    # Parse the short string literal with escapes.
    m = re.search(rf"{re.escape(key)}\s*=\s*\"", src)
    if not m:
        raise ValueError(f"missing assignment for {key}")
    i = m.end()  # position after opening quote
    buf = []
    while i < len(src):
        ch = src[i]
        if ch == '"':
            break
        buf.append(ch)
        i += 1
    else:
        raise ValueError("unterminated string literal")
    inner = "".join(buf)
    return lua_unescape_short_string(inner)


def run() -> None:
    here = Path(__file__).resolve()
    convert_py = here.parent / "convert_mmaps_to_lua.py"
    assert convert_py.is_file(), convert_py

    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        input_dir = root / "data"
        (input_dir / "mmaps").mkdir(parents=True, exist_ok=True)
        (input_dir / "maps").mkdir(parents=True, exist_ok=True)

        # Fake files
        map_id = 0
        mmap_path = input_dir / "mmaps" / f"{map_id:03d}.mmap"
        mmap_path.write_bytes(b"A" * 28 + b"EXTRA")

        tiles = {
            (0, 0): b"TILE00" * 10,
            (1, 2): b"TILE12" * 7 + b"\x00\x01\x02\xff",
        }
        maps = {
            (0, 0): b"MAP00" * 9,
        }

        for (tx, ty), raw in tiles.items():
            (input_dir / "mmaps" / f"{map_id:03d}{tx:02d}{ty:02d}.mmtile").write_bytes(raw)
        for (tx, ty), raw in maps.items():
            (input_dir / "maps" / f"{map_id:03d}{tx:02d}{ty:02d}.map").write_bytes(raw)

        # Manifest
        manifest = root / "manifest.txt"
        manifest.write_text(
            "\n".join(
                [
                    f"mmaps/{map_id:03d}.mmap",
                    *(f"mmaps/{map_id:03d}{tx:02d}{ty:02d}.mmtile" for (tx, ty) in tiles.keys()),
                    *(f"maps/{map_id:03d}{tx:02d}{ty:02d}.map" for (tx, ty) in maps.keys()),
                    "",
                ]
            ),
            encoding="utf-8",
            newline="\n",
        )

        out_addons = root / "addons"
        addon_prefix = "qhstub_mmapdata_test"

        cmd = (
            f"python \"{convert_py}\" "
            f"--input-data-dir \"{input_dir}\" "
            f"--output-addons-dir \"{out_addons}\" "
            f"--addon-prefix \"{addon_prefix}\" "
            f"--manifest \"{manifest}\" "
            f"--no-copy-libdeflate"
        )
        rc = os.system(cmd)
        if rc != 0:
            raise RuntimeError(f"converter failed (exit {rc})")

        # Find shard for tiles above with shard_dim default 8 => sx,sy = 0,0 for both
        shard_dir = out_addons / f"{addon_prefix}_{map_id:03d}_00_00"
        data_lua = shard_dir / "data.lua"
        # Read as latin-1 to preserve raw bytes 0..255 in the file.
        src = data_lua.read_text(encoding="latin-1")

        nav_index = extract_lua_string_assignment(src, "serialize_index")
        nav_data = extract_lua_string_assignment(src, "serialize_data")

        for (tx, ty), raw in tiles.items():
            k = tx * 64 + ty + 1
            comp = search_index(nav_index, nav_data, k)
            assert comp is not None, (tx, ty)
            got = zlib.decompress(comp, wbits=-15)
            assert got == raw, (tx, ty)

        terr_index = extract_lua_string_assignment(src, "serialize_index")  # first match is nav; crude
        # Better: slice after "terrain = {" for terrain fields.
        terrain_pos = src.find("terrain")
        assert terrain_pos >= 0
        terr_src = src[terrain_pos:]
        terr_index = extract_lua_string_assignment(terr_src, "serialize_index")
        terr_data = extract_lua_string_assignment(terr_src, "serialize_data")

        for (tx, ty), raw in maps.items():
            k = tx * 64 + ty + 1
            comp = search_index(terr_index, terr_data, k)
            assert comp is not None, (tx, ty)
            got = zlib.decompress(comp, wbits=-15)
            assert got == raw, (tx, ty)

        print("OK: roundtrip converter/index/deflate verified on tiny dataset")


if __name__ == "__main__":
    run()

