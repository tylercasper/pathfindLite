#!/usr/bin/env python3
"""
convert_mmaps_to_lua.py

Convert a CMaNGOS-style dataset:
  <input>/mmaps/*.mmap, *.mmtile
  <input>/maps/*.map

into a set of WoW addon folders that store the same bytes as compressed Lua
"static blobs" (analogous to QuestHelper static_*.lua), sharded for on-demand loading.

Output structure (under --output-addons-dir):

  <addon_prefix>/
    <addon_prefix>.toc
    LibDeflate.lua              (copied, optional but recommended)
    MmapLuaDB_Core.lua          (config + 28-byte mmap params per map)

  <addon_prefix>_<mapId3>_<sx2>_<sy2>/   (LoadOnDemand shard addon)
    <addon_prefix>_<mapId3>_<sx2>_<sy2>.toc
    data.lua                    (nav + terrain indexed compressed stores)

Compression:
  - raw DEFLATE stream (RFC1951, wbits=-15)
  - decompressed in Lua with LibDeflate:DecompressDeflate()

Indexing:
  - QuestHelper-style BST index using "adaptint" varints (see qhstub_db.lua search_index)
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


MMAP_RE = re.compile(r"^(?P<map>\d{3})\.mmap$")
MMTILE_RE = re.compile(r"^(?P<map>\d{3})(?P<tx>\d{2})(?P<ty>\d{2})\.mmtile$")
MMTIL_RE = re.compile(r"^(?P<map>\d{3})(?P<tx>\d{2})(?P<ty>\d{2})\.mmtil$")  # tolerate typo
MAP_RE = re.compile(r"^(?P<map>\d{3})(?P<tx>\d{2})(?P<ty>\d{2})\.map$")


def tile_key(tx: int, ty: int) -> int:
    # Must be >0 because 0 is sentinel in the BST index encoding.
    return tx * 64 + ty + 1


def encode_adaptint(n: int) -> bytes:
    # Base-128 little-endian varint with continuation flag in LSB.
    # Payload is stored in bits 1..7 as floor(byte/2).
    if n < 0:
        raise ValueError(f"adaptint cannot encode negative: {n}")
    out = bytearray()
    while True:
        payload = n % 128
        n //= 128
        b = payload * 2
        if n != 0:
            b += 1
        out.append(b)
        if n == 0:
            break
    return bytes(out)


def build_bst_index(entries: Dict[int, Tuple[int, int]]) -> bytes:
    # entries: key -> (ofs, lenMinus1), where ofs is 1-based into serialize_data
    keys = sorted(entries.keys())

    def rec(klist: List[int]) -> bytes:
        if not klist:
            return encode_adaptint(0)
        mid = len(klist) // 2
        k = klist[mid]
        left = rec(klist[:mid])
        right = rec(klist[mid + 1 :])
        ofs, lenm1 = entries[k]
        rlink = len(left)
        return (
            encode_adaptint(k)
            + encode_adaptint(ofs)
            + encode_adaptint(lenm1)
            + encode_adaptint(rlink)
            + left
            + right
        )

    return rec(keys)


def deflate_raw(data: bytes, level: int) -> bytes:
    # Raw DEFLATE (wbits=-15) to match LibDeflate:DecompressDeflate()
    c = zlib.compressobj(level=level, wbits=-15)
    return c.compress(data) + c.flush()


def lua_escape_byte(b: int, safe_ascii: bool) -> bytes:
    # Escape bytes for inclusion in a Lua short string literal.
    #
    # By default (safe_ascii=False), this matches QuestHelper-style blobs:
    # most bytes are written raw into the .lua file.
    #
    # On some Windows Lua builds, raw 0x1A (Ctrl+Z) can behave like EOF in text mode.
    # When safe_ascii=True, we avoid raw control/high bytes by using \ddd escapes.
    if b == 34:
        return b'\\"'
    if b == 92:
        return b"\\\\"
    if b == 10:
        return b"\\n"
    if b == 13:
        return b"\\r"
    if b == 0:
        return b"\\000"

    if safe_ascii:
        if b < 32 or b >= 127:
            return ("\\%03d" % b).encode("ascii")
        return bytes([b])

    # raw mode
    if b == 0x1A:
        # Still escape Ctrl+Z even in raw mode for external-friendliness.
        return b"\\026"
    return bytes([b])


def write_lua_short_string_literal(
    fp, data: bytes, wrap_bytes: int = 120, safe_ascii: bool = False, cont_indent: bytes = b""
) -> None:
    # Writes a Lua expression that evaluates to the byte string.
    #
    # IMPORTANT: Do NOT use backslash-newline inside string literals.
    # In Lua 5.1, "A\<newline>B" inserts an actual newline byte (0x0A), corrupting binary streams.
    #
    # We wrap by emitting concatenated short literals:
    #   "part1" ..
    #   "part2" ..
    #   "part3"
    if wrap_bytes <= 0 or len(data) <= wrap_bytes:
        fp.write(b'"')
        for bb in data:
            fp.write(lua_escape_byte(bb, safe_ascii=safe_ascii))
        fp.write(b'"')
        return

    # Use table.concat({ "chunk", "chunk", ... }) to avoid:
    # - inserting bytes (no backslash-newline tricks)
    # - Lua parser "too many syntax levels" from a huge .. chain
    fp.write(b"table.concat({\n")
    n = len(data)
    i = 0
    while i < n:
        j = min(i + wrap_bytes, n)
        if cont_indent:
            fp.write(cont_indent)
        fp.write(b"\t\"")
        for bb in data[i:j]:
            fp.write(lua_escape_byte(bb, safe_ascii=safe_ascii))
        fp.write(b"\",\n")
        i = j
    if cont_indent:
        fp.write(cont_indent)
    fp.write(b"})")


def write_lua_kv_string(
    fp, key: str, data: bytes, indent: str = "\t", wrap_bytes: int = 120, safe_ascii: bool = False
) -> None:
    fp.write(indent.encode("ascii"))
    fp.write(key.encode("ascii"))
    fp.write(b" = ")
    ind = indent.encode("ascii")
    write_lua_short_string_literal(fp, data, wrap_bytes=wrap_bytes, safe_ascii=safe_ascii, cont_indent=ind)
    fp.write(b",\n")


def write_text_file(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.replace("\r\n", "\n").replace("\r", "\n"), encoding="utf-8", newline="\n")


def copy_file(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    with src.open("rb") as fsrc, dst.open("wb") as fdst:
        while True:
            chunk = fsrc.read(1024 * 1024)
            if not chunk:
                break
            fdst.write(chunk)


def find_default_libdeflate_src() -> Optional[Path]:
    # Try to locate the LibDeflate.lua that exists in this workspace layout.
    here = Path(__file__).resolve()
    for p in [here] + list(here.parents):
        cand = p / "AddOns" / "Details" / "Libs" / "LibDeflate" / "LibDeflate.lua"
        if cand.is_file():
            return cand
    return None


@dataclass(frozen=True)
class TileRef:
    map_id: int
    tx: int
    ty: int
    path: Path

    @property
    def key(self) -> int:
        return tile_key(self.tx, self.ty)

    @property
    def shard_xy(self) -> Tuple[int, int]:
        raise RuntimeError("shard_xy requires shard_dim; compute externally")


def parse_args(argv: List[str]) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Convert .mmap/.mmtile/.map dataset to sharded Lua blob addons.")
    ap.add_argument("--input-data-dir", required=True, help="Directory containing mmaps/ and maps/ subfolders.")
    ap.add_argument("--output-addons-dir", required=True, help="Directory where addon folders will be created.")
    ap.add_argument("--addon-prefix", default="qhstub_mmapdata", help="Core addon folder/name prefix.")
    ap.add_argument("--shard-dim", type=int, default=8, help="Shard dimension in tiles (default 8).")
    ap.add_argument("--interface", type=int, default=30300, help="WoW Interface number for generated .toc files.")
    ap.add_argument("--compression-level", type=int, default=6, help="zlib compression level 0..9 (raw deflate).")
    ap.add_argument(
        "--manifest",
        default="",
        help="Optional manifest (one relative path per line) to convert only a subset. "
        "Paths are relative to input-data-dir, e.g. mmaps/000.mmap, mmaps/0000000.mmtile, maps/0000000.map.",
    )
    ap.add_argument(
        "--lua-safe-ascii",
        action="store_true",
        help="Escape control/high bytes so generated .lua files are ASCII-safe "
        "(slower/larger, but avoids Ctrl+Z issues in some Windows Lua interpreters).",
    )
    ap.add_argument(
        "--libdeflate-src",
        default="",
        help="Path to LibDeflate.lua to copy into the core addon. If empty, script tries to auto-detect.",
    )
    ap.add_argument(
        "--no-copy-libdeflate",
        action="store_true",
        help="Do not copy LibDeflate.lua into the core addon (requires LibDeflate to exist at runtime).",
    )
    ap.add_argument(
        "--map-ids",
        default="",
        help="Optional comma-separated list of map IDs to include (e.g. 0,1,530). If empty, includes all found.",
    )
    return ap.parse_args(argv)


def main(argv: List[str]) -> int:
    args = parse_args(argv)

    input_dir = Path(args.input_data_dir)
    output_dir = Path(args.output_addons_dir)
    addon_prefix = str(args.addon_prefix)
    shard_dim = int(args.shard_dim)
    interface = int(args.interface)
    level = int(args.compression_level)
    safe_ascii = bool(args.lua_safe_ascii)

    if shard_dim <= 0 or shard_dim > 64:
        raise SystemExit("--shard-dim must be in 1..64")
    if level < 0 or level > 9:
        raise SystemExit("--compression-level must be in 0..9")

    mmaps_dir = input_dir / "mmaps"
    maps_dir = input_dir / "maps"
    if not mmaps_dir.is_dir():
        raise SystemExit(f"missing directory: {mmaps_dir}")
    if not maps_dir.is_dir():
        raise SystemExit(f"missing directory: {maps_dir}")

    restrict_maps: Optional[set[int]] = None
    if args.map_ids:
        restrict_maps = {int(x.strip()) for x in args.map_ids.split(",") if x.strip() != ""}

    # ---------------------------------------------------------------------
    # Scan input dataset
    # ---------------------------------------------------------------------
    params_by_map: Dict[int, bytes] = {}
    nav_tiles: Dict[Tuple[int, int, int], Dict[int, Path]] = {}      # (map,sx,sy) -> {tileKey: path}
    terr_tiles: Dict[Tuple[int, int, int], Dict[int, Path]] = {}     # (map,sx,sy) -> {tileKey: path}
    maps_seen: set[int] = set()

    if args.manifest:
        manifest_path = Path(args.manifest)
        if not manifest_path.is_file():
            raise SystemExit(f"--manifest not found: {manifest_path}")
        lines = manifest_path.read_text(encoding="utf-8", errors="replace").splitlines()

        def resolve_manifest_path(line: str) -> Optional[Path]:
            s = line.strip()
            if not s or s.startswith("#"):
                return None
            s = s.replace("\\", "/")
            p = Path(s)
            if p.is_absolute():
                return p
            return input_dir / p

        for raw in lines:
            p = resolve_manifest_path(raw)
            if not p:
                continue
            # Normalize to a real file path; skip non-existent entries quietly.
            if not p.is_file():
                continue

            parent = p.parent.name
            name = p.name

            if parent == "mmaps":
                m = MMAP_RE.match(name)
                if m:
                    map_id = int(m.group("map"))
                    if restrict_maps is not None and map_id not in restrict_maps:
                        continue
                    maps_seen.add(map_id)
                    head = p.read_bytes()[:28]
                    if len(head) < 28:
                        print(f"WARNING: {p} is too short for dtNavMeshParams (need 28 bytes)", file=sys.stderr)
                        continue
                    params_by_map[map_id] = head
                    continue

                m = MMTILE_RE.match(name) or MMTIL_RE.match(name)
                if m:
                    map_id = int(m.group("map"))
                    if restrict_maps is not None and map_id not in restrict_maps:
                        continue
                    tx = int(m.group("tx"))
                    ty = int(m.group("ty"))
                    maps_seen.add(map_id)
                    sx = tx // shard_dim
                    sy = ty // shard_dim
                    shard = (map_id, sx, sy)
                    nav_tiles.setdefault(shard, {})[tile_key(tx, ty)] = p
                    continue

            if parent == "maps":
                m = MAP_RE.match(name)
                if m:
                    map_id = int(m.group("map"))
                    if restrict_maps is not None and map_id not in restrict_maps:
                        continue
                    tx = int(m.group("tx"))
                    ty = int(m.group("ty"))
                    maps_seen.add(map_id)
                    sx = tx // shard_dim
                    sy = ty // shard_dim
                    shard = (map_id, sx, sy)
                    terr_tiles.setdefault(shard, {})[tile_key(tx, ty)] = p
                    continue

    else:
        # .mmap params files
        for ent in mmaps_dir.iterdir():
            if not ent.is_file():
                continue
            m = MMAP_RE.match(ent.name)
            if not m:
                continue
            map_id = int(m.group("map"))
            if restrict_maps is not None and map_id not in restrict_maps:
                continue
            maps_seen.add(map_id)
            with ent.open("rb") as f:
                head = f.read(28)
            if len(head) < 28:
                print(f"WARNING: {ent} is too short for dtNavMeshParams (need 28 bytes)", file=sys.stderr)
                continue
            params_by_map[map_id] = head

        # .mmtile nav tiles
        for ent in mmaps_dir.iterdir():
            if not ent.is_file():
                continue
            m = MMTILE_RE.match(ent.name) or MMTIL_RE.match(ent.name)
            if not m:
                continue
            map_id = int(m.group("map"))
            if restrict_maps is not None and map_id not in restrict_maps:
                continue
            tx = int(m.group("tx"))
            ty = int(m.group("ty"))
            maps_seen.add(map_id)
            sx = tx // shard_dim
            sy = ty // shard_dim
            shard = (map_id, sx, sy)
            nav_tiles.setdefault(shard, {})[tile_key(tx, ty)] = ent

        # .map terrain tiles
        for ent in maps_dir.iterdir():
            if not ent.is_file():
                continue
            m = MAP_RE.match(ent.name)
            if not m:
                continue
            map_id = int(m.group("map"))
            if restrict_maps is not None and map_id not in restrict_maps:
                continue
            tx = int(m.group("tx"))
            ty = int(m.group("ty"))
            maps_seen.add(map_id)
            sx = tx // shard_dim
            sy = ty // shard_dim
            shard = (map_id, sx, sy)
            terr_tiles.setdefault(shard, {})[tile_key(tx, ty)] = ent

    if not maps_seen:
        print("No maps found. Expected files like mmaps/000.mmap and mmaps/0000000.mmtile", file=sys.stderr)
        return 2

    missing_params = sorted([m for m in maps_seen if m not in params_by_map])
    if missing_params:
        raise SystemExit(f"Missing .mmap params for map IDs: {', '.join(map(str, missing_params))}")

    # ---------------------------------------------------------------------
    # Generate core addon
    # ---------------------------------------------------------------------
    core_dir = output_dir / addon_prefix
    core_toc = core_dir / f"{addon_prefix}.toc"
    core_libdeflate_dst = core_dir / "LibDeflate.lua"
    core_db = core_dir / "MmapLuaDB_Core.lua"

    core_dir.mkdir(parents=True, exist_ok=True)

    # LibDeflate copy (optional but recommended)
    if not args.no_copy_libdeflate:
        src = Path(args.libdeflate_src) if args.libdeflate_src else find_default_libdeflate_src()
        if not src or not src.is_file():
            raise SystemExit(
                "LibDeflate.lua not found. Provide --libdeflate-src or use --no-copy-libdeflate."
            )
        copy_file(src, core_libdeflate_dst)

    # Core .toc
    core_files = []
    if not args.no_copy_libdeflate:
        core_files.append("LibDeflate.lua")
    core_files.append("MmapLuaDB_Core.lua")
    core_toc_text = "\n".join(
        [
            f"## Interface: {interface}",
            f"## Title: {addon_prefix}",
            "## Notes: Sharded mmap/mmtile/map blob store (generated).",
            "## Version: 1",
            "",
            *core_files,
            "",
        ]
    )
    write_text_file(core_toc, core_toc_text)

    # Core DB file (binary Lua)
    with core_db.open("wb") as fp:
        fp.write(b"-- MmapLuaDB_Core.lua (generated)\n")
        fp.write(b"MmapLuaDB = MmapLuaDB or {}\n")
        fp.write(b"MmapLuaDB.config = MmapLuaDB.config or {}\n")
        fp.write(b"MmapLuaDB.config.format_version = 1\n")
        fp.write(f"MmapLuaDB.config.shard_dim = {shard_dim}\n".encode("ascii"))
        fp.write(f'MmapLuaDB.config.addon_prefix = "{addon_prefix}"\n'.encode("ascii"))
        fp.write(f"MmapLuaDB.config.interface_version = {interface}\n".encode("ascii"))
        fp.write(b"MmapLuaDB.params = MmapLuaDB.params or {}\n")
        fp.write(b"MmapLuaDB.shards = MmapLuaDB.shards or {}\n")
        fp.write(b"\n")

        for map_id in sorted(maps_seen):
            params = params_by_map.get(map_id)
            if not params:
                # No params means PathFinder will be unable to init this map; still record missing.
                continue
            fp.write(f"MmapLuaDB.params[{map_id}] = ".encode("ascii"))
            write_lua_short_string_literal(fp, params, wrap_bytes=120, safe_ascii=safe_ascii, cont_indent=b"")
            fp.write(b"\n")

    # ---------------------------------------------------------------------
    # Generate shard addons
    # ---------------------------------------------------------------------
    # Union of shard keys across nav and terrain
    shard_keys = set(nav_tiles.keys()) | set(terr_tiles.keys())
    shard_keys = {k for k in shard_keys if (restrict_maps is None or k[0] in restrict_maps)}

    def shard_addon_name(map_id: int, sx: int, sy: int) -> str:
        return f"{addon_prefix}_{map_id:03d}_{sx:02d}_{sy:02d}"

    for (map_id, sx, sy) in sorted(shard_keys):
        addon_name = shard_addon_name(map_id, sx, sy)
        shard_dir = output_dir / addon_name
        shard_dir.mkdir(parents=True, exist_ok=True)

        # Shard .toc (LoadOnDemand)
        toc_path = shard_dir / f"{addon_name}.toc"
        toc_text = "\n".join(
            [
                f"## Interface: {interface}",
                f"## Title: {addon_name}",
                "## Notes: mmap shard blob (generated).",
                "## Version: 1",
                "## LoadOnDemand: 1",
                f"## Dependencies: {addon_prefix}",
                "",
                "data.lua",
                "",
            ]
        )
        write_text_file(toc_path, toc_text)

        # Build stores for nav and terrain
        nav_map = nav_tiles.get((map_id, sx, sy), {})
        terr_map = terr_tiles.get((map_id, sx, sy), {})

        def build_store(tile_paths: Dict[int, Path]) -> Tuple[bytes, bytes, int]:
            # Returns (serialize_index, serialize_data, count)
            if not tile_paths:
                return encode_adaptint(0), b"", 0

            data_blob = bytearray()
            entries: Dict[int, Tuple[int, int]] = {}
            for k in sorted(tile_paths.keys()):
                p = tile_paths[k]
                raw = p.read_bytes()
                comp = deflate_raw(raw, level=level)
                ofs = len(data_blob) + 1  # 1-based for Lua
                data_blob.extend(comp)
                entries[k] = (ofs, len(comp) - 1)
            index_blob = build_bst_index(entries)
            return index_blob, bytes(data_blob), len(tile_paths)

        nav_index, nav_data, nav_count = build_store(nav_map)
        terr_index, terr_data, terr_count = build_store(terr_map)

        # Write shard Lua file (binary)
        lua_path = shard_dir / "data.lua"
        with lua_path.open("wb") as fp:
            fp.write(b"-- data.lua (generated)\n")
            fp.write(b"MmapLuaDB = MmapLuaDB or {}\n")
            fp.write(b"MmapLuaDB.shards = MmapLuaDB.shards or {}\n")
            fp.write(f"local mapId = {map_id}\n".encode("ascii"))
            fp.write(f"local sx = {sx}\n".encode("ascii"))
            fp.write(f"local sy = {sy}\n".encode("ascii"))
            fp.write(b"MmapLuaDB.shards[mapId] = MmapLuaDB.shards[mapId] or {}\n")
            fp.write(b"MmapLuaDB.shards[mapId][sx] = MmapLuaDB.shards[mapId][sx] or {}\n")
            fp.write(b"MmapLuaDB.shards[mapId][sx][sy] = {\n")
            fp.write(b"\tnav = {\n")
            write_lua_kv_string(fp, "serialize_index", nav_index, indent="\t\t", wrap_bytes=120, safe_ascii=safe_ascii)
            write_lua_kv_string(fp, "serialize_data", nav_data, indent="\t\t", wrap_bytes=120, safe_ascii=safe_ascii)
            fp.write(f"\t\tcount = {nav_count},\n".encode("ascii"))
            fp.write(b"\t},\n")
            fp.write(b"\tterrain = {\n")
            write_lua_kv_string(fp, "serialize_index", terr_index, indent="\t\t", wrap_bytes=120, safe_ascii=safe_ascii)
            write_lua_kv_string(fp, "serialize_data", terr_data, indent="\t\t", wrap_bytes=120, safe_ascii=safe_ascii)
            fp.write(f"\t\tcount = {terr_count},\n".encode("ascii"))
            fp.write(b"\t},\n")
            fp.write(b"}\n")

    print(f"Generated core + {len(shard_keys)} shard addons under: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

