## MmapLuaDB format (QuestHelper-style “static” blobs for mmaps/maps)

This format stores the same raw bytes that `MmapFileLoader.lua` would read from disk:

- `%03d.mmap` (navmesh params): **first 28 bytes only**
- `%03d%02d%02d.mmtile` (navmesh tile): full file bytes
- `%03d%02d%02d.map` (terrain tile): full file bytes (may be missing for some tiles)

The bytes are stored inside Lua addon files as **compressed binary strings**, with a compact index that lets the loader extract a single tile’s compressed blob without loading/decompressing everything.

### Core addon

- **Addon folder / name**: `${addon_prefix}` (default `qhstub_mmapdata`)
- **Non-LoadOnDemand** (always enabled/loaded)
- Provides:
  - `MmapLuaDB.config`
  - `MmapLuaDB.params[mapId]` (28-byte strings)

Core global:

- `MmapLuaDB` (table)
- `MmapLuaDB.config` (table):
  - `format_version` (number, currently `1`)
  - `shard_dim` (number, e.g. `8`)
  - `addon_prefix` (string, e.g. `"qhstub_mmapdata"`)
  - `interface_version` (number, e.g. `30300`)
- `MmapLuaDB.params` (table): `params[mapId] = <28-byte string>`
- `MmapLuaDB.shards` (table): created empty in core; populated by shard addons

### Shard addons (LoadOnDemand)

Each shard addon contains data for one `(mapId, sx, sy)` shard.

- **Shard size**: `SHARD_DIM = MmapLuaDB.config.shard_dim`
- **Tile grid**: `tx,ty ∈ [0..63]`
- **Shard coords**:
  - `sx = floor(tx / SHARD_DIM)`
  - `sy = floor(ty / SHARD_DIM)`

**Addon folder / name**:

`${addon_prefix}_${mapId3}_${sx2}_${sy2}`

- `mapId3` is 3 digits, zero-padded (`%03d`)
- `sx2`, `sy2` are 2 digits, zero-padded (`%02d`)

Example: `qhstub_mmapdata_000_03_07`

Each shard addon executes Lua that populates:

`MmapLuaDB.shards[mapId][sx][sy] = shardTable`

where `shardTable` is:

- `nav` (table):
  - `serialize_index` (binary string)
  - `serialize_data` (binary string)
  - `count` (number of tiles in this shard for nav)
- `terrain` (table): same fields as `nav`

### Tile key encoding

Within a map, a tile is addressed by `(tx, ty)` and encoded as a single integer key:

`tileKey = tx * 64 + ty + 1`

`+1` ensures `tileKey > 0` because `0` is reserved as a sentinel in the index encoding.

### Compression

Each tile’s raw bytes are compressed independently as a **raw DEFLATE stream** (RFC 1951; no zlib headers).

- **Decompress** with `LibDeflate:DecompressDeflate(compressedBytes)`

### `serialize_data` layout

For each shard and type (`nav`/`terrain`):

- `serialize_data` is a single concatenated binary string of *per-tile compressed blobs*.
- For each stored tile, the index provides:
  - `ofs` (1-based byte offset into `serialize_data`)
  - `lenMinus1` (byte length minus 1)

The compressed blob bytes are:

`serialize_data:sub(ofs, ofs + lenMinus1)`

### `serialize_index` encoding (QuestHelper-style BST + varints)

`serialize_index` encodes a binary search tree (BST) of `(tileKey -> (ofs,lenMinus1))` nodes.

- Integers are encoded as a base-128 varint (“adaptint”):
  - Each byte stores 7 bits of payload in bits 1..7 (`payload = floor(byte/2)`).
  - Bit 0 is the continuation flag (`1` = more bytes follow, `0` = last byte).
- A null node is encoded as a single adaptint integer `0`.

Node encoding (pre-order):

- `tileKey`
- `ofs`
- `lenMinus1`
- `rlink` (byte length of the **entire left subtree encoding**)
- `<left subtree bytes>`
- `<right subtree bytes>`

Lookup algorithm (matches `qhstub_db.lua`’s `search_index` behavior):

- Read `tileKey` at current cursor
  - If `tileKey == 0`: missing / end
- Read `ofs`, `lenMinus1`, `rlink`
- If `tileKey == wanted`: return `serialize_data:sub(ofs, ofs+lenMinus1)`
- If `tileKey < wanted`: skip `rlink` bytes to jump over left subtree to right subtree
- Else (wanted < tileKey): continue into left subtree (no skip)

### Lua source encoding (binary `.lua`)

To keep files compact, `serialize_index` and `serialize_data` are stored in Lua short string literals that may contain arbitrary bytes (like QuestHelper `static_*.lua`). The generator escapes only:

- `\\0` as `\\000`
- `\\n` as `\\n`
- `\\r` as `\\r`
- `\"` as `\\\"`
- `\\` as `\\\\`

And uses Lua’s `\\` + newline line-continuation escape to wrap long literals across lines without concatenation.

