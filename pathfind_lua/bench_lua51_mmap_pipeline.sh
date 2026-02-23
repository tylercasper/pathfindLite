#!/bin/bash
# bench_lua51_mmap_pipeline.sh
#
# End-to-end confirmation pipeline:
#  1) Record which .mmap/.mmtile/.map files are required for the given orderings
#     by running PathFinder with the original on-disk loader.
#  2) Convert ONLY those required files into the sharded Lua-blob addon format.
#  3) Compare computeDistance results between the original loader and the Lua loader.
#
# Usage:
#   ./bench_lua51_mmap_pipeline.sh <origDataDir> <mapId> [orderingsFile] [addonsOutDir] [addonPrefix]
#
# Notes:
# - Requires: lua (5.1) and python on PATH.
# - Designed for bash (macOS/Linux/Git-Bash).

set -euo pipefail

SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do
  DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done
cd "$(cd -P "$(dirname "$SOURCE")" && pwd)"

ORIG_DATA_DIR="${1:-}"
MAP_ID="${2:-0}"
ORDERINGS_FILE="${3:-orderings.lua}"
ADDONS_OUT_DIR="${4:-./generated_addons}"
ADDON_PREFIX="${5:-qhstub_mmapdata}"

if [ -z "$ORIG_DATA_DIR" ]; then
  echo "Usage: $0 <origDataDir> <mapId> [orderingsFile] [addonsOutDir] [addonPrefix]" 1>&2
  exit 2
fi

LUA=(lua)

echo "=== Step 1: record required files ==="
"${LUA[@]}" bench_mmap_record.lua "$ORDERINGS_FILE" "$ORIG_DATA_DIR" "$MAP_ID" \
  --present-out required_present.txt --missing-out required_missing.txt

echo ""
echo "=== Step 2: convert required files to Lua blobs ==="
python tools/convert_mmaps_to_lua.py \
  --input-data-dir "$ORIG_DATA_DIR" \
  --output-addons-dir "$ADDONS_OUT_DIR" \
  --addon-prefix "$ADDON_PREFIX" \
  --manifest required_present.txt

echo ""
echo "=== Step 3: compare original vs lua loader ==="
"${LUA[@]}" bench_mmap_compare.lua "$ORDERINGS_FILE" "$ORIG_DATA_DIR" "$MAP_ID" "$ADDONS_OUT_DIR" \
  --addon-prefix "$ADDON_PREFIX"

echo ""
echo "=== OK: loaders match ==="

