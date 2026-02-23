-- bench_mmap_bytes_compare.lua
-- Verifies that MmapLuaLoader returns the exact same raw bytes as MmapFileLoader
-- for a given manifest of required files.
--
-- Usage:
--   lua bench_mmap_bytes_compare.lua <manifest> <origDataDir> <addonsDir> [--addon-prefix <prefix>]
--
-- Manifest format:
--   One relative path per line (relative to origDataDir), using forward slashes:
--     mmaps/000.mmap
--     mmaps/0002239.mmtile
--     maps/0002239.map
--
-- Exits non-zero on mismatch.

local function die(msg)
  io.stderr:write("ERROR: " .. tostring(msg) .. "\n")
  os.exit(1)
end

local function read_lines(path)
  local f, err = io.open(path, "r")
  if not f then die("opening manifest " .. tostring(path) .. ": " .. tostring(err)) end
  local out = {}
  while true do
    local l = f:read("*l")
    if not l then break end
    l = (l:gsub("\r", "")):match("^%s*(.-)%s*$")
    if l ~= "" and l:sub(1,1) ~= "#" then out[#out + 1] = l end
  end
  f:close()
  return out
end

local function parse_args(argv)
  local flags = {}
  local positional = {}
  local i = 1
  while argv and i <= #argv do
    local a = argv[i]
    if a == "--addon-prefix" then
      flags.addon_prefix = argv[i + 1]
      i = i + 2
    else
      positional[#positional + 1] = a
      i = i + 1
    end
  end
  return positional, flags
end

local positional, flags = parse_args(arg or {})

local manifest = positional[1]
local origDir  = positional[2]
local addonsDir = positional[3]
if not manifest or not origDir or not addonsDir then
  die("Usage: lua bench_mmap_bytes_compare.lua <manifest> <origDataDir> <addonsDir> [--addon-prefix <prefix>]")
end

local addon_prefix = flags.addon_prefix or "qhstub_mmapdata"

local fileLoader = require("MmapFileLoader")
local luaLoader  = require("MmapLuaLoader")

_G.MmapLuaAddonPrefix = addon_prefix
_G.MmapLuaDB = nil

local lines = read_lines(manifest)
if #lines == 0 then die("manifest is empty") end

local function parse_rel(rel)
  local mm = rel:match("^mmaps/(%d%d%d)%.mmap$")
  if mm then
    return "mmap", tonumber(mm)
  end
  local map, tx, ty = rel:match("^mmaps/(%d%d%d)(%d%d)(%d%d)%.mmtile$")
  if map then
    return "mmtile", tonumber(map), tonumber(tx), tonumber(ty)
  end
  map, tx, ty = rel:match("^maps/(%d%d%d)(%d%d)(%d%d)%.map$")
  if map then
    return "map", tonumber(map), tonumber(tx), tonumber(ty)
  end
  return nil
end

local mismatches = 0
local checked = 0

for _, rel in ipairs(lines) do
  local kind, mapId, tx, ty = parse_rel(rel)
  if not kind then
    die("unrecognized manifest line: " .. tostring(rel))
  end

  local a, b
  if kind == "mmap" then
    a = (select(1, fileLoader.loadNavMeshParams(origDir, mapId)))
    b = (select(1, luaLoader.loadNavMeshParams(addonsDir, mapId)))
  elseif kind == "mmtile" then
    a = (select(1, fileLoader.loadNavTile(origDir, mapId, tx, ty)))
    b = (select(1, luaLoader.loadNavTile(addonsDir, mapId, tx, ty)))
  elseif kind == "map" then
    a = (select(1, fileLoader.loadTerrainTile(origDir, mapId, tx, ty)))
    b = (select(1, luaLoader.loadTerrainTile(addonsDir, mapId, tx, ty)))
  end

  checked = checked + 1
  if type(a) ~= "string" or type(b) ~= "string" or #a ~= #b or a ~= b then
    mismatches = mismatches + 1
    if mismatches <= 10 then
      print(string.format("MISMATCH %s  orig=%s(%s) lua=%s(%s)",
        rel,
        type(a), type(a) == "string" and #a or "nil",
        type(b), type(b) == "string" and #b or "nil"
      ))
    end
  end
end

print(string.format("Checked:    %d", checked))
print(string.format("Mismatches: %d", mismatches))
if mismatches > 0 then os.exit(1) end

