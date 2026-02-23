-- bench_mmap_manifest.lua
-- Computes the set of required data files for a coordinate-pair workload
-- WITHOUT loading navmesh or terrain data into memory.
--
-- This avoids OOM in cases where the full benchmark would load too many tiles at once.
--
-- Usage:
--   lua bench_mmap_manifest.lua [orderingsFile] [dataDir] [mapId] [flags...]
--
-- Flags:
--   --present-out <file>   output manifest of existing required files (default: required_present.txt)
--   --missing-out <file>   output manifest of missing required files (default: required_missing.txt)
--
-- Output format:
--   One relative path per line (relative to dataDir), using forward slashes.

local function die(msg)
  io.stderr:write("ERROR: " .. tostring(msg) .. "\n")
  os.exit(1)
end

local function parse_args(argv)
  local flags = {}
  local positional = {}
  local i = 1
  while argv and i <= #argv do
    local a = argv[i]
    if a == "--present-out" then
      flags.present_out = argv[i + 1]
      i = i + 2
    elseif a == "--missing-out" then
      flags.missing_out = argv[i + 1]
      i = i + 2
    else
      positional[#positional + 1] = a
      i = i + 1
    end
  end
  return positional, flags
end

local positional, flags = parse_args(arg or {})

local orderings_file = positional[1] or "orderings.json"
local DATA_DIR       = positional[2] or "."
local MAP_ID         = positional[3] and tonumber(positional[3]) or 0

local present_out = flags.present_out or "required_present.txt"
local missing_out = flags.missing_out or "required_missing.txt"

-- Must match PathFinder.lua worldToTile()
local BLOCK_SIZE  = 533.33333
local TILE_ORIGIN = 32
local WORLD_MAX   = 17066.666

local function norm(p)
  return (tostring(p or "")):gsub("\\", "/")
end

local function join_path(a, b)
  a = norm(a)
  b = norm(b)
  if a == "" then return b end
  if a:sub(-1) == "/" then return a .. b end
  return a .. "/" .. b
end

local function file_exists(path)
  local f = io.open(path, "rb")
  if f then f:close(); return true end
  return false
end

local function worldToTile(x, y)
  local ftx = TILE_ORIGIN - x / BLOCK_SIZE
  local fty = TILE_ORIGIN - y / BLOCK_SIZE
  if ftx < 0 or ftx >= 64 or fty < 0 or fty >= 64 then
    return nil
  end
  return math.floor(ftx), math.floor(fty)
end

local function add(set, rel)
  set[rel] = true
end

-- Parse input file (same as bench.lua)
local f, err = io.open(orderings_file, "r")
if not f then die("opening " .. orderings_file .. ": " .. tostring(err)) end
local src = f:read("*a")
f:close()

local chunk, cerr = loadstring("return {" .. src .. "}")
if not chunk then die("parsing file: " .. tostring(cerr)) end
local data = chunk()
local orderings = data["orderings"]
if type(orderings) ~= "table" then die("missing [\"orderings\"] table") end

local pairs_list = {}
for _, ordering in ipairs(orderings) do
  local coord_sets = ordering[3]
  if coord_sets then
    for _, cs in ipairs(coord_sets) do
      pairs_list[#pairs_list + 1] = {
        math.floor(cs[1]),
        math.floor(cs[2]),
        math.floor(cs[3]),
        math.floor(cs[4]),
      }
    end
  end
end
if #pairs_list == 0 then die("no coordinate pairs found in file") end

local required = {}
add(required, string.format("mmaps/%03d.mmap", MAP_ID))

local nav_tiles = {}
local terr_tiles = {}

for _, p in ipairs(pairs_list) do
  local x1, y1, x2, y2 = p[1], p[2], p[3], p[4]
  if math.abs(x1) > WORLD_MAX or math.abs(y1) > WORLD_MAX or math.abs(x2) > WORLD_MAX or math.abs(y2) > WORLD_MAX then
    -- PathFinder would reject; skip
  else
    local tx1, ty1 = worldToTile(x1, y1)
    local tx2, ty2 = worldToTile(x2, y2)
    if tx1 and tx2 then
      local txMin = math.min(tx1, tx2)
      local txMax = math.max(tx1, tx2)
      local tyMin = math.min(ty1, ty2)
      local tyMax = math.max(ty1, ty2)
      for tx = txMin, txMax do
        for ty = tyMin, tyMax do
          nav_tiles[tx * 64 + ty] = { tx = tx, ty = ty }
        end
      end
    end
    if tx1 then terr_tiles[tx1 * 64 + ty1] = { tx = tx1, ty = ty1 } end
    if tx2 then terr_tiles[tx2 * 64 + ty2] = { tx = tx2, ty = ty2 } end
  end
end

for _, t in pairs(nav_tiles) do
  add(required, string.format("mmaps/%03d%02d%02d.mmtile", MAP_ID, t.tx, t.ty))
end
for _, t in pairs(terr_tiles) do
  add(required, string.format("maps/%03d%02d%02d.map", MAP_ID, t.tx, t.ty))
end

local present, missing = {}, {}
for rel in pairs(required) do
  local abs = join_path(DATA_DIR, rel)
  if file_exists(abs) then
    present[rel] = true
  else
    missing[rel] = true
  end
end

local function write_manifest(path, set)
  local keys = {}
  for k in pairs(set or {}) do keys[#keys + 1] = k end
  table.sort(keys)
  local out, e = io.open(path, "w")
  if not out then die("writing manifest " .. tostring(path) .. ": " .. tostring(e)) end
  for _, k in ipairs(keys) do out:write(k, "\n") end
  out:close()
  return #keys
end

local n_present = write_manifest(present_out, present)
local n_missing = write_manifest(missing_out, missing)

print(string.format("File:       %s", orderings_file))
print(string.format("Data dir:    %s  map %d", DATA_DIR, MAP_ID))
print(string.format("Pairs:       %d", #pairs_list))
print(string.format("Nav tiles:   %d", (function() local c=0; for _ in pairs(nav_tiles) do c=c+1 end; return c end)()))
print(string.format("Terr tiles:  %d", (function() local c=0; for _ in pairs(terr_tiles) do c=c+1 end; return c end)()))
print(string.format("Present:     %d files -> %s", n_present, present_out))
print(string.format("Missing:     %d files -> %s", n_missing, missing_out))

