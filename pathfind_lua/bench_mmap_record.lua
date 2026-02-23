-- bench_mmap_record.lua
-- Loads a PathFinder over a list of coordinate pairs, but only to determine
-- which .mmap/.mmtile/.map files are required for those pairs.
--
-- Usage:
--   lua bench_mmap_record.lua [orderingsFile] [dataDir] [mapId] [flags...]
--
-- Flags:
--   --present-out <file>   output manifest of existing required files (default: required_present.txt)
--   --missing-out <file>   output manifest of missing attempted files (default: required_missing.txt)
--
-- Output format:
--   One relative path per line (relative to dataDir), using forward slashes.
--
-- This uses MmapRecordingFileLoader via PathFinder's MmapLoaderModule switch.

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
local DATA_DIR       = positional[2] or "/Volumes/Storage/Files/tbc"
local MAP_ID         = positional[3] and tonumber(positional[3]) or 0

local present_out = flags.present_out or "required_present.txt"
local missing_out = flags.missing_out or "required_missing.txt"

-- Suppress [pathfind] debug output during recording
PathFinderDebug = false

-- Parse input file (same format as bench.lua)
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

if #pairs_list == 0 then
  die("no coordinate pairs found in file")
end

-- Clear any prior recording state
_G.MmapRequiredFiles = { present = {}, missing = {}, all = {} }

-- Force PathFinder to use the recording loader
_G.MmapLoaderModule = "MmapRecordingFileLoader"
package.loaded["PathFinder"] = nil

local PathFinder = require("PathFinder")

local pf = PathFinder.new(DATA_DIR, MAP_ID)
if not pf:isValid() then
  die("PathFinder failed to initialise (navmesh params missing?)")
end

-- Record required tiles without running A* (just the tile loads that computeDistance would trigger)
local t0 = os.clock()
for _, p in ipairs(pairs_list) do
  pf:ensureNavTilesLoaded(p[1], p[2], p[3], p[4])
  pf:getTerrainHeight(p[1], p[2])
  pf:getTerrainHeight(p[3], p[4])
end
local elapsed = os.clock() - t0

local rec = _G.MmapRequiredFiles or {}

local function write_manifest(path, set)
  local keys = {}
  for k in pairs(set or {}) do keys[#keys + 1] = k end
  table.sort(keys)
  local out, e = io.open(path, "w")
  if not out then die("writing manifest " .. tostring(path) .. ": " .. tostring(e)) end
  for _, k in ipairs(keys) do
    out:write(k, "\n")
  end
  out:close()
  return #keys
end

local n_present = write_manifest(present_out, rec.present)
local n_missing = write_manifest(missing_out, rec.missing)

print(string.format("File:       %s", orderings_file))
print(string.format("Data dir:    %s  map %d", DATA_DIR, MAP_ID))
print(string.format("Pairs:       %d", #pairs_list))
print(string.format("Recorded:    %.1f ms", elapsed * 1000))
print(string.format("Present:     %d files -> %s", n_present, present_out))
print(string.format("Missing:     %d files -> %s", n_missing, missing_out))

