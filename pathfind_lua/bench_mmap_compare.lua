-- bench_mmap_compare.lua
-- Compares computeDistance results between:
--   (A) original on-disk loader (MmapFileLoader) reading a dataDir with mmaps/ + maps/
--   (B) Lua blob loader (MmapLuaLoader) reading a directory containing generated addon folders
--
-- Usage:
--   lua bench_mmap_compare.lua [orderingsFile] [origDataDir] [mapId] [addonsDir] [flags...]
--
-- Flags:
--   --addon-prefix <prefix>   (default: qhstub_mmapdata)
--   --tolerance <float>       (default: 0.01)
--
-- Exits non-zero on mismatch.

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
    if a == "--addon-prefix" then
      flags.addon_prefix = argv[i + 1]
      i = i + 2
    elseif a == "--tolerance" then
      flags.tolerance = tonumber(argv[i + 1])
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
local ORIG_DATA_DIR  = positional[2] or "/Volumes/Storage/Files/tbc"
local MAP_ID         = positional[3] and tonumber(positional[3]) or 0
local ADDONS_DIR     = positional[4] or "./generated_addons"

local addon_prefix = flags.addon_prefix or "qhstub_mmapdata"
local tol = (type(flags.tolerance) == "number" and flags.tolerance) or 0.01

-- Suppress [pathfind] debug output
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
if #pairs_list == 0 then die("no coordinate pairs found in file") end

local function run(loaderModule, dataDir)
  _G.MmapLoaderModule = loaderModule
  package.loaded["PathFinder"] = nil

  local PathFinder = require("PathFinder")
  local t0 = os.clock()
  local pf = PathFinder.new(dataDir, MAP_ID)
  local load_elapsed = os.clock() - t0
  if not pf:isValid() then
    return nil, load_elapsed, "PathFinder invalid"
  end

  local results = {}
  local t1 = os.clock()
  for _, p in ipairs(pairs_list) do
    results[#results + 1] = pf:computeDistance(p[1], p[2], p[3], p[4])
  end
  local elapsed = os.clock() - t1
  return results, load_elapsed, elapsed
end

-- Run original loader (disk)
local orig_results, orig_load, orig_elapsed_or_err = run("MmapFileLoader", ORIG_DATA_DIR)
if not orig_results then
  die("original loader failed: " .. tostring(orig_elapsed_or_err))
end

-- Run Lua blob loader (addons dir)
_G.MmapLuaAddonPrefix = addon_prefix
_G.MmapLuaDB = nil
package.loaded["MmapLuaLoader"] = nil

local lua_results, lua_load, lua_elapsed_or_err = run("MmapLuaLoader", ADDONS_DIR)
if not lua_results then
  die("lua loader failed: " .. tostring(lua_elapsed_or_err))
end

local mismatches = 0
local first = {}

for i, p in ipairs(pairs_list) do
  local a = orig_results[i]
  local b = lua_results[i]
  local ok
  if (a == nil or a < 0) and (b == nil or b < 0) then
    ok = true
  elseif type(a) == "number" and type(b) == "number" and math.abs(a - b) <= tol then
    ok = true
  else
    ok = false
  end
  if not ok then
    mismatches = mismatches + 1
    if #first < 10 then
      first[#first + 1] = {
        i = i,
        x1 = p[1], y1 = p[2], x2 = p[3], y2 = p[4],
        a = a, b = b,
      }
    end
  end
end

print(string.format("File:        %s", orderings_file))
print(string.format("Orig data:    %s  map %d", ORIG_DATA_DIR, MAP_ID))
print(string.format("Addons data:  %s  prefix %s", ADDONS_DIR, addon_prefix))
print(string.format("Pairs:        %d", #pairs_list))
print(string.format("Tolerance:    %.4f", tol))
print("")
print("Timing:")
print(string.format("  original: load %.1f ms, compute %.1f ms", orig_load * 1000, orig_elapsed_or_err * 1000))
print(string.format("  lua     : load %.1f ms, compute %.1f ms", lua_load * 1000, lua_elapsed_or_err * 1000))
print("")
print(string.format("Mismatches: %d / %d", mismatches, #pairs_list))

if mismatches > 0 then
  print("")
  print("First mismatches:")
  for _, m in ipairs(first) do
    print(string.format(
      "#%d  (%d,%d)->(%d,%d)  orig=%s  lua=%s",
      m.i, m.x1, m.y1, m.x2, m.y2, tostring(m.a), tostring(m.b)
    ))
  end
  os.exit(1)
end

