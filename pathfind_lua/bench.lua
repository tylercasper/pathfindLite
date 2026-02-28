-- bench.lua — benchmark C++ and Lua pathfinders against a set of coordinate pairs.
--
-- Usage:
--   luajit bench.lua [file] [dataDir] [mapId] [cppBin] [flags...]
--
--   file    — path to an orderings file (default: orderings.json)
--   dataDir — path containing mmaps/ and maps/ (default: /Volumes/Storage/Files/tbc)
--   mapId   — WoW map ID (default: 0)
--   cppBin  — path to the C++ pathfind binary
--
-- Output filter flags (no pipe needed):
--   --mismatches   print only the "Mismatches: N / M" summary line
--   --timing       print only the Lua/C++ timing summary block
--
-- FILE FORMAT
-- -----------
-- Files must use the following Lua-table syntax:
--
--   ["orderings"] = {
--     {
--       <solo_number>,          -- ignored
--       { <numbers...> },       -- ignored
--       {                       -- list of coordinate pairs
--         { x1, y1, x2, y2 },
--         { x1, y1, x2, y2 },
--         ...
--       },
--     },
--     ...
--   }
--
-- Coordinates are WoW world-space yards (valid range: roughly ±17066).
-- Decimal portions are ignored (only integer parts are used).
-- Any number of orderings and pairs per ordering is supported.

-- ---------------------------------------------------------------------------
-- Argument parsing: separate flags from positional args
-- ---------------------------------------------------------------------------
local filter_mismatches = false
local filter_timing     = false
local positional = {}
if arg then
    for i = 1, #arg do
        if     arg[i] == "--mismatches" then filter_mismatches = true
        elseif arg[i] == "--timing"     then filter_timing     = true
        else   positional[#positional + 1] = arg[i]
        end
    end
end

local orderings_file = positional[1] or "orderings.json"
local DATA_DIR       = positional[2] or "/Volumes/Storage/Files/tbc"
local MAP_ID         = positional[3] and tonumber(positional[3]) or 0
local CPP_BIN        = positional[4] or
    "/Volumes/Storage/dev2/cmangos/3build/contrib/pathfindlib/pathfind"

-- Suppress [pathfind] debug output during benchmarking
PathFinderDebug = false

-- Tune LuaJIT: increase trace budget so complex A* loops can be fully compiled
if jit then jit.opt.start(3, "maxmcode=65536", "maxrecord=8000") end

-- ---------------------------------------------------------------------------
-- Parse file
-- ---------------------------------------------------------------------------
local f, err = io.open(orderings_file, "r")
if not f then
    io.stderr:write("ERROR opening " .. orderings_file .. ": " .. tostring(err) .. "\n")
    os.exit(1)
end
local src = f:read("*a")
f:close()

local chunk, cerr = loadstring("return {" .. src .. "}")
if not chunk then
    io.stderr:write("ERROR parsing file: " .. tostring(cerr) .. "\n")
    os.exit(1)
end
local data = chunk()
local orderings = data["orderings"]

local pairs_list = {}
for _, ordering in ipairs(orderings) do
    local coord_sets = ordering[3]
    if coord_sets then
        for _, cs in ipairs(coord_sets) do
            table.insert(pairs_list, {
                math.floor(cs[1]),
                math.floor(cs[2]),
                math.floor(cs[3]),
                math.floor(cs[4]),
            })
        end
    end
end

local verbose = not filter_mismatches and not filter_timing

if verbose then
    print(string.format("File:     %s", orderings_file))
    print(string.format("Data dir: %s  map %d", DATA_DIR, MAP_ID))
    print(string.format("Pairs:    %d", #pairs_list))
    print("")
end

-- ---------------------------------------------------------------------------
-- Lua benchmark
-- ---------------------------------------------------------------------------
local PathFinder = require("PathFinder")

local lua_load_t0 = os.clock()
local pf = PathFinder.new(DATA_DIR, MAP_ID)
local lua_load_elapsed = os.clock() - lua_load_t0

if not pf:isValid() then
    io.stderr:write("ERROR: Lua PathFinder failed to initialise\n")
    os.exit(1)
end

local lua_results = {}
local lua_t0 = os.clock()
for _, p in ipairs(pairs_list) do
    table.insert(lua_results, pf:computeDistance(p[1], p[2], p[3], p[4]))
end
local lua_elapsed = os.clock() - lua_t0

-- ---------------------------------------------------------------------------
-- C++ benchmark (one subprocess per pair)
-- ---------------------------------------------------------------------------
local cpp_results = {}
local cpp_t0 = os.clock()
for _, p in ipairs(pairs_list) do
    local cmd = string.format('%s %s %d %d %d %d %d 2>/dev/null',
        CPP_BIN, DATA_DIR, MAP_ID, p[1], p[2], p[3], p[4])
    local h = io.popen(cmd)
    local out = h:read("*l")
    h:close()
    table.insert(cpp_results, tonumber(out))
end
local cpp_elapsed = os.clock() - cpp_t0

-- ---------------------------------------------------------------------------
-- Results table
-- ---------------------------------------------------------------------------
local function fmt(d)
    if d == nil or d < 0 then return "  no path" end
    return string.format("%9.4f", d)
end

local mismatches = 0
if verbose then
    print(string.format("%-4s  %-26s  %-10s  %-10s  %s",
        "#", "x1,y1 -> x2,y2", "Lua", "C++", "match"))
    print(string.rep("-", 68))
end

for i, p in ipairs(pairs_list) do
    local l = lua_results[i]
    local c = cpp_results[i]
    local match
    if (l == nil or l < 0) and (c == nil or c < 0) then
        match = "ok"
    elseif l ~= nil and c ~= nil and math.abs(l - c) < 0.01 then
        match = "ok"
    else
        match = "MISMATCH"
        mismatches = mismatches + 1
    end
    if verbose then
        print(string.format("%-4d  %6d,%-6d -> %6d,%-6d  %s  %s  %s",
            i, p[1], p[2], p[3], p[4], fmt(l), fmt(c), match))
    end
end

-- ---------------------------------------------------------------------------
-- Timing summary
-- ---------------------------------------------------------------------------
local n = #pairs_list

if filter_mismatches then
    -- Single line, easy to read programmatically
    print(string.format("Mismatches: %d / %d", mismatches, n))
elseif filter_timing then
    -- Just the timing block (what bench_lua51.sh needs, without tail -6)
    print("Lua:")
    print(string.format("  load:       %8.1f ms", lua_load_elapsed * 1000))
    print(string.format("  compute:    %8.1f ms total,  %.3f ms/pair avg",
        lua_elapsed * 1000, lua_elapsed / n * 1000))
    print("")
    print("C++ (per-process, includes startup overhead):")
    print(string.format("  compute:    %8.1f ms total,  %.3f ms/pair avg",
        cpp_elapsed * 1000, cpp_elapsed / n * 1000))
else
    print(string.rep("-", 68))
    print(string.format("Mismatches: %d / %d", mismatches, n))
    print("")
    print("Lua:")
    print(string.format("  load:       %8.1f ms", lua_load_elapsed * 1000))
    print(string.format("  compute:    %8.1f ms total,  %.3f ms/pair avg",
        lua_elapsed * 1000, lua_elapsed / n * 1000))
    print("")
    print("C++ (per-process, includes startup overhead):")
    print(string.format("  compute:    %8.1f ms total,  %.3f ms/pair avg",
        cpp_elapsed * 1000, cpp_elapsed / n * 1000))
end
