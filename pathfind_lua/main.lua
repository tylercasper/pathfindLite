-- main.lua
-- Entry point for the Lua pathfinder.
-- Usage: luajit main.lua <dataDir> <mapId> <x1> <y1> <x2> <y2>

local PathFinder = require("PathFinder")

local function main()
    local args = arg  -- LuaJIT / Lua 5.1 command-line args table

    if not args or #args < 6 then
        io.stderr:write(string.format("Usage: %s <dataDir> <mapId> <x1> <y1> <x2> <y2>\n",
            args and args[0] or "main.lua"))
        os.exit(1)
    end

    local dataDir = args[1]
    local mapId   = tonumber(args[2])
    local x1      = tonumber(args[3])
    local y1      = tonumber(args[4])
    local x2      = tonumber(args[5])
    local y2      = tonumber(args[6])

    if not mapId or not x1 or not y1 or not x2 or not y2 then
        io.stderr:write("ERROR: invalid arguments\n")
        os.exit(1)
    end

    local pf = PathFinder.new(dataDir, math.floor(mapId))
    if not pf:isValid() then
        io.stderr:write(string.format("Failed to load navmesh for map %d from '%s'\n", mapId, dataDir))
        os.exit(1)
    end

    local dist = pf:computeDistance(x1, y1, x2, y2)
    print(string.format("%.4f", dist))
end

main()
