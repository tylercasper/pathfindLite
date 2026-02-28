-- PathFinder.lua
-- Port of PathFinder.cpp to Lua 5.1
-- Loads .mmap/.mmtile navmesh tiles and .map terrain files.

local NavMeshModule  = require("NavMesh")
local NavMeshQuery   = require("NavMeshQuery")
local TerrainMap     = require("TerrainMap")
local loaderModule = (type(_G) == "table" and rawget(_G, "MmapLoaderModule")) or nil
if type(loaderModule) ~= "string" or loaderModule == "" then
    loaderModule = "MmapFileLoader"
end
local MmapLoader     = require(loaderModule)

local TERRAIN_INVALID_HEIGHT = TerrainMap.TERRAIN_INVALID_HEIGHT

-- Set PathFinderDebug = false before requiring this module to suppress log output.
local function dbg(fmt, ...)
    if PathFinderDebug == false then return end
    local msg = string.format("[pathfind] " .. fmt, ...)
    local qhs = rawget(_G, "QHS")
    if type(qhs) == "table" and type(qhs.chat) == "function" then
        -- Route through qhstub's logger so it persists into SavedVariables.
        qhs.chat(msg)
    elseif type(io) == "table" and io.stderr and type(io.stderr.write) == "function" then
        io.stderr:write(msg .. "\n")
    elseif type(DEFAULT_CHAT_FRAME) == "table" and type(DEFAULT_CHAT_FRAME.AddMessage) == "function" then
        DEFAULT_CHAT_FRAME:AddMessage(msg)
    elseif type(print) == "function" then
        print(msg)
    end
end

local M = {}

local BLOCK_SIZE  = 533.33333
local TILE_ORIGIN = 32
local MAX_POLYS   = 4096
local WORLD_MAX   = 17066.666

-- ---------------------------------------------------------------------------
-- Coordinate conversion helpers
-- ---------------------------------------------------------------------------

-- WoW (x, y, z) -> Recast (y, z, x)
local function toRecast(x, y, z)
    return {y, z, x}
end

-- worldToTile: compute tile coords from WoW (x,y)
-- Returns tx, ty or nil on failure
local function worldToTile(x, y)
    local ftx = TILE_ORIGIN - x / BLOCK_SIZE
    local fty = TILE_ORIGIN - y / BLOCK_SIZE
    if ftx < 0 or ftx >= 64 or fty < 0 or fty >= 64 then
        dbg("ERROR: coords (%.2f, %.2f) out of world bounds", x, y)
        return nil
    end
    return math.floor(ftx), math.floor(fty)
end

local function packTileID(tx, ty)
    return tx * 65536 + ty
end

-- ---------------------------------------------------------------------------
-- PathFinder constructor
-- ---------------------------------------------------------------------------

function M.new(dataDir, mapId)
    local pf = {
        _dataDir         = dataDir,
        _mapId           = mapId,
        _navMesh         = nil,
        _navQuery        = nil,
        _loadedNavTiles  = {},   -- set: tileID -> true
        _terrainTiles    = {},   -- map: tileID -> TerrainMap
        _initialized     = false,
        -- Pre-allocated per-query buffers (avoids alloc on every computeDistance call)
        _startPos        = {0,0,0},
        _endPos          = {0,0,0},
        _endPosAdj       = {0,0,0},
        _extents         = {2.0, 4.0, 2.0},
    }

    -- Load .mmap params file
    -- dtNavMeshParams: orig[3] + tileWidth + tileHeight + maxTiles + maxPolys = 7*4 = 28 bytes
    local paramsData, mmapPath = MmapLoader.loadNavMeshParams(dataDir, mapId)
    dbg("loading navmesh params: %s", mmapPath)
    if not paramsData then
        dbg("ERROR: could not read %s", mmapPath)
        return pf
    end

    -- Create NavMesh
    local navmesh = NavMeshModule.new(paramsData)
    pf._navMesh = navmesh

    -- Create NavMeshQuery
    pf._navQuery = NavMeshQuery.new(navmesh, 65535)
    pf._filter   = NavMeshQuery.newFilter()

    dbg("navmesh ready")
    pf._initialized = true

    -- loadNavTile(tx, ty)
    function pf:loadNavTile(tx, ty)
        local id = packTileID(tx, ty)
        if self._loadedNavTiles[id] then return true end

        local data, path = MmapLoader.loadNavTile(self._dataDir, self._mapId, tx, ty)
        dbg("loading nav tile (%d,%d): %s", tx, ty, path)
        if not data then
            dbg("WARNING: nav tile not found or too short")
            return false
        end

        local ok, err = self._navMesh:addTile(data)
        if not ok then
            dbg("ERROR: addTile failed: %s", tostring(err))
            return false
        end

        dbg("nav tile (%d,%d) loaded OK (%d bytes)", tx, ty, #data)
        self._loadedNavTiles[id] = true
        return true
    end

    -- ensureNavTilesLoaded(x1, y1, x2, y2)
    -- Loads all nav tiles covering the bounding box of (x1,y1)-(x2,y2), plus TILE_PADDING
    -- tiles of margin on each side.  A* paths can leave the straight-line bounding box when
    -- routing around terrain, so the extra margin prevents "tile not loaded" failures.
    local TILE_PADDING = 1
    function pf:ensureNavTilesLoaded(x1, y1, x2, y2)
        local tx1, ty1 = worldToTile(x1, y1)
        local tx2, ty2 = worldToTile(x2, y2)
        if not tx1 or not tx2 then return end

        -- Lua analyzer may infer nil; guard is above, values are integers here.
        local txMin = math.max(0,  math.min(tx1, tx2) - TILE_PADDING)
        local txMax = math.min(63, math.max(tx1, tx2) + TILE_PADDING)
        local tyMin = math.max(0,  math.min(ty1, ty2) - TILE_PADDING)
        local tyMax = math.min(63, math.max(ty1, ty2) + TILE_PADDING)

        dbg("tile range x:[%d,%d] y:[%d,%d]", txMin or 0, txMax or 0, tyMin or 0, tyMax or 0)

        for tx = txMin, txMax do
            for ty = tyMin, tyMax do
                self:loadNavTile(tx, ty)
            end
        end
    end

    -- getTerrainHeight(x, y) -> height
    function pf:getTerrainHeight(x, y)
        local tx, ty = worldToTile(x, y)
        if not tx then return TERRAIN_INVALID_HEIGHT end

        local id = packTileID(tx, ty)
        if not self._terrainTiles[id] then
            local data, path = MmapLoader.loadTerrainTile(self._dataDir, self._mapId, tx, ty)
            dbg("loading terrain tile (%d,%d): %s", tx, ty, path)

            local tm = TerrainMap.new(data)
            self._terrainTiles[id] = tm

            local ok = tm:isLoaded()
            dbg("terrain tile (%d,%d) %s", tx, ty, ok and "loaded OK" or "FAILED (no .map file?)")
        end

        local h = self._terrainTiles[id]:getHeight(x, y)
        dbg("terrain height at (%.2f, %.2f) = %.4f", x, y, h)
        return h
    end

    -- _run_query(x1, y1, x2, y2) -> straightResult, nstraight, total  or  nil, 0, -1
    -- Shared body used by computeDistance and computePath.
    function pf:_run_query(x1, y1, x2, y2)
        if math.abs(x1) > WORLD_MAX or math.abs(y1) > WORLD_MAX or
           math.abs(x2) > WORLD_MAX or math.abs(y2) > WORLD_MAX then
            dbg("ERROR: coordinates exceed world bounds")
            return nil, 0, -1.0
        end

        if not self._initialized then
            dbg("ERROR: not initialized")
            return nil, 0, -1.0
        end

        self:ensureNavTilesLoaded(x1, y1, x2, y2)

        local z1 = self:getTerrainHeight(x1, y1)
        local z2 = self:getTerrainHeight(x2, y2)

        if z1 == TERRAIN_INVALID_HEIGHT or z2 == TERRAIN_INVALID_HEIGHT then
            dbg("ERROR: terrain height lookup failed (z1=%.2f z2=%.2f)", z1, z2)
            return nil, 0, -1.0
        end

        -- Fill pre-allocated recast buffers (avoids 2 table allocs per call)
        local startPos = self._startPos
        startPos[1]=y1; startPos[2]=z1; startPos[3]=x1
        local endPos = self._endPos
        endPos[1]=y2; endPos[2]=z2; endPos[3]=x2
        dbg("recast start=(%.2f,%.2f,%.2f) end=(%.2f,%.2f,%.2f)",
            startPos[1], startPos[2], startPos[3],
            endPos[1],   endPos[2],   endPos[3])

        local extents = self._extents   -- pre-allocated {2.0, 4.0, 2.0}
        local filter  = self._filter

        local startRef, startNearPt = self._navQuery:findNearestPoly(startPos, extents, filter)
        local endRef,   endNearPt   = self._navQuery:findNearestPoly(endPos,   extents, filter)

        dbg("startRef=%.0f endRef=%.0f", startRef or 0, endRef or 0)

        if not startRef or startRef == 0 or not endRef or endRef == 0 then
            dbg("ERROR: could not find nearest poly")
            return nil, 0, -1.0
        end

        local path, npolys, status = self._navQuery:findPath(
            startRef, endRef, startPos, endPos, filter, MAX_POLYS)
        dbg("findPath: %d polys", npolys)

        if npolys == 0 then return nil, 0, -1.0 end

        -- Clamp end to last reachable poly if incomplete (pre-alloc buffer)
        local endPosAdj = self._endPosAdj
        endPosAdj[1]=endPos[1]; endPosAdj[2]=endPos[2]; endPosAdj[3]=endPos[3]
        if path[npolys] ~= endRef then
            dbg("path incomplete — clamping to last reachable poly")
            local cp, _ = self._navQuery:closestPointOnPoly(path[npolys], endPos)
            if cp then
                endPosAdj[1] = cp[1]; endPosAdj[2] = cp[2]; endPosAdj[3] = cp[3]
            end
        end

        local straightResult, nstraight, _ = self._navQuery:findStraightPath(
            startPos, endPosAdj, path, npolys, MAX_POLYS, 0)

        dbg("findStraightPath: %d points", nstraight)

        if nstraight < 2 then return nil, 0, -1.0 end

        local total = 0.0
        for k = 1, nstraight - 1 do
            local a = straightResult[k]
            local b = straightResult[k+1]
            local dx = b[1]-a[1]; local dy = b[2]-a[2]; local dz = b[3]-a[3]
            total = total + math.sqrt(dx*dx + dy*dy + dz*dz)
        end

        dbg("total distance: %.4f", total)
        return straightResult, nstraight, total
    end

    -- computeDistance(x1, y1, x2, y2) -> distance or -1
    function pf:computeDistance(x1, y1, x2, y2)
        dbg("computeDistance (%.4f, %.4f) -> (%.4f, %.4f)", x1, y1, x2, y2)
        local _, _, total = self:_run_query(x1, y1, x2, y2)
        return total
    end

    -- computePath(x1, y1, x2, y2) -> waypoints, total_dist  or  nil, -1
    -- waypoints is an array of {wx, wy} in WoW world-yard coordinates.
    -- Recast stores points as {wow_y, wow_z, wow_x}, so wx=[3], wy=[1].
    function pf:computePath(x1, y1, x2, y2)
        dbg("computePath (%.4f, %.4f) -> (%.4f, %.4f)", x1, y1, x2, y2)
        local straightResult, nstraight, total = self:_run_query(x1, y1, x2, y2)
        if not straightResult then
            return nil, -1.0
        end

        local waypoints = {}
        for k = 1, nstraight do
            local pt = straightResult[k]   -- {wow_y, wow_z, wow_x}
            waypoints[k] = { pt[3], pt[1] }  -- {wx, wy}
        end

        return waypoints, total
    end

    function pf:isValid()
        return self._initialized
    end

    return pf
end

if package and package.loaded then
    package.loaded["PathFinder"] = M
end
return M
