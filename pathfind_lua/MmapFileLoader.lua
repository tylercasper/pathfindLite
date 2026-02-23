-- MmapFileLoader.lua
-- Loads CMaNGOS navmesh and terrain data files from disk.
-- Returns raw byte strings that PathFinder passes to NavMesh/TerrainMap for parsing.
--
-- To support a different on-disk format, create an alternative module that
-- implements the same three functions:
--   loadNavMeshParams(dataDir, mapId)     -> bytes (28) or nil
--   loadNavTile(dataDir, mapId, tx, ty)   -> bytes or nil
--   loadTerrainTile(dataDir, mapId, tx, ty) -> bytes or nil

local M = {}

-- Returns the 28-byte dtNavMeshParams block from the .mmap file, or nil on error.
function M.loadNavMeshParams(dataDir, mapId)
    local path = string.format("%s/mmaps/%03d.mmap", dataDir, mapId)
    local fp = io.open(path, "rb")
    if not fp then return nil, path end
    local data = fp:read(28)
    fp:close()
    if not data or #data < 28 then return nil, path end
    return data, path
end

-- Returns the full raw bytes of one .mmtile nav tile, or nil on error.
function M.loadNavTile(dataDir, mapId, tx, ty)
    local path = string.format("%s/mmaps/%03d%02d%02d.mmtile",
        dataDir, mapId, tx, ty)
    local f = io.open(path, "rb")
    if not f then return nil, path end
    local data = f:read("*a")
    f:close()
    if not data or #data < 20 then return nil, path end
    return data, path
end

-- Returns the full raw bytes of one .map terrain tile, or nil if missing.
function M.loadTerrainTile(dataDir, mapId, tx, ty)
    local path = string.format("%s/maps/%03d%02d%02d.map",
        dataDir, mapId, tx, ty)
    local f = io.open(path, "rb")
    if not f then return nil, path end
    local data = f:read("*a")
    f:close()
    return data, path  -- nil data is OK (terrain tile may be absent)
end

return M
