-- MmapRecordingFileLoader.lua
-- Like MmapFileLoader.lua, but records which files were needed.
--
-- Intended for offline tooling/benchmarks to determine the minimal subset of
-- .mmap/.mmtile/.map files needed for a given coordinate set.
--
-- Records into a global table:
--   MmapRequiredFiles = {
--     present = { [relPath]=true, ... },
--     missing = { [relPath]=true, ... },
--     all     = { [relPath]=true, ... },
--   }
--
-- relPath is relative to dataDir, using forward slashes, e.g.:
--   mmaps/000.mmap
--   mmaps/0000000.mmtile
--   maps/0000000.map

local M = {}

local function norm(p)
  return (tostring(p or "")):gsub("\\", "/")
end

local function ensure_tbl()
  _G.MmapRequiredFiles = _G.MmapRequiredFiles or { present = {}, missing = {}, all = {} }
  _G.MmapRequiredFiles.present = _G.MmapRequiredFiles.present or {}
  _G.MmapRequiredFiles.missing = _G.MmapRequiredFiles.missing or {}
  _G.MmapRequiredFiles.all = _G.MmapRequiredFiles.all or {}
  return _G.MmapRequiredFiles
end

local function relpath(dataDir, path)
  local dd = norm(dataDir)
  local p = norm(path)
  if dd ~= "" then
    if dd:sub(-1) ~= "/" then dd = dd .. "/" end
    if p:sub(1, #dd) == dd then
      return p:sub(#dd + 1)
    end
  end
  return p
end

local function record(dataDir, path, ok)
  local t = ensure_tbl()
  local rel = relpath(dataDir, path)
  t.all[rel] = true
  if ok then
    t.present[rel] = true
  else
    t.missing[rel] = true
  end
end

-- Returns the 28-byte dtNavMeshParams block from the .mmap file, or nil on error.
function M.loadNavMeshParams(dataDir, mapId)
  local path = string.format("%s/mmaps/%03d.mmap", dataDir, mapId)
  local fp = io.open(path, "rb")
  if not fp then
    record(dataDir, path, false)
    return nil, path
  end
  local data = fp:read(28)
  fp:close()
  local ok = (type(data) == "string" and #data >= 28)
  record(dataDir, path, ok)
  if not ok then return nil, path end
  return data, path
end

-- Returns the full raw bytes of one .mmtile nav tile, or nil on error.
function M.loadNavTile(dataDir, mapId, tx, ty)
  local path = string.format("%s/mmaps/%03d%02d%02d.mmtile", dataDir, mapId, tx, ty)
  local f = io.open(path, "rb")
  if not f then
    record(dataDir, path, false)
    return nil, path
  end
  local data = f:read("*a")
  f:close()
  local ok = (type(data) == "string" and #data >= 20)
  record(dataDir, path, ok)
  if not ok then return nil, path end
  return data, path
end

-- Returns the full raw bytes of one .map terrain tile, or nil if missing.
function M.loadTerrainTile(dataDir, mapId, tx, ty)
  local path = string.format("%s/maps/%03d%02d%02d.map", dataDir, mapId, tx, ty)
  local f = io.open(path, "rb")
  if not f then
    record(dataDir, path, false)
    return nil, path
  end
  local data = f:read("*a")
  f:close()
  record(dataDir, path, type(data) == "string")
  return data, path
end

if package and package.loaded then
  package.loaded["MmapRecordingFileLoader"] = M
end
return M

