-- TerrainMap.lua
-- Port of TerrainMap.cpp to Lua 5.1
-- Reads a .map file (as a Lua string) and provides getHeight(x, y).

local bin = require("binary")

local M = {}

-- Constants matching the C++ extractor
local MAP_MAGIC         = 0x5350414d  -- 'MAPS'
local MAP_VERSION_MAGIC = 0x342e3173  -- 's1.4'
local MAP_HEIGHT_MAGIC  = 0x5447484d  -- 'MHGT'

local MAP_HEIGHT_NO_HEIGHT = 0x0001
local MAP_HEIGHT_AS_INT16  = 0x0002
local MAP_HEIGHT_AS_INT8   = 0x0004

local TERRAIN_INVALID_HEIGHT = -200000.0
local GRID_SIZE              = 533.33333
local MAP_RESOLUTION         = 128
local TILE_ORIGIN            = 32

-- Hole lookup tables
local holetab_h = {0x1111, 0x2222, 0x4444, 0x8888}
local holetab_v = {0x000F, 0x00F0, 0x0F00, 0xF000}

local function band(a, b)
    -- Bitwise AND via modular arithmetic (handles values up to 2^16)
    local result = 0
    local bit = 1
    while bit <= a or bit <= b do
        if (a % (bit*2)) >= bit and (b % (bit*2)) >= bit then
            result = result + bit
        end
        bit = bit * 2
    end
    return result
end

-- isHole(holes, row, col) -- holes is a 16x16 array (1-indexed)
local function isHole(holes, row, col)
    local cellRow = math.floor(row / 8) + 1
    local cellCol = math.floor(col / 8) + 1
    local holeRow = math.floor((row % 8) / 2) + 1
    local holeCol = math.floor((col - (math.floor(col/8)*8)) / 2) + 1
    if cellRow < 1 or cellRow > 16 or cellCol < 1 or cellCol > 16 then return false end
    local hole = holes[(cellRow-1)*16 + cellCol]
    return band(band(hole, holetab_h[holeCol] or 0), holetab_v[holeRow] or 0) ~= 0
end

-- MapFileHeader layout:
--   mapMagic(u32) versionMagic(u32) areaMapOffset(u32) areaMapSize(u32)
--   heightMapOffset(u32) heightMapSize(u32) liquidMapOffset(u32) liquidMapSize(u32)
--   holesOffset(u32) holesSize(u32)
-- Total: 10 * 4 = 40 bytes

-- MapHeightHeader layout:
--   fourcc(u32) flags(u32) gridHeight(f) gridMaxHeight(f)
-- Total: 16 bytes

function M.new(data)
    -- data is a Lua string containing the full .map file

    local tm = {
        _loaded                  = false,
        _gridHeight              = TERRAIN_INVALID_HEIGHT,
        _gridIntHeightMultiplier = 0,
        _V9                      = nil,  -- float, 129*129
        _V8                      = nil,  -- float, 128*128
        _uint16_V9               = nil,
        _uint16_V8               = nil,
        _uint8_V9                = nil,
        _uint8_V8                = nil,
        _holes                   = {},   -- flat array, 16*16 = 256 entries
        _mode                    = "flat",
    }

    -- Initialize holes to 0
    for k = 1, 256 do tm._holes[k] = 0 end

    -- Define methods up-front so callers can safely use them even when load fails.
    function tm:isLoaded()
        return self._loaded
    end

    -- getHeight(x, y) -> height in WoW world units
    function tm:getHeight(x, y)
        if not self._loaded then
            return TERRAIN_INVALID_HEIGHT
        end
        if self._mode == "flat" then
            return self._gridHeight
        elseif self._mode == "float" then
            return self:_getHeightFromFloat(x, y)
        elseif self._mode == "uint8" then
            return self:_getHeightFromUint8(x, y)
        elseif self._mode == "uint16" then
            return self:_getHeightFromUint16(x, y)
        end
        return TERRAIN_INVALID_HEIGHT
    end

    if not data or #data < 40 then
        return tm
    end

    -- Parse MapFileHeader
    local i = 1
    local mapMagic, versionMagic
    mapMagic,    i = bin.readU32(data, i)
    versionMagic,i = bin.readU32(data, i)

    if mapMagic ~= MAP_MAGIC or versionMagic ~= MAP_VERSION_MAGIC then
        if io and io.stderr and io.stderr.write then
            io.stderr:write("[terrain] bad map magic\n")
        end
        return tm
    end

    local areaMapOffset, areaMapSize, heightMapOffset, heightMapSize
    local liquidMapOffset, liquidMapSize, holesOffset, holesSize
    areaMapOffset,   i = bin.readU32(data, i)
    areaMapSize,     i = bin.readU32(data, i)
    heightMapOffset, i = bin.readU32(data, i)
    heightMapSize,   i = bin.readU32(data, i)
    liquidMapOffset, i = bin.readU32(data, i)
    liquidMapSize,   i = bin.readU32(data, i)
    holesOffset,     i = bin.readU32(data, i)
    holesSize,       i = bin.readU32(data, i)

    -- Load holes (16*16 uint16 = 512 bytes)
    if holesOffset ~= 0 then
        local hi = holesOffset + 1  -- 1-based
        for k = 1, 256 do
            local v; v, hi = bin.readU16(data, hi)
            tm._holes[k] = v
        end
    end

    -- Load height data
    if heightMapOffset ~= 0 then
        local hi = heightMapOffset + 1  -- 1-based
        -- MapHeightHeader
        local fourcc, flags, gridHeight, gridMaxHeight
        fourcc,        hi = bin.readU32(data, hi)
        flags,         hi = bin.readU32(data, hi)
        gridHeight,    hi = bin.readFloat(data, hi)
        gridMaxHeight, hi = bin.readFloat(data, hi)

        if fourcc ~= MAP_HEIGHT_MAGIC then
            if io and io.stderr and io.stderr.write then
                io.stderr:write("[terrain] bad height magic\n")
            end
            return tm
        end

        tm._gridHeight = gridHeight

        if band(flags, MAP_HEIGHT_NO_HEIGHT) ~= 0 then
            tm._mode = "flat"
        elseif band(flags, MAP_HEIGHT_AS_INT16) ~= 0 then
            -- Read 129*129 uint16 V9 + 128*128 uint16 V8
            local v9 = {}
            for k = 1, 129*129 do
                local v; v, hi = bin.readU16(data, hi)
                v9[k] = v
            end
            local v8 = {}
            for k = 1, 128*128 do
                local v; v, hi = bin.readU16(data, hi)
                v8[k] = v
            end
            tm._uint16_V9 = v9
            tm._uint16_V8 = v8
            tm._gridIntHeightMultiplier = (gridMaxHeight - gridHeight) / 65535.0
            tm._mode = "uint16"
        elseif band(flags, MAP_HEIGHT_AS_INT8) ~= 0 then
            -- Read 129*129 uint8 V9 + 128*128 uint8 V8
            local v9 = {}
            for k = 1, 129*129 do
                local v; v, hi = bin.readU8(data, hi)
                v9[k] = v
            end
            local v8 = {}
            for k = 1, 128*128 do
                local v; v, hi = bin.readU8(data, hi)
                v8[k] = v
            end
            tm._uint8_V9 = v9
            tm._uint8_V8 = v8
            tm._gridIntHeightMultiplier = (gridMaxHeight - gridHeight) / 255.0
            tm._mode = "uint8"
        else
            -- float
            local v9 = {}
            for k = 1, 129*129 do
                local v; v, hi = bin.readFloat(data, hi)
                v9[k] = v
            end
            local v8 = {}
            for k = 1, 128*128 do
                local v; v, hi = bin.readFloat(data, hi)
                v8[k] = v
            end
            tm._V9 = v9
            tm._V8 = v8
            tm._mode = "float"
        end
    end

    tm._loaded = true

    function tm:_getHeightFromFloat(x, y)
        if not self._V8 or not self._V9 then return TERRAIN_INVALID_HEIGHT end

        local fx = MAP_RESOLUTION * (TILE_ORIGIN - x / GRID_SIZE)
        local fy = MAP_RESOLUTION * (TILE_ORIGIN - y / GRID_SIZE)

        local x_int = math.floor(fx)
        local y_int = math.floor(fy)
        local xf = fx - x_int
        local yf = fy - y_int
        x_int = x_int % MAP_RESOLUTION  -- & (MAP_RESOLUTION - 1)
        y_int = y_int % MAP_RESOLUTION

        if isHole(self._holes, x_int, y_int) then return TERRAIN_INVALID_HEIGHT end

        local a, b, c
        if xf + yf < 1.0 then
            if xf > yf then
                local h1 = self._V9[ x_int      * 129 + y_int   + 1]
                local h2 = self._V9[(x_int + 1) * 129 + y_int   + 1]
                local h5 = 2.0 * self._V8[x_int * 128 + y_int + 1]
                a = h2 - h1; b = h5 - h1 - h2; c = h1
            else
                local h1 = self._V9[x_int * 129 + y_int     + 1]
                local h3 = self._V9[x_int * 129 + y_int + 1 + 1]
                local h5 = 2.0 * self._V8[x_int * 128 + y_int + 1]
                a = h5 - h1 - h3; b = h3 - h1; c = h1
            end
        else
            if xf > yf then
                local h2 = self._V9[(x_int + 1) * 129 + y_int     + 1]
                local h4 = self._V9[(x_int + 1) * 129 + y_int + 1 + 1]
                local h5 = 2.0 * self._V8[x_int * 128 + y_int + 1]
                a = h2 + h4 - h5; b = h4 - h2; c = h5 - h4
            else
                local h3 = self._V9[ x_int      * 129 + y_int + 1 + 1]
                local h4 = self._V9[(x_int + 1) * 129 + y_int + 1 + 1]
                local h5 = 2.0 * self._V8[x_int * 128 + y_int + 1]
                a = h4 - h3; b = h3 + h4 - h5; c = h5 - h4
            end
        end

        return a * xf + b * yf + c
    end

    function tm:_getHeightFromUint8(x, y)
        if not self._uint8_V8 or not self._uint8_V9 then return self._gridHeight end

        local fx = MAP_RESOLUTION * (TILE_ORIGIN - x / GRID_SIZE)
        local fy = MAP_RESOLUTION * (TILE_ORIGIN - y / GRID_SIZE)

        local x_int = math.floor(fx)
        local y_int = math.floor(fy)
        local xf = fx - x_int
        local yf = fy - y_int
        x_int = x_int % MAP_RESOLUTION
        y_int = y_int % MAP_RESOLUTION

        if isHole(self._holes, x_int, y_int) then return TERRAIN_INVALID_HEIGHT end

        -- ptr = &m_uint8_V9[x_int * 128 + x_int + y_int]  (note: 128+1 stride = 129)
        -- ptr[0]   = V9[x_int*129 + y_int]
        -- ptr[1]   = V9[x_int*129 + y_int + 1]
        -- ptr[129] = V9[(x_int+1)*129 + y_int]
        -- ptr[130] = V9[(x_int+1)*129 + y_int + 1]
        local base = x_int * 129 + y_int + 1  -- 1-based index
        local p0   = self._uint8_V9[base]
        local p1   = self._uint8_V9[base + 1]
        local p129 = self._uint8_V9[base + 129]
        local p130 = self._uint8_V9[base + 130]
        local v8   = self._uint8_V8[x_int * 128 + y_int + 1]

        local a, b, c
        if xf + yf < 1.0 then
            if xf > yf then
                a = p129 - p0
                b = 2 * v8 - p0 - p129
                c = p0
            else
                a = 2 * v8 - p0 - p1
                b = p1 - p0
                c = p0
            end
        else
            if xf > yf then
                a = p129 + p130 - 2 * v8
                b = p130 - p129
                c = 2 * v8 - p130
            else
                a = p130 - p1
                b = p1 + p130 - 2 * v8
                c = 2 * v8 - p130
            end
        end

        return (a * xf + b * yf + c) * self._gridIntHeightMultiplier + self._gridHeight
    end

    function tm:_getHeightFromUint16(x, y)
        if not self._uint16_V8 or not self._uint16_V9 then return self._gridHeight end

        local fx = MAP_RESOLUTION * (TILE_ORIGIN - x / GRID_SIZE)
        local fy = MAP_RESOLUTION * (TILE_ORIGIN - y / GRID_SIZE)

        local x_int = math.floor(fx)
        local y_int = math.floor(fy)
        local xf = fx - x_int
        local yf = fy - y_int
        x_int = x_int % MAP_RESOLUTION
        y_int = y_int % MAP_RESOLUTION

        if isHole(self._holes, x_int, y_int) then return TERRAIN_INVALID_HEIGHT end

        local base = x_int * 129 + y_int + 1  -- 1-based
        local p0   = self._uint16_V9[base]
        local p1   = self._uint16_V9[base + 1]
        local p129 = self._uint16_V9[base + 129]
        local p130 = self._uint16_V9[base + 130]
        local v8   = self._uint16_V8[x_int * 128 + y_int + 1]

        local a, b, c
        if xf + yf < 1.0 then
            if xf > yf then
                a = p129 - p0
                b = 2 * v8 - p0 - p129
                c = p0
            else
                a = 2 * v8 - p0 - p1
                b = p1 - p0
                c = p0
            end
        else
            if xf > yf then
                a = p129 + p130 - 2 * v8
                b = p130 - p129
                c = 2 * v8 - p130
            else
                a = p130 - p1
                b = p1 + p130 - 2 * v8
                c = 2 * v8 - p130
            end
        end

        return (a * xf + b * yf + c) * self._gridIntHeightMultiplier + self._gridHeight
    end

    return tm
end

M.TERRAIN_INVALID_HEIGHT = TERRAIN_INVALID_HEIGHT

if package and package.loaded then
    package.loaded["TerrainMap"] = M
end
return M
