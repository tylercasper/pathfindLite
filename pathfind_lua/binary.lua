-- binary.lua
-- Little-endian binary reading utilities for Lua 5.1
-- All functions take a string `s` and 1-based byte offset `i`.
-- Returns (value, next_offset).

local M = {}

function M.readU8(s, i)
    local b = s:byte(i)
    return b, i + 1
end

function M.readU16(s, i)
    local a, b = s:byte(i, i+1)
    return a + b*256, i + 2
end

function M.readI16(s, i)
    local v, ni = M.readU16(s, i)
    if v >= 32768 then v = v - 65536 end
    return v, ni
end

-- For uint32 we use plain arithmetic to avoid signed overflow from bit lib.
function M.readU32(s, i)
    local a, b, c, d = s:byte(i, i+3)
    return a + b*256 + c*65536 + d*16777216, i + 4
end

function M.readI32(s, i)
    local v, ni = M.readU32(s, i)
    if v >= 2147483648 then v = v - 4294967296 end
    return v, ni
end

-- IEEE 754 single precision float, manual decode.
-- No bit library needed; pure arithmetic is correct.
function M.readFloat(s, i)
    local a, b, c, d = s:byte(i, i+3)
    -- little-endian: bits = d<<24 | c<<16 | b<<8 | a
    local bits = a + b*256 + c*65536 + d*16777216

    local sign     = math.floor(bits / 2147483648)  -- bit 31
    local exponent = math.floor(bits / 8388608) % 256  -- bits 30-23
    local mantissa = bits % 8388608                 -- bits 22-0

    if exponent == 255 then
        -- NaN or Inf
        if mantissa ~= 0 then
            return 0/0, i + 4  -- NaN
        else
            return sign == 0 and math.huge or -math.huge, i + 4
        end
    end

    local value
    if exponent == 0 then
        -- Denormalized
        value = math.ldexp(mantissa, -149)
    else
        -- Normalized: implicit leading 1
        value = math.ldexp(mantissa + 8388608, exponent - 150)
    end

    if sign == 1 then value = -value end
    return value, i + 4
end

-- uint64 as two uint32 parts stored as a Lua double.
-- For WoW TBC tile counts this stays within double precision.
function M.readU64(s, i)
    local lo, ni = M.readU32(s, i)
    local hi, ni2 = M.readU32(s, ni)
    return lo + hi * 4294967296, ni2
end

return M
