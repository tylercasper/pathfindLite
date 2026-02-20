-- test.lua
-- Run with: luajit test.lua
-- Must be run from the pathfind_lua directory.

local DATA_DIR = "/Volumes/Storage/Files/tbc"

local PathFinder = require("PathFinder")
local passed = 0
local failed = 0

local function check(name, got, expected, tolerance)
    tolerance = tolerance or 0.01
    if got == nil then
        print(string.format("FAIL  %s  got nil, expected %.4f", name, expected))
        failed = failed + 1
    elseif math.abs(got - expected) <= tolerance then
        print(string.format("PASS  %s  %.4f", name, got))
        passed = passed + 1
    else
        print(string.format("FAIL  %s  got %.4f, expected %.4f", name, got, expected))
        failed = failed + 1
    end
end

local function checkNeg(name, got)
    if got == nil or got < 0 then
        print(string.format("PASS  %s  %.4f (no path)", name, got or -1))
        passed = passed + 1
    else
        print(string.format("FAIL  %s  expected no path, got %.4f", name, got))
        failed = failed + 1
    end
end

-- ---------------------------------------------------------------------------
-- Map 0 (Eastern Kingdoms)
-- ---------------------------------------------------------------------------
local pf0 = PathFinder.new(DATA_DIR, 0)
if not pf0:isValid() then
    print("SKIP  map 0 — navmesh failed to load")
else
    -- Two nearby points in open terrain; reference distance from C++ binary = 27.2250
    check("map0 nearby open terrain",
          pf0:computeDistance(-9095, 314, -9117, 330), 27.2250, 0.5)
end

-- ---------------------------------------------------------------------------
-- Summary
-- ---------------------------------------------------------------------------
print(string.format("\n%d passed, %d failed", passed, failed))
if failed > 0 then os.exit(1) end
