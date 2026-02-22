#!/bin/bash
# Runs bench.lua N times using the standard Lua 5.1 interpreter and reports per-run and average compute time.
cd "$(dirname "$0")"
N=${1:-5}
LUA51=/tmp/lua-5.1.5/src/lua
LUA_CPATH="/tmp/LuaBitOp-1.0.2/?.so"
export LUA_CPATH
total=0
i=1
while [ "$i" -le "$N" ]; do
    result=$(${LUA51} bench.lua orderings.lua 2>/dev/null | tail -6)
    echo "=== Run $i ==="
    echo "$result"
    ms=$(echo "$result" | grep "compute:" | head -1 | grep -oE '[0-9]+\.[0-9]+ ms total' | grep -oE '[0-9]+\.[0-9]+')
    total=$(echo "$total + ${ms:-0}" | bc)
    i=$((i + 1))
done
avg=$(echo "scale=1; $total / $N" | bc)
echo ""
echo "=== Average Lua 5.1 compute over $N runs: ${avg} ms total ==="
