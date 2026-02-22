#!/bin/bash
# Runs bench.lua N times and reports per-run and average Lua compute time.
cd "$(dirname "$0")"
N=${1:-10}
total=0
i=1
while [ "$i" -le "$N" ]; do
    result=$(luajit bench.lua orderings.lua 2>/dev/null | tail -6)
    echo "=== Run $i ==="
    echo "$result"
    ms=$(echo "$result" | grep "compute:" | head -1 | grep -oE '[0-9]+\.[0-9]+ ms total' | grep -oE '[0-9]+\.[0-9]+')
    total=$(echo "$total + ${ms:-0}" | bc)
    i=$((i + 1))
done
avg=$(echo "scale=1; $total / $N" | bc)
echo ""
echo "=== Average Lua compute over $N runs: ${avg} ms total ==="
