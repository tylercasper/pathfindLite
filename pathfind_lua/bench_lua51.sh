#!/bin/bash
# Runs bench.lua N times using the standard Lua 5.1 interpreter and reports per-run and average compute time.
# Resolves symlinks so this script works when invoked via a symlink from another directory.
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do
    DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
    SOURCE="$(readlink "$SOURCE")"
    [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done
cd "$(cd -P "$(dirname "$SOURCE")" && pwd)"

N=${1:-5}

if [ "$(id -u)" = "0" ]; then
    LUA=(/usr/sbin/taskpolicy -a /usr/bin/nice -n -20 -- lua)
else
    LUA=(lua)
fi

mismatches=$("${LUA[@]}" bench.lua orderings.lua --mismatches 2>/dev/null)
echo "=== Correctness: $mismatches ==="

total=0
i=1
while [ "$i" -le "$N" ]; do
    result=$("${LUA[@]}" bench.lua orderings.lua --timing 2>/dev/null)
    echo "=== Run $i ==="
    echo "$result"
    ms=$(echo "$result" | grep "compute:" | head -1 | grep -oE '[0-9]+\.[0-9]+ ms total' | grep -oE '[0-9]+\.[0-9]+')
    total=$(echo "$total + ${ms:-0}" | bc)
    i=$((i + 1))
done
avg=$(echo "scale=1; $total / $N" | bc)
echo ""
echo "=== Average Lua 5.1 compute over $N runs: ${avg} ms total ==="
