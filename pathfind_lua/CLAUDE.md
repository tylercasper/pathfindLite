# pathfind_lua Optimization Session

## Project Overview
Lua port of CMaNGOS C++ Detour/Recast pathfinding (`PathFinder.cpp`). Goal: minimize `computeDistance` latency on **standard Lua 5.1** (non-JIT), matching the WoW addon environment.

---

## Benchmark Commands

**Primary benchmark (Lua 5.1, non-JIT) — USE THIS:**
```bash
sudo -n ./bench_lua51.sh 50    # preferred: elevated priority, 50 runs
./bench_lua51.sh 50            # fallback if sudo unavailable
```
- Interpreter: `lua` from PATH (`/usr/local/bin/lua`, Lua 5.1.5)
- Bitop extension: `/usr/local/lib/lua/5.1/bit.so` (found via default `LUA_CPATH`)
- When run with `sudo`, each `lua` invocation is wrapped with
  `taskpolicy -a nice -n -20` for reduced OS scheduling jitter.
- **Always use 50-run averages for decisions.** Single-run variance is ~±10%.
- The script prints a correctness check (mismatch count) before timing runs.
- `sudo -n` requires passwordless sudo to be configured for `bench_lua51.sh`.

**DO NOT use `bench10.sh` for optimization decisions.**
`bench10.sh` runs LuaJIT (JIT-compiled). JIT performance does not reflect the
standard Lua 5.1 interpreter. Keep it as a sanity-check guard only.

---

## Established Baselines

| Version | Lua 5.1 (ms) | Notes |
|---------|-------------|-------|
| HEAD (389f7ab, unoptimized) | ~TBD | goto-only fix needed to run; see BENCHMARKS.md |
| After sessions 1–5 (18542f4) | **13,182 ms** | 10-run avg (old environment) |
| Test AV (a02a4e9, current HEAD) | **TBD** | 50-run avg with sudo; new baseline pending |

**8 pre-existing mismatches** (pairs 25, 29, 33, 37, 41, 45, 47, 90) are
floating-point precision differences vs C++, NOT regressions.
Always verify mismatch count stays at 8 after any change.

---

## Key Constraints

1. **NO FOR LOOPS IN BASH COMMANDS** — always write a script file instead.
2. **findPath local limit (LuaJIT only, not a concern for Lua 5.1):** On LuaJIT,
   `findPath` already has ~30+ locals; adding more causes JIT register spills.
   On the Lua 5.1 interpreter there is no register-spill concern — feel free to
   add locals to reduce repeated field lookups.
3. **Single-threaded:** Module-level pre-allocated buffers are safe.
4. **Benchmark results go in BENCHMARKS.md.** Always record before/after timings
   and analysis. Use 50-run averages for decisions.

---

## Benchmark Results File

All optimization attempts are recorded in `BENCHMARKS.md` in this directory.
Format: each entry has:
- What changed
- Before / after timing (ms total for 96 pairs, multi-run average preferred)
- Match count (should be 88/96 — 8 pre-existing mismatches are expected)
- Why it succeeded or failed

---

## Architecture

| File | Role |
|------|------|
| `NavMesh.lua` | dtNavMesh: BVH queries, tile management, polygon data |
| `NavMeshQuery.lua` | dtNavMeshQuery: A* pathfind, string path, nearest poly |
| `PathFinder.lua` | High-level API: load tiles, terrain heights, computeDistance |
| `TerrainMap.lua` | .map terrain height lookup |
| `bench.lua` | Benchmark vs C++ binary, prints timing table |
| `bench_lua51.sh` | **Primary** multi-run script (Lua 5.1 interpreter) |
| `bench10.sh` | LuaJIT multi-run script — guard use only, not for decisions |
| `orderings.lua` | 96 coordinate pairs for benchmarking |

## Coordinate Convention
WoW (x, y, z) → Recast (y, z, x). Navmesh tiles addressed as (tx, ty) where `tx = floor(32 - x/533.333)`.

---

## Optimization Strategy

### Primary Targets (in order of impact for Lua 5.1 interpreter)
1. **Table allocation in hot loops** — every `{x,y,z}` creation costs GC time;
   use pre-allocated module-level buffers and write into them in-place
2. **Repeated field lookups** — cache `self.field` as a local before loops;
   GETTABLE bytecodes are expensive in the interpreter
3. **Function call overhead** — method calls (GETTABLE + CALL + RETURN) are
   expensive; inline hot helpers where the code stays readable
4. **Algorithmic complexity** — O(n) vs O(1) matters more without JIT optimizing
   away small loops (indexed heap, hash maps, etc.)

### Known Hot Paths (from LuaJIT profiler — Lua 5.1 distribution may differ)
- `getNode` (22%) → hash chain traversal per A* node lookup
- `findPath` (21%) → A* outer + inner loops
- `_trickleDown` / `_bubbleUp` (4–8% each) → heap operations
- `_writeMidPoint` (6%) → per A* edge expansion
- `connectExtLinks` / `findConnectingPolys` (5–7% each) → tile loading

---

## Bisect / Regression Protocol

When a change causes a regression:
1. Run 50 benchmark runs (`sudo -n ./bench_lua51.sh 50`) to confirm
2. Check `BENCHMARKS.md` for the last known-good state
3. Revert the suspected change and re-benchmark
4. Document finding in `BENCHMARKS.md`
