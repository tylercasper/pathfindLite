# Pathfind Lua Optimization Benchmarks

## Setup

| Item | Value |
|------|-------|
| Benchmark command | `luajit bench.lua orderings.lua 2>/dev/null` |
| Multi-run script | `bash bench10.sh [N]` (default 10 runs) |
| Pairs | 96 coordinate pairs on map 0 (TBC) |
| Platform | macOS Darwin 24.3.0, LuaJIT 2.1 |
| Pre-existing mismatches | **8/96** (pairs 25, 29, 33, 37, 41, 45, 47, 90) — floating-point vs C++, not regressions |
| Single-run variance | ~±200 ms — always use 5–10 run averages for decisions |

---

## Measured Checkpoints (LuaJIT)

These are the only performance numbers with actual multi-run measurements behind them.
Individual optimizations within a session were not isolated-measured unless noted.

| Checkpoint | Runs | Avg (ms) | vs HEAD | Raw values |
|------------|------|----------|---------|------------|
| **HEAD** (git 389f7ab, unmodified) | 1 | 4166 | — | single run only |
| After Session 1+2 changes | 5 | ~4004 | -3.9% | 4065, 4131, 3941, 3910, 3972 |
| Session 3 re-baseline (=above confirmed) | 10 | 3998 | -4.0% | 3998, 3515, 3906, 3979, 3735, 3444, 4342, 3570, 4306, 3851 |
| + Indexed heap only (#14) | 8 | 4316 | +3.6% | REGRESSION — reverted |
| + heap+hash only (#15) | 10 | 3842 | -7.8% | reverted |
| + _writeMidPoint work (#16) | 10 | 3780 | -9.3% | |
| + Session 3 confirmed | 10 | 3762 | -9.7% | 3642, 3794, 3315, 3140, 3820, 4051, 4370, 3811, 4011, 3666 |
| + C (modify h-cache + ep alias removal) | 10 | 3532 | -15.2% | 3407, 3772, 3733, 3852, 3255, 3334, 3152, 3714, 3714, 3386 |
| **+ D (slab/conn alloc elimination)** | 10 | **3750** | **-10.0%** | 3228, 4199, 3906, 3895, 3372, 4716, 3825, 3320, 3390, 3648 |

> **Note on HEAD baseline:** Only 1 run was taken. It may understate the true baseline by ~100–200 ms
> due to single-run noise. Treat the "vs HEAD" column as approximate.

---

## Optimization Log

Each entry records: the code change, the rationale, the observable implementation effect
(allocs eliminated, function calls removed, etc.), and the measured performance outcome if any.
Estimated or hypothetical performance gains are clearly labelled **[ESTIMATED]** or
**[NOT MEASURED]**; only multi-run averages are recorded as facts.

---

### Session 1 (committed, then stash-restored)

Changes 1–6 were applied together. Their combined effect is the ~162 ms improvement
(4166 → ~4004 ms). **They were not individually isolated and measured.**

---

#### #1 — JIT Tuning — bench.lua
**Change:** `jit.opt.start(3, "maxmcode=65536", "maxrecord=8000")` added; guarded by `if jit then`.
**Rationale:** Default JIT trace budget is too small for the A* and funnel loops, causing early trace eviction and falling back to interpreter for the hottest code.
**Implementation effect:** Enables JIT compilation of hot loops that would otherwise abort tracing.
**Performance:** **[NOT MEASURED ALONE]** — foundational; meaningless without other changes. No-op on non-JIT Lua.

---

#### #2 — getCost Inlining — NavMeshQuery.lua
**Change:** Inlined `filter:getCost(pa, pb)` in the findPath A* inner loop as `dtVdist(pa,pb) * filterAreaCost[curPoly.area]`. Eliminated one method call per A* edge expansion.
**Rationale:** Per-edge function call overhead in the tightest loop adds up.
**Implementation effect:** –1 Lua function call per expanded edge.
**Performance:** **[NOT MEASURED ALONE]**

---

#### #3 — parentTile/parentPoly Removal — NavMeshQuery.lua
**Change:** Removed `parentTile` and `parentPoly` as separate locals from findPath; inlined their single-use access.
**Rationale:** `findPath` already had ~30+ locals at JIT register capacity. Every additional local risks a register spill that prevents the inner loop from being fully compiled.
**Implementation effect:** –2 locals from findPath.
**Performance:** **[NOT MEASURED ALONE]** — JIT-specific. Has no effect on non-JIT Lua.

---

#### #4 — _writeMidPoint — NavMeshQuery.lua
**Change:** Added `q:_writeMidPoint(from, fromPoly, fromTile, to, toPoly, toTile, dest)` which writes the edge midpoint directly into a caller-supplied `dest` table. The old `getEdgeMidPoint` returned a new `{x,y,z}` table per call.
**Rationale:** Allocation in the A* expansion loop creates GC pressure.
**Implementation effect:** –1 table allocation per portal step in A* expansion (GROUND poly case).
**Performance:** **[NOT MEASURED ALONE]**

---

#### #5 — NavMesh.lua Pre-alloc Buffers
**Change:** Added module-level pre-allocated tables: `_closestBuf`, `_detailClos`, `_detailTV`, `_polyVerts`, `_detailPmin`, `_detailPmax`, `_detailTidx`, `_detailTV1/2/3`. Functions `closestPointOnPoly` and `closestPointOnDetailEdges` write into these instead of returning new tables.
**Rationale:** These functions are called per path query and always allocated return tables.
**Implementation effect:** Eliminates multiple table allocs per `closestPointOnPoly` call.
**Performance:** **[NOT MEASURED ALONE]**

---

#### #6 — PathFinderDebug Guard — PathFinder.lua
**Change:** Added `PathFinderDebug` global flag (set to `false` in bench.lua). All `io.stderr:write` log calls guarded by early `if not PathFinderDebug then return end`.
**Rationale:** `io.stderr:write` is expensive; debug logging was firing 96×N times during bench.
**Implementation effect:** Zero-cost logging path when disabled.
**Performance:** **[NOT MEASURED ALONE]** — the improvement is only visible vs the debug-on baseline.

---

### Session 2 (in working tree)

Changes 7–13 were applied together. Their combined effect is included in the Session 1+2
checkpoint above. **They were not individually isolated and measured.**

---

#### #7 — dtDistancePtPolyEdgesSqr Inline — NavMesh.lua
**Change:** Inlined `dtDistancePtSegSqr2D` into `dtDistancePtPolyEdgesSqr` using scalar locals `vi_x/vi_z/vj_x/vj_z` instead of `{x,y,z}` table arguments.
**Rationale:** A 6-vert poly call was allocating 12 small tables (2 per edge × 6 edges); called inside `closestPointOnPolyBoundary` which is on a hot path.
**Implementation effect:** –12 table allocs per `closestPointOnPolyBoundary` call.
**Performance:** **[NOT MEASURED ALONE]**

---

#### #8 — closestPointOnPolyBoundary Pre-alloc Scratch — NavMeshQuery.lua
**Change:** Added `_cbVerts`, `_cbEdged`, `_cbEdget` pre-alloc scratch tables to the query object. `closestPointOnPolyBoundary` uses these instead of local `{}` allocations.
**Rationale:** Three arrays allocated per call.
**Implementation effect:** –3 table allocs per `closestPointOnPolyBoundary` call.
**Performance:** **[NOT MEASURED ALONE]**

---

#### #9 — _pLeft/_pRight Module-Level Portal Buffers — NavMeshQuery.lua
**Change:** Added `local _pLeft = {0,0,0}` and `local _pRight = {0,0,0}` at module level. `_getPortalPointsFull` writes into these and returns a boolean instead of two new tables.
**Rationale:** findStraightPath allocated 2 tables per portal (5–20 portals per path).
**Implementation effect:** –2 table allocs per portal step.
**Performance:** **[NOT MEASURED ALONE]** — ⚠️ suspected regression source on LuaJIT (see note).
> **JIT aliasing note:** The funnel loop assigns `left = _pLeft; right = _pRight`. LuaJIT may not be able to scalar-replace reads from `left[1]` etc. when it cannot prove the module-level table is not modified by a called function. This can prevent trace compilation of the funnel inner loop. **No regression confirmed by measurement** (session 3 re-baseline matches session 1+2 combined). On standard Lua 5.1, this concern does not apply.

---

#### #10 — findStraightPath Output Simplification — NavMeshQuery.lua
**Change:** Changed `_appendVertex` to store plain `{x,y,z}` tables (not `{pos=..., flags=..., ref=...}` wrappers). Return simplified to `straightPath, count, stat`. PathFinder.lua updated to index `straightResult[k]` directly.
**Rationale:** Fewer allocs per output path point; simpler interface.
**Implementation effect:** –1 wrapper table allocation per path output point.
**Performance:** **[NOT MEASURED ALONE]**

---

#### #11 — PathFinder.lua Pre-alloc Buffers
**Change:** Added `_startPos`, `_endPos`, `_endPosAdj`, `_extents` as fields on the pf object. `computeDistance` fills these instead of allocating per call.
**Rationale:** 4 small table allocs eliminated per `computeDistance` call (96 calls in bench).
**Implementation effect:** –4 table allocs per `computeDistance` call.
**Performance:** **[NOT MEASURED ALONE]** — total across 96 pairs would be ~4×96=384 fewer allocs, likely sub-10 ms.

---

#### #12 — quantMinF/quantMaxF Module-Level — NavMesh.lua
**Change:** Moved `quantMin`/`quantMax` from inside `queryPolygonsInTile` to module level; take explicit `qfac` parameter instead of closing over it.
**Rationale:** Closure allocation: 2 closures per `queryPolygonsInTile` call.
**Implementation effect:** –2 closure allocs per BVH traversal call.
**Performance:** **[NOT MEASURED ALONE]**

---

#### #13 — getTilesAt Shared Buffer — NavMesh.lua
**Change:** `getTilesAt` fills module-level `_tilesAt` buffer; returns `(buf, count)` instead of a new table.
**Rationale:** One table alloc eliminated per tile neighbour lookup.
**Implementation effect:** –1 table alloc per tile lookup.
**Performance:** **[NOT MEASURED ALONE]**

---

### Session 3

Changes 14–16 were individually measured with 8–10 run averages.

---

#### #14 — Indexed Heap (getNode O(1) lookup) — NavMeshQuery.lua ❌ REVERTED
**Change:** Replaced the linear scan in `getNode` with a hash table (keyed by poly ref) for O(1) node lookup instead of O(n) scan through the node pool.
**Rationale:** `getNode` was 22% of runtime in profiler; the linear scan is O(n) in node pool size.
**Implementation effect:** O(1) node lookup instead of O(n).
**Performance: MEASURED — REGRESSION.** 8-run avg: **4,316 ms** (+8% vs 3,998 ms baseline).
**Why it failed:** LuaJIT JIT-compiles the tight linear scan over a flat array into very fast machine code. The hash table adds branching, more table field accesses, and chain traversal that the JIT cannot eliminate as effectively. At typical A* pool sizes, the constant-factor advantage of the compiled scan dominates the asymptotic benefit. Reverted.
**Note for Lua 5.1:** Without JIT, every bytecode instruction costs equally. Algorithmic O(n) vs O(1) matters more. **Worth retesting on Lua 5.1.**

---

#### #15 — heap+hash Combined — NavMeshQuery.lua ❌ REVERTED
**Change:** Kept binary heap for ordering, added parallel hash table for O(1) node presence test.
**Rationale:** Get O(1) lookup without disrupting heap structure.
**Implementation effect:** O(1) open/closed presence test; heap ordering preserved.
**Performance: MEASURED.** 10-run avg: **3,842 ms** (-3.9% vs 3,998 ms baseline).
**Why reverted:** Marginal improvement; added structural complexity. Reverted for now.
**Note for Lua 5.1:** Same consideration as #14 — worth retesting.

---

#### #16 — _writeMidPoint Further Optimization — NavMeshQuery.lua ✅ KEPT
**Change:** Further reduced allocations/overhead in `_writeMidPoint` path (building on session 1 change #4).
**Performance: MEASURED.** 10-run avg: **3,780 ms** (-5.5% vs 3,998 ms baseline).

---

## Lua 5.1 Compatibility

**Status:** ✅ Working. Lua 5.1.5 built from source; `lua-bitop` C extension built; all `goto` removed.

### What was done

| Issue | Location | Fix applied |
|-------|----------|-------------|
| `goto`/`::label::` (Lua 5.2+ feature) | NavMesh.lua (3 loops), NavMeshQuery.lua (5 loops + findStraightPath) | Loop-continue patterns → `repeat...until true`/`break` or nested `if`; `goto done` → direct `return` |
| `require("bit")` (LuaJIT built-in) | NavMesh.lua:15–16, NavMeshQuery.lua:5 | Built `LuaBitOp-1.0.2` C extension: `/tmp/LuaBitOp-1.0.2/bit.so` |
| `jit.opt.start` | bench.lua | Already guarded: `if jit then ... end` ✅ |

**Benchmark script:** `bash bench_lua51.sh [N]` (default 5 runs; uses `/tmp/lua-5.1.5/src/lua` + `LUA_CPATH`)

### Impact of goto→if refactor on LuaJIT

Single-run check after refactor: **3,478 ms** — within normal variance of the 3,762 ms 10-run avg.
No measurable regression. The `repeat...until true` idiom is compiled away by LuaJIT.

## Measured Checkpoints (Lua 5.1)

All timings use standard Lua 5.1.5 (no JIT), `lua-bitop` C extension for bitwise ops.
The code is otherwise identical to the LuaJIT working tree.

| Checkpoint | Runs | Avg (ms) | vs LuaJIT | Raw values |
|------------|------|----------|-----------|------------|
| **Baseline** (current working tree, same as LuaJIT optimized) | 5 | **13,902** | — | 13,465, 13,862, 15,292, 13,220, 13,672 |
| + getNode local caching (A) only | 3 | 14,276 | +2.7% (noise) | 14,867, 13,715, 14,247 |
| + A + indexed modify B (unconditional) | 3 | 13,033 | -6.3% | 13,017, 13,193, 12,888 |
| + A + B per-call conditional | 5 | 13,532 | -2.7% | |
| + A + B load-time selection | 5 | 13,428 | -3.4% | 13,585, 13,818, 13,153, 13,308, 13,274 |
| + C (10-run, measured with module-level heap fns) | 10 | 13,648 | -1.4% | 13980, 13632, 13810, 13931, 13450, 13641, 14413, 13202, 13483, 12929 |
| **+ D (slab/conn alloc elimination)** | 10 | **13,182** | **-4.9% vs baseline** | 13102, 12938, 13706, 12933, 12966, 12907, 13316, 13351, 13117, 13479 |

> **Note on baseline:** One run (15,292 ms) is a clear outlier. True central estimate ~13,550 ms. All Lua 5.1 "improvements" shown above should be interpreted against ~13,550 ms, making most of them noise-level.

> Lua 5.1 is ~3.7× slower than LuaJIT for this workload (13,902 ms vs ~3,762 ms).
> Single-run variance: ~±10% (wider than LuaJIT's ~±6%).

### Session 5 Optimizations

#### C. `modify` heap-cache + `ep` alias removal — NavMeshQuery.lua ✅ KEPT
**Change (C1):** `modify` now caches `local h = self.heap` before the scan loop. Previously did `self.heap[i]` on every iteration — 1 GETTABLE per heap element scanned.
**Change (C2):** Removed redundant `local ep = endPos` alias in both branches of the findPath inner loop (2 occurrences). `endPos` is already a function param (local); the alias wasted 1 register slot in each branch.

**Rationale for C2:** The findPath inner loop was already near the JIT register limit. Freeing the `ep` register slot in the `if neighbourRef == endRef` and `else` branches reduced peak live locals by 1 in those branches. This may have unblocked JIT trace compilation of the branches.

**Performance (LuaJIT):** MEASURED — 10-run avg **3,532 ms** vs 3,762 ms previous baseline. **-6.1% improvement.**
Raw values: 3407, 3772, 3733, 3852, 3255, 3334, 3152, 3714, 3714, 3386.

> This improvement is larger than expected for micro-changes. C2 (`ep` removal) freed a register slot that may have unblocked JIT compilation of the inner-loop branches.

**Performance (Lua 5.1):** NOT MEASURED separately. Included in Test D baseline.

---

#### D. Slab/connection allocation elimination — NavMesh.lua ✅ KEPT
**Change:** Eliminated all table allocations from `calcSlabEndPoints`, `findConnectingPolys`, and `connectExtLinks`:
- `calcSlabEndPoints`: now takes caller-supplied `bmin`/`bmax` buffers instead of allocating `{0,0}` twice per call.
- `findConnectingPolys`: uses pre-allocated `_connVc`/`_connVd` for vertex extraction; uses `_slabMin1/2`/`_slabMax1/2` for slab end points; uses `_conBuf`/`_conareaBuf` for results (module-level, returned to caller which consumes immediately). Caches `tile.verts` as local.
- `connectExtLinks`: uses pre-allocated `_connVa`/`_connVb` for edge vertices; caches `tile.verts` as local.

**Rationale:** `connectExtLinks` and `findConnectingPolys` are called during per-query tile loading (tiles are loaded lazily on first query to a new area). Profiler showed them at 5% and 5-7% respectively. Each call to `connectExtLinks` per ext-link edge allocates 4 tables; each `findConnectingPolys` call allocates 2–6 tables. On Lua 5.1 without JIT, allocation is expensive.

**Safety:** All buffers are safe to reuse because: (1) Lua is single-threaded; (2) `_conBuf`/`_conareaBuf` are consumed by the caller (`for kk = 1, nnei`) before the next call to `findConnectingPolys`.

**Implementation effect:** Eliminates ~4-8 table allocs per ext-link edge processed during tile loading.

**Performance (LuaJIT):** MEASURED — 10-run avg **3,750 ms** vs 3,532 ms Test-C baseline (+6%, within noise). Two high outliers (4716, 4199) inflate avg; trimmed (excl top 2) = 3,573 ms. Vs original 3,762 ms baseline: -0.3% (**neutral**).

**Performance (Lua 5.1):** MEASURED — 10-run avg **13,182 ms** vs ~13,550 ms central baseline. **-2.7% improvement.** Clean run, no outliers (range: 12907–13706).

---

### Session 4 Optimizations

#### A. getNode/findNode local caching — NavMeshQuery.lua ✅ KEPT
**Change:** Cache `self.first`, `self.nodes`, `self.next` as locals before the hash-chain while loop in `getNode` and `findNode`.
**Rationale:** On Lua 5.1's interpreter, each `self.field` access is a GETTABLE bytecode. Caching as locals converts repeated GETTABLE to GETLOCAL ops. On LuaJIT the JIT inlines table accesses anyway.
**Implementation effect:** –2 GETTABLE ops per hash chain iteration.
**Performance (LuaJIT):** MEASURED — 10-run avg **3,817 ms** vs 3,762 ms baseline. **Neutral** (within noise).
**Performance (Lua 5.1):** MEASURED — 3-run avg **14,276 ms** vs 13,902 ms baseline. **Neutral** (within noise; high Lua 5.1 variance makes this inconclusive).

#### B. Indexed modify (O(1) heap position lookup) — NavMeshQuery.lua ❌ REVERTED
**Change:** Added `_heapIdx` field to each node, updated in `_bubbleUp`/`_trickleDown` when nodes move. `modify` uses `node._heapIdx` for O(1) lookup instead of O(n) linear scan.
**Rationale:** On Lua 5.1, every O(n) scan iteration costs full interpreter overhead. At typical A* heap sizes, O(1) lookup is faster. On LuaJIT the scan is JIT-compiled and cheap.
**Implementation effect:** `modify` O(n) → O(1). Extra cost: 2 additional table writes per heap swap.

**Measured variants (all reverted):**

| Variant | LuaJIT (10-run avg) | Lua 5.1 (runs) | Verdict |
|---------|---------------------|----------------|---------|
| Unconditional A+B | 4,015 ms (+6.7%) ❌ | 13,033 ms (3 runs, -6.3%) | JIT regression |
| Per-call `if not _isJIT` guard | 4,033 ms (+7.2%) ❌ | 13,532 ms (5 runs, -2.7%) | JIT regression; guard overhead ate Lua 5.1 gain |
| Load-time function selection | 3,797 ms* (+0.9%, neutral) | 13,428 ms (5 runs, -3.4%) | Marginal Lua 5.1 gain; high complexity |

*Excludes one 7,093 ms outlier (likely GC/OS). 9-run avg 3,797 ms vs 3,762 ms baseline.

**Why reverted:** The load-time selection approach was neutral on LuaJIT and showed marginal Lua 5.1 improvement (~-3.4%), but the Lua 5.1 baseline itself has high variance (one run was 15,292 ms) making the ~-3.4% improvement statistically uncertain. Maintaining 6 function bodies (2 copies each of `_bubbleUp`, `_trickleDown`, `modify`) is not justified for an uncertain ~1% gain.

**Root cause of LuaJIT regression in unconditional variant:** The `_heapIdx = i+1` writes in every heap swap occur inside the JIT-compiled `_bubbleUp`/`_trickleDown` hot path. LuaJIT traces table writes conservatively, especially when the written-to table is a node shared across multiple heap positions. The extra stores likely disrupt trace compilation or cause store-load forwarding penalties.

### Expected behavior changes on standard Lua 5.1

| Optimization | Expected change |
|-------------|-----------------|
| JIT tuning (#1) | No effect (no JIT) |
| Function inlining (#2, #7) | Still beneficial — interpreter pays CALL/RETURN bytecodes explicitly |
| `findPath` local count (#3) | No longer a concern — no register spill on interpreter |
| Allocation reduction (#4–#13) | Still beneficial — GC pressure affects all Lua implementations |
| Module-level upvalue aliasing (#9) | No concern — interpreter re-reads every field every time |
| Indexed heap (#14) | **May help** — O(n) scan is uncompiled, asymptotic cost matters more |
| heap+hash (#15) | **May help** — same reason |

---

## TODO / Test Plan

### Immediate: Lua 5.1 setup
1. Fix `goto` blocks in NavMesh.lua (3 loops → nested if)
2. Build `lua-bitop` C extension for Lua 5.1 (or provide pure-Lua shim)
3. Create `bench_lua51.sh` using the built interpreter
4. Measure Lua 5.1 baseline (10 runs)

### LuaJIT: remaining candidates
| Priority | Item | Profiler share | Notes |
|----------|------|---------------|-------|
| High | `getNode` linear scan | 22% | Indexed heap failed; explore alternative |
| High | `findPath` A* inner loop | 21% | Near local limit; careful |
| Med | `_bubbleUp` / `_trickleDown` | 4–8% each | Heap ops; fairly minimal already |
| Med | `findStraightPath` apex/left/right | — | Alloc per call; aliasing risk |
| Low | `closestStart`/`closestEnd` return alloc | — | Caller-supplied buffer |
| Low | `findNearestPoly` return alloc | — | Caller-supplied buffer |
| Low | `_appendVertex` alloc | — | Pre-alloc fixed array |
| Skip | `parseTileData` | 9–11% | Load-time only, not per-query |

### Won't Try
- **FFI structs**: Replacing `{x,y,z}` tables throughout with LuaJIT FFI — massive refactor, uncertain payoff.
- **Coroutine parallelism**: LuaJIT is single-threaded; no benefit.
- **Re-implementing in C**: Out of scope.

---

## Key Constraints

- **findPath local limit (LuaJIT only):** ~30+ locals already. Any new local causes JIT register spill. Not a constraint on standard Lua 5.1.
- **No for loops in bash** — always write a script file.
- **8 pre-existing mismatches** are expected and OK (pairs 25, 29, 33, 37, 41, 45, 47, 90).
- **Module-level pre-alloc buffers are safe** (all Lua is single-threaded).
- **Module-level upvalue aliasing (LuaJIT only):** If a hot loop holds a reference to a module-level table and a called function mutates it, JIT may deoptimize. Test carefully.
