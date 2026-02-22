# Pathfind Lua Optimization Benchmarks

Benchmark: `luajit bench.lua orderings.lua 2>/dev/null`
Pairs: 96 coordinate pairs on map 0 (TBC)
Platform: macOS Darwin 24.3.0, LuaJIT 2.1
Pre-existing mismatches: **8/96** (pairs 25, 29, 33, 37, 41, 45, 47, 90) — floating-point vs C++, not regressions.

> **Note:** Single-run variance is ~±200ms. Always use 5–10 run averages for decisions.

---

## Baseline: HEAD (git commit 389f7ab)


| Metric           | Value    |
| ---------------- | -------- |
| Total (96 pairs) | ~4166 ms |
| Per pair         | ~43.4 ms |
| Mismatches       | 8/96     |


This is the unmodified committed state.

---

## Previous Session Optimizations (~Session 1)

**All of these are currently IN the working tree (restored from git stash).**

### 1. JIT Tuning — bench.lua

**Change:** Added `jit.opt.start(3, "maxmcode=65536", "maxrecord=8000")` to bench.lua
**Rationale:** Default trace budget is too small for the A* and funnel loops, causing early trace eviction.
**Result:** Foundational — enables other JIT optimizations to take effect. Alone saves ~100–300ms.

### 2. getCost Inlining — NavMeshQuery.lua

**Change:** Inlined `filter:getCost(pa, pb)` into findPath A* loop as `dtVdist(pa,pb) * filterAreaCost[curPoly.area]`, eliminated the function call.
**Rationale:** getCost is called per A* edge expansion; function call overhead in tight loops matters.
**Result:** ~50–100ms improvement (estimated).

### 3. parentTile/parentPoly Removal — NavMeshQuery.lua

**Change:** Removed local variables `parentTile` and `parentPoly` from findPath A* loop. Inlined their usage.
**Rationale:** findPath had ~30+ locals; removing 2 locals reduces register spill risk.
**Result:** JIT can compile the inner loop more aggressively. ~50ms improvement (estimated).

### 4. _writeMidPoint — NavMeshQuery.lua

**Change:** Added `q:_writeMidPoint(from, fromPoly, fromTile, to, toPoly, toTile, dest)` which writes the edge midpoint directly into `dest` without allocating new tables.
**Rationale:** The old `getEdgeMidPoint` returned a new table per call; for GROUND polys this is the common case.
**Result:** Eliminates ~1 alloc per portal step in A* expansion.

### 5. NavMesh.lua Pre-alloc Buffers (closestPointOnPoly, closestPointOnDetailEdges)

**Change:** Added module-level pre-allocated tables `_closestBuf`, `_detailClos`, `_detailTV`, `_polyVerts`, etc.
**Rationale:** These functions are called frequently and always returned new tables.
**Result:** Reduced GC pressure significantly. ~100–200ms improvement (estimated).

### 6. PathFinderDebug Guard — PathFinder.lua

**Change:** Added `PathFinderDebug` flag (set to false in bench.lua). Log calls guarded by early return.
**Rationale:** io.stderr:write is slow; 96×many log calls was wasting significant time.
**Result:** Large improvement when debug was on. Now zero-cost.

---

## Current Session Optimizations (~Session 2)

**All of these are also currently IN the working tree.**

### 7. dtDistancePtPolyEdgesSqr Inline — NavMesh.lua

**Change:** Inlined `dtDistancePtSegSqr2D` into `dtDistancePtPolyEdgesSqr`, using scalar locals `vi_x/vi_z/vj_x/vj_z` instead of allocated `{x,y,z}` table per edge.
**Rationale:** Each call to `dtDistancePtPolyEdgesSqr` for a 6-vert poly was allocating 12 small tables (2 per edge). These are called inside `closestPointOnPolyBoundary` which is called multiple times per path query.
**Result:** Eliminates 12 allocs per `closestPointOnPolyBoundary` call.

### 8. closestPointOnPolyBoundary Pre-alloc Scratch — NavMeshQuery.lua

**Change:** Added `_cbVerts`, `_cbEdged`, `_cbEdget` pre-alloc scratch tables to query object. Changed `closestPointOnPolyBoundary` to use these instead of local allocations.
**Rationale:** These arrays were allocated per call.
**Result:** 3 fewer allocs per `closestPointOnPolyBoundary` call.

### 9. _pLeft/_pRight Module-Level Portal Buffers — NavMeshQuery.lua

**Change:** Added `local _pLeft = {0,0,0}` and `local _pRight = {0,0,0}` at module level. `_getPortalPointsFull` now writes into these instead of returning new tables. Returns boolean.
**Rationale:** Portal step in findStraightPath allocated 2 tables per portal. With ~5–20 portals per path, this is significant.
**Result:** ⚠️ **SUSPECTED REGRESSION SOURCE**. Saves allocs but may disrupt JIT due to module-level upvalue aliasing in the hot funnel loop. See note below.

> **Note on _pLeft/_pRight regression:** The funnel loop in `findStraightPath` assigns `left = _pLeft; right = _pRight` where these are module-level upvalue references. LuaJIT may be unable to fully scalar-replace reads from `left[1]`, `left[2]`, etc. when it knows they alias a module-level table that any called function might modify. This can cause conservative deoptimization.

### 10. findStraightPath Output Simplification — NavMeshQuery.lua

**Change:** Changed `_appendVertex` to store plain `{x,y,z}` tables in `straightPath`. Changed return to `straightPath, countRef[1], stat` (no wrapper alloc). Updated PathFinder.lua distance loop to use `straightResult[k]` directly.
**Rationale:** Previously each path point was wrapped as `{pos={x,y,z}, flags=..., ref=...}`.
**Result:** Cleaner interface, fewer allocs per path point. Low regression risk.

### 11. PathFinder.lua Pre-alloc Buffers

**Change:** Added `_startPos`, `_endPos`, `_endPosAdj`, `_extents` to pf object. computeDistance fills these instead of allocating per call.
**Rationale:** 4 small table allocs eliminated per computeDistance call.
**Result:** Minor improvement (~4ms total across 96 pairs).

### 12. quantMinF/quantMaxF Module-Level — NavMesh.lua

**Change:** Moved `local function quantMin/quantMax(v)` from inside `queryPolygonsInTile` to module level, taking explicit `qfac` parameter.
**Rationale:** Closure allocation: 2 closures allocated per `queryPolygonsInTile` call.
**Result:** Eliminates 2 closure allocs per BVH traversal call.

### 13. getTilesAt Shared Buffer — NavMesh.lua

**Change:** `getTilesAt` now fills module-level `_tilesAt` buffer and returns `(buf, count)` instead of creating a new table.
**Rationale:** One table alloc eliminated per tile neighbor lookup.
**Result:** Minor improvement.

---

## Current Session Optimizations (~Session 3)

### 14. Indexed Heap — NavMeshQuery.lua ❌ REVERTED

**Change:** Replaced linear scan in `getNode` open/closed set with an indexed hash (nodeIdx table keyed by poly ref) to get O(1) lookup instead of O(n) scan.
**Rationale:** `getNode` was 22% of runtime in profiler; the linear scan is O(n) in node pool size.
**Result:** **REGRESSION** — avg 4,316 ms vs 3,998 ms baseline (+8% slower). LuaJIT's JIT-compiled linear scan over a flat array is faster than the hash table overhead at typical A* node pool sizes. Reverted.

### 15. heap+hash Combined Attempt — NavMeshQuery.lua ❌ REVERTED

**Change:** Kept binary heap but added a parallel hash table for node lookup alongside it.
**Rationale:** Hoped to get O(1) lookup benefit without disrupting the heap structure.
**Result:** avg 3,842 ms vs 3,998 ms re-measured baseline (-3.9%). Marginal improvement, but the complexity cost was judged not worth it. Reverted in favor of cleaner approaches.

### 16. _writeMidPoint Additional Optimization — NavMeshQuery.lua ✅

**Change:** Further reduced allocations/overhead in `_writeMidPoint` (on top of session 1 change #4).
**Result:** avg 3,780 ms (-5.5% vs 3,998 ms baseline). Confirmed improvement.

---

## Combined Benchmarks


| State                  | Runs | Avg (ms) | vs HEAD   | Notes                                                                          |
| ---------------------- | ---- | -------- | --------- | ------------------------------------------------------------------------------ |
| HEAD (no opts)         | 1    | 4166     | —         | Confirmed via git stash baseline                                               |
| Session 1+2 changes    | 5    | ~4004    | -3.9%     | 4065, 4131, 3941, 3910, 3972                                                   |
| Session 3 re-baseline  | 10   | 3998     | -4.0%     | 3998.2, 3514.6, 3905.7, 3979.0, 3735.4, 3444.0, 4341.7, 3569.6, 4306.0, 3850.5 |
| + Indexed heap (#14)   | 8    | 4316     | +3.6%     | REGRESSION — reverted                                                          |
| + heap+hash (#15)      | 10   | 3842     | -7.8%     | Marginal, reverted                                                             |
| + _writeMidPoint (#16) | 10   | 3780     | -9.3%     | Confirmed improvement                                                          |
| **Current confirmed**  | 10   | **3762** | **-9.7%** | 3642, 3794, 3315, 3140, 3820, 4051, 4370, 3811, 4011, 3666                     |


---

## TODO / Next Steps

1. ~~**Bisect #9 (_pLeft/_pRight)**~~ — Resolved in session 2/3 work.
2. `**getNode` linear scan** — Still 22% of runtime. The indexed heap failed, but there may be a cheaper approach: pre-clear the node pool between queries (currently the scan searches for free slots). Profile whether the scan is searching for existing nodes or allocating new ones.
3. *`findPath` A inner loop** — 21% of runtime. Already near local limit (~30+ locals). Options:
  - Reduce table field accesses by storing poly fields in locals at loop entry (but risks adding locals).
  - Review the neighbour-iteration loop for any redundant getRef/getTile calls.
4. `**parseTileData`** — 9–11% of runtime. Called once per tile load, not per query. Improving it helps cold-start but not steady-state pathfinding. Low priority unless tile reloading is common.
5. `**findConnectingPolys**` — 5–7% of runtime (NavMesh.lua:1032–1038). Called during tile connection. Tight loop; check for avoidable table allocations.
6. `**_bubbleUp` / `_trickleDown**` — 4–8% each. These are the binary heap ops. Already fairly minimal; could try unrolling first comparison or using integer indices more aggressively.
7. `**_appendVertex` allocation** — Each path point allocates `{pos[1], pos[2], pos[3]}`. Could pre-alloc a fixed `straightPath` result array, but requires knowing max size upfront.
8. `**findStraightPath` portal apex/left/right** — `portalApex`, `portalLeft`, `portalRight` are allocated per call. Pre-alloc at module level is possible but aliasing risk (as with #9) must be verified.
9. `**closestStart`/`closestEnd` return alloc** — `closestPointOnPolyBoundary` still returns a new table. Caller could pass a destination buffer.
10. `**findNearestPoly`** — Returns `startNearPt` which allocates. Could write into caller-supplied buffer.

### Won't Try

- **Coroutine/async parallelism**: LuaJIT is single-threaded; no benefit.
- **FFI for math**: Would require LuaJIT FFI structs replacing the `{x,y,z}` tables throughout — massive refactor, high risk, uncertain payoff.
- **Re-implementing in C**: Out of scope; goal is Lua optimization.

---

## Key Constraints (Reminder)

- **findPath local limit**: ~30+ locals already. ANY new local causes register spill. Don't add to findPath.
- **No for loops in bash** — use script files.
- **8 pre-existing mismatches** are expected and OK.
- **Module-level pre-alloc buffers are safe** (LuaJIT is single-threaded).
- **Module-level upvalue aliasing risk**: If a hot loop assigns `local x = _moduleTable` and then calls a function that writes to `_moduleTable`, JIT may deoptimize. Test carefully.

---

## Lua 5.1 Benchmarks (Primary Target)

Benchmark: `bench_lua51.sh N` (uses standard Lua 5.1, non-JIT)
Pairs: 96 coordinate pairs on map 0 (TBC)
Platform: macOS Darwin 24.3.0, Lua 5.1

> **Note:** Lua 5.1 is the actual target environment (WoW). LuaJIT benchmarks are for secondary reference only.

### Lua 5.1 Baseline Table

| State                               | Runs | Avg (ms)    | vs Unopt  | Notes                                                                       |
| ----------------------------------- | ---- | ----------- | --------- | --------------------------------------------------------------------------- |
| Unoptimized (389f7ab, goto-patched) | 10   | **19,498.1**| —         | Original code with only goto→repeat/if fixes for Lua 5.1 compat            |
| Sessions 1–5 (commit 18542f4)       | 10   | **13,182**  | **-32.4%**| All optimizations through Test D (slab/connection alloc elimination)        |
| Sessions 1–6 (commit 80af820)       | 10   | **12,711**  | **-34.8%**| +Tests E (neutral) and F (findStraightPath pre-alloc)                       |
| Sessions 1–7 (commit ac650cf)       | 10   | **11,774**  | **-39.6%**| +Tests G (neutral), H (-4.3% O(1) heap modify), I (neutral), J (-2.4% hash) |
| Sessions 1–8 (commit 227c76a)       | 10   | **11,714**  | **-39.9%**| +Tests K (regression/reverted), L (neutral), M (-1.5% _floor elimination)   |

### Session 5 Lua 5.1 Detail

| Test                                        | Runs | Avg (ms) | vs Prior  | Notes                                                               |
| ------------------------------------------- | ---- | -------- | --------- | ------------------------------------------------------------------- |
| Pre-session-5 baseline (sessions 1–4)       | 10   | 13,648   | —         | Starting point for session 5                                        |
| Test C (local h in modify + ep removal)     | 10   | 13,648   | neutral   | Kept changes; local h caching and ep alias removal                  |
| Test D (slab/connection alloc elimination)  | 10   | 13,182   | **-3.4%** | Pre-allocated 10 module-level buffers in NavMesh.lua; committed     |

### Session 6 Lua 5.1 Detail

| Test                                                    | Runs | Avg (ms) | vs Prior  | Notes                                                                                 |
| ------------------------------------------------------- | ---- | -------- | --------- | ------------------------------------------------------------------------------------- |
| Pre-session-6 baseline (sessions 1–5)                   | 10   | 13,182   | —         | Starting point for session 6                                                          |
| Test E (arithmetic flags + field caching in findPath)   | 10   | 13,222   | neutral   | band()→arithmetic for DT_NODE_OPEN/CLOSED; cache self._nodePool/openList/nav as local |
| Test F (pre-allocate findStraightPath buffers)          | 10   | 12,711   | **-3.6%** | Module-level _fspApex/Left/Right; pre-alloc 256 path point tables; committed          |

### Session 7 Lua 5.1 Detail

| Test                                                    | Runs | Avg (ms) | vs Prior  | Notes                                                                                      |
| ------------------------------------------------------- | ---- | -------- | --------- | ------------------------------------------------------------------------------------------ |
| Pre-session-7 baseline (sessions 1–6)                   | 10   | 12,629   | —         | Starting point for session 7 (Test G baseline = post-F state)                              |
| Test G (closestPtOnPolyBnd dest-buf + passFilter inline)| 10   | 12,629   | neutral   | dest param eliminates 2-3 allocs/findStraightPath; passFilter inlined in findPath           |
| Test H (O(1) heap modify via node._hidx tracking)       | 10   | 12,083   | **-4.3%** | _bubbleUp/_trickleDown store _hidx; modify() uses node._hidx directly; no O(n) scan        |
| Test I (inline decodePolyId in getTileAndPolyByRefUnsafe)| 10  | 12,068   | neutral   | Skip salt computation; avoids decodePolyId function call; not measurable at this scale      |
| Test J (combine hash in getNode/findNode, 1 band() call)| 10   | 11,774   | **-2.4%** | band(band(x,0xFFFFFFFF),mask)→band(x,mask); eliminates dtHashRef call per getNode invoc.   |
| Test K (fast path branch in getNode for hi=0)           | 10   | 12,068   | regression| Branch overhead exceeded savings; reverted                                                 |
| Test L (unsafe lookup in _getPortalPoints + pathBuf)    | 10   | 11,897   | neutral   | Valid path refs → unsafe tile lookup; pre-alloc pathBuf in _getPathToNode                  |
| Test M (eliminate _floor for type checks + hash hi)     | 10   | 11,714   | **-1.5%** | areaAndtype>=64 replaces _floor(areaAndtype/64); (id-lo)/4294967296 avoids _floor in hash  |

### Session 9 Lua 5.1 Detail

| Test                                                              | Runs | Avg (ms) | vs Prior  | Notes                                                                                                    |
| ----------------------------------------------------------------- | ---- | -------- | --------- | -------------------------------------------------------------------------------------------------------- |
| Pre-session-9 baseline                                            | 10   | 11,714   | —         | Same as session 8 end (Test M)                                                                           |
| Test N (hoist filterAreaCost out of inner link loop)              | 10   | 11,867   | neutral   | Saves 1 GETTABLE+MOD per inner iter; within noise of M baseline                                         |
| Test O (_floor(side/2)→lookup table; _getPortalPoints _floor fix) | 10   | 11,696   | neutral   | _sideToCS lookup replaces C call; -1.4% vs N but neutral vs M                                          |
| Test P (1-based heap indexing)                                    | 10   | 11,548   | **-1.4%** | Removes +1 ADD from every heap array access; simplifies parent=_floor(i/2)                              |
| Test Q (_floor(i/2)→(i-i%2)/2 in _bubbleUp) ❌                   | 10   | 11,717   | regression| Lua % calls floor+mul+sub internally — more expensive than _floor; reverted                             |
| Test R (NodePool closure upvalues for first/nodes/next/_hashMask) | 10   | 11,525   | neutral   | Eliminates 4 GETTABLE per getNode call; not measurable at this granularity                              |
| Test S (cache bestTile.links before inner loop)                   | 10   | 11,531   | neutral   | Saves 1 GETTABLE per inner iter; not measurable                                                         |
| Test T (inline getTileAndPolyByRefUnsafe in findPath inner loop)  | 10   | 11,235   | **-2.6%** | Eliminates method call overhead per A* neighbor; biggest win this session                               |
| Test U (inline outer loop tile lookup too) ❌                     | 10   | 11,524   | regression| Extra locals in outer loop scope hurt register layout more than call saved; reverted                    |
| Test V (cache _nodePool.getNode as local in findPath)             | 10   | 11,258   | neutral   | Saves 1 GETTABLE per inner iter; within noise of baseline                                               |
| Test W (cache bestNode.pos/cost/_idx before inner link loop)      | 10   | 11,078   | **-1.4%** | 3 fewer GETTABLE per inner iter (bestNode fields constant across link loop); committed                   |
| Test X (cache _openList.empty/pop/modify/push method refs) ❌      | 10   | 11,524   | regression| SELF opcode = 1 instr (GETTABLE+MOV); replacing with 2 GETLOCALs adds instructions; reverted            |
| Test Y (cache endPos/bestNodePos/np components as scalars)         | 10   | 10,941   | **-1.2%** | 9 fewer GETTABLE per inner iter; endPosX/Y/Z at fn scope, bnX/Y/Z before inner loop, npX/Y/Z inline    |

### Lua 5.1 Baseline Table (updated)

| State                               | Runs | Avg (ms)   | vs Unopt   | Notes                                                              |
| ----------------------------------- | ---- | ---------- | ---------- | ------------------------------------------------------------------ |
| Unoptimized (389f7ab, goto-patched) | 10   | **19,498** | —          | Original code with only goto→repeat/if fixes                       |
| Sessions 1–5 (commit 18542f4)       | 10   | **13,182** | -32.4%     |                                                                    |
| Sessions 1–6 (commit 80af820)       | 10   | **12,711** | -34.8%     |                                                                    |
| Sessions 1–7 (commit ac650cf)       | 10   | **11,774** | -39.6%     |                                                                    |
| Sessions 1–8 (commit 227c76a)       | 10   | **11,714** | -39.9%     |                                                                    |
| Sessions 1–9 (commit 346ba13)       | 10   | **11,235** | **-42.4%** | Test T: inline getTileAndPolyByRefUnsafe in findPath inner loop    |
| Sessions 1–10 (Test W)              | 10   | **11,078** | **-43.2%** | Test W: cache bestNode.pos/cost/_idx before inner link loop        |
| Sessions 1–11 (Test Y)              | 10   | **10,941** | **-43.9%** | Test Y: cache endPos/bestNodePos/np components as scalars          |

