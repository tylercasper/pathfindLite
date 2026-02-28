# Optimization Patterns and Hypotheses

This file records empirical patterns and hypotheses discovered during Lua 5.1
optimization of NavMeshQuery.lua. Use it to avoid re-trying failed approaches
and to identify when conditions have changed enough to re-test a hypothesis.

---

## Pattern 1: Inner-Loop Opcode Overhead in Lua 5.1

**Status:** Confirmed empirically (ABD-v1, ABD-v2, ABE — all neutral or regressed).

**Observation:** In the Lua 5.1 switch-dispatch interpreter, adding ANY net
bytecodes to the `findPath` inner link-loop body tends to regress or be neutral,
even when the added bytecodes theoretically eliminate more expensive operations
(C-calls, multiple GETTABLEs, etc.).

**Examples tested:**
| Test | Change | Result |
|------|--------|--------|
| ABD-v1 | Partial `getNode` inline (find only) | +4.4% regression |
| ABD-v2 | Full `getNode` inline (find + alloc) | +4.7% regression |
| ABE | Early break for CLOSED neighbours (`if nf>=2`) | Neutral (±0%) |

**Hypothesis:** Per-opcode dispatch cost in Lua 5.1's `lvm.c` switch loop is
so uniform and large relative to what each opcode "saves" that any net addition
of bytecodes hurts even when it avoids C-calls. The inner loop is already close
to the minimum achievable bytecode count.

**Corollary:** Only changes that **remove** bytecodes from the already-hot
path (or reduce the number of inner-loop *iterations* algorithmically) are
likely to help.

**When to re-test:** If the inner loop body is significantly restructured, or
if profiling shows a change in the dominant opcode mix (e.g., after bitop
library replacement).

---

## Pattern 2: CLOSED-Node Rarity in Benchmark Workload

**Status:** Hypothesis (inferred from ABE neutral result).

**Observation:** Test ABE added `if nf >= 2 then break end` before cost
computation (2 sqrt C-calls + ~15 ops), which is always correct for a
consistent heuristic. Result was neutral (9,875 ms vs 9,874 ms).

**Hypothesis:** CLOSED-node encounters are rare in the benchmark path set
(< ~5% of inner iterations). The savings per CLOSED encounter (2 sqrt C-calls
avoided) are outweighed by the overhead of the extra `nf >= 2` check on ALL
iterations.

**Implication:** Optimizations that only trigger on CLOSED neighbours will
yield negligible gains for this benchmark workload.

**When to re-test:** If the benchmark coordinate pairs change significantly
(e.g., longer paths with more A* backtracking), or after algorithmic changes
that increase CLOSED-node revisit frequency.

---

## Pattern 3: getNode Inline Regresses Despite GETUPVAL→GETLOCAL Savings

**Status:** Confirmed (ABD-v1 and ABD-v2 both regressed).

**Observation:** `_getNode` is called once per inner iteration (~22% of runtime
per LuaJIT profiler). Inlining its logic (hash lookup + chain walk + allocation)
seemed promising, but both partial and full inlines regressed.

**Root cause (ABD-v1):** Partial inline caused double-search: inline found
nothing for new nodes, then fell through to `_getNode` which searched again.

**Root cause (ABD-v2):** Full inline (avoiding double-search) still regressed
because the inline code body (~30 opcodes for find + ~20 for alloc) costs more
in the Lua 5.1 dispatcher than the 2 CALL+RETURNs it replaces, plus the
upvalue-to-local conversion savings were insufficient.

**When to re-test:** After bitop library replacement (which will change how the
hash bucket computation works and may reduce the inline body size). Also worth
re-testing if `getNode` call frequency decreases due to algorithmic changes.

---

## Dead Ends (Do Not Re-Try Without New Evidence)

- **`getNode` full inline in findPath** — regresses in Lua 5.1 (tested twice).
- **Partial `getNode` inline with fallback** — causes double-search; always wrong.
- **CLOSED-node early-exit guard** — neutral only; not worth more engineering
  effort unless CLOSED-encounter rate changes.

---

## Open Hypotheses (Not Yet Tested)

1. **`findStraightPath` has uninlined hot operations** — `_getPortalPoints` is
   called per path edge and may have function-call overhead worth inlining.

2. **`connectExtLinks` / `findConnectingPolys` bottleneck at tile load** — per
   LuaJIT profiler 5–7% each; these run at tile-load time, not per-query.
   Low priority for query latency but worth examining.

3. **Heap parent formula `(_bhi - _bhi%2) / 2`** — two arithmetic ops
   (`%` + `/`) per heap bubble step; could be replaced with a lookup table
   or bit shift (but bit shift requires bitop, so deferred).

4. **`dtVdist` / `_sqrt` call count** — currently 2–3 sqrt C-calls per inner
   iteration; some may be eliminatable by comparing squared distances where
   exact distance isn't needed (e.g., when only comparing relative costs).

5. **Pre-computing `filterAreaCost[areaAndtype % 64]` for neighbour poly** —
   `neighbourPoly.areaAndtype % 64` is computed once per edge in the non-endRef
   branch but `% 64` mod is a C-level Lua operation; might cache in local.
