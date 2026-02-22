-- NavMeshQuery.lua
-- Port of DetourNavMeshQuery.cpp to Lua 5.1

local nm   = require("NavMesh")
local band = require("bit").band

local DT_NULL_LINK    = nm.DT_NULL_LINK
local DT_EXT_LINK     = nm.DT_EXT_LINK
local DT_POLYTYPE_GROUND              = nm.DT_POLYTYPE_GROUND
local DT_POLYTYPE_OFFMESH_CONNECTION  = nm.DT_POLYTYPE_OFFMESH_CONNECTION
local DT_TILE_SHIFT   = nm.DT_TILE_SHIFT   -- 2^20 = 1048576; for inlining getTileAndPolyByRefUnsafe
local DT_TILE_MASK1   = nm.DT_TILE_MASK + 1 -- 2^28; used in modulo for tile index

local dtVdist     = nm.dtVdist
local dtVdistSqr  = nm.dtVdistSqr
local dtVlenSqr   = nm.dtVlenSqr
local dtVequal    = nm.dtVequal
local dtVcopy     = nm.dtVcopy
local dtVsub      = nm.dtVsub
local dtVadd      = nm.dtVadd
local dtVlerp     = nm.dtVlerp
local dtVmin      = nm.dtVmin
local dtVmax      = nm.dtVmax
local dtTriArea2D = nm.dtTriArea2D
local dtOverlapBounds = nm.dtOverlapBounds
local dtClamp     = nm.dtClamp
local dtMax       = nm.dtMax
local dtMin       = nm.dtMin
local dtAbs       = nm.dtAbs
local dtSqr       = nm.dtSqr
local nextPow2    = nm.nextPow2
local dtOppositeTile = nm.dtOppositeTile
local dtDistancePtSegSqr2D     = nm.dtDistancePtSegSqr2D
local dtDistancePtPolyEdgesSqr = nm.dtDistancePtPolyEdgesSqr
local dtIntersectSegSeg2D       = nm.dtIntersectSegSeg2D
local dtIntersectSegmentPoly2D  = nm.dtIntersectSegmentPoly2D

-- Cache stdlib lookups as locals (avoids global table lookup in hot paths)
local _floor = math.floor
local _sqrt  = math.sqrt

-- Module-level portal buffers (safe: Lua is single-threaded, no re-entrancy).
-- _getPortalPointsFull writes into these; callers read them before the next call.
local _pLeft  = {0,0,0}
local _pRight = {0,0,0}

-- findStraightPath portal apex/left/right buffers (reused every call).
local _fspApex       = {0,0,0}
local _fspLeft       = {0,0,0}
local _fspRight      = {0,0,0}
-- closestPointOnPolyBoundary destination buffers (no alloc per call).
local _fspClosestEnd = {0,0,0}

-- Lookup table: _floor(side/2) for link.side values 0-5 (external links).
-- Avoids _floor() C call in the hot findPath inner loop.
-- side: 0->0, 1->0, 2->1, 3->1, 4->2, 5->2  (1-based: index = side+1)
local _sideToCS = {0, 0, 1, 1, 2, 2}


-- Status flags
local DT_SUCCESS       = 0x40000000
local DT_FAILURE       = 0x80000000
local DT_IN_PROGRESS   = 0x20000000
local DT_PARTIAL_RESULT     = 0x40  -- bit 6
local DT_BUFFER_TOO_SMALL   = 0x10  -- bit 4
local DT_OUT_OF_NODES        = 0x20  -- bit 5
local DT_INVALID_PARAM       = 0x08  -- bit 3

-- Node flags
local DT_NODE_OPEN              = 0x01
local DT_NODE_CLOSED            = 0x02
local DT_NODE_PARENT_DETACHED   = 0x04

-- Straight path flags (matches DetourNavMesh.h)
local DT_STRAIGHTPATH_START               = 0x01
local DT_STRAIGHTPATH_END                 = 0x02
local DT_STRAIGHTPATH_OFFMESH_CONNECTION  = 0x04
local DT_STRAIGHTPATH_AREA_CROSSINGS      = 0x01  -- options
local DT_STRAIGHTPATH_ALL_CROSSINGS       = 0x02  -- options

local H_SCALE = 0.999

local M = {}

-- ---------------------------------------------------------------------------
-- Hash function for poly refs (64-bit aware)
-- ---------------------------------------------------------------------------
local function dtHashRef(a)
    local lo = a % 4294967296
    local hi = _floor(a / 4294967296)
    -- band(x, 0xFFFFFFFF) is faster than x % 4294967296 for the final reduction
    return band(lo * 2654435761 + hi * 1000003, 0xFFFFFFFF)
end

-- ---------------------------------------------------------------------------
-- NodePool: hash map of polyRef -> node(s)
-- ---------------------------------------------------------------------------
local function newNodePool(maxNodes, hashSize)
    local pool = {
        nodes     = {},     -- array of node tables, 1-indexed
        first     = {},     -- hash bucket -> first node index (1-based, 0=empty)
        next      = {},     -- node index -> next node in chain
        maxNodes  = maxNodes,
        hashSize  = hashSize,
        _hashMask = hashSize - 1,  -- power-of-2 mask for band() bucket reduction
        nodeCount = 0,
        _dirty    = {},     -- dirty bucket list for O(k) clear
        _dirtyN   = 0,
    }
    -- Initialize buckets to 0 (empty)
    for i = 1, hashSize do pool.first[i] = 0 end
    for i = 1, maxNodes do pool.next[i]  = 0 end

    -- Capture table refs and constants as closure upvalues:
    -- saves self.field GETTABLE overhead on every getNode/findNode call.
    local _first    = pool.first
    local _nodes    = pool.nodes
    local _next     = pool.next
    local _hashMask = pool._hashMask
    local _maxNodes = pool.maxNodes

    function pool:clear()
        local d, n = self._dirty, self._dirtyN
        for i = 1, n do _first[d[i]] = 0 end
        self._dirtyN = 0
        self.nodeCount = 0
    end

    function pool:getNodeIdx(node)
        if not node then return 0 end
        return node._idx
    end

    function pool:getNodeAtIdx(idx)
        if idx == 0 then return nil end
        return _nodes[idx]
    end

    function pool:getNode(id, state)
        -- WoW poly refs always fit in 32 bits: lo==id, hi-bits term is always 0.
        -- Simplified: band(id * 2654435761, _hashMask) saves 1 MOD + SUB + DIV + MUL + ADD.
        -- first/nodes/next/hashMask captured as closure upvalues: no per-call self.field lookup.
        local bucket = band(id * 2654435761, _hashMask) + 1
        local i = _first[bucket]
        while i ~= 0 do
            local n = _nodes[i]
            if n.id == id and n.state == state then
                return n
            end
            i = _next[i]
        end

        if self.nodeCount >= _maxNodes then
            return nil
        end

        -- Track dirty bucket for fast clear
        if _first[bucket] == 0 then
            local dn = self._dirtyN + 1
            self._dirtyN = dn
            self._dirty[dn] = bucket
        end

        self.nodeCount = self.nodeCount + 1
        local idx = self.nodeCount
        -- Reuse existing table to reduce allocation pressure
        local node = _nodes[idx]
        if node then
            -- pos/cost/total/pidx resets removed: findPath always writes these before reading them
            -- (flags=0 path: _writeMidPoint sets pos; inner loop sets cost/total/pidx before push/modify)
            node.state=state; node.flags=0; node.id=id
        else
            node = {
                _idx   = idx,
                pos    = {0,0,0},
                cost   = 0,
                total  = 0,
                pidx   = 0,
                state  = state,
                flags  = 0,
                id     = id,
            }
            _nodes[idx] = node
        end
        _next[idx]    = _first[bucket]
        _first[bucket] = idx
        return node
    end

    function pool:findNode(id, state)
        -- Same simplification as getNode: lo==id for all WoW poly refs
        local bucket = band(id * 2654435761, _hashMask) + 1
        local i = _first[bucket]
        while i ~= 0 do
            local n = _nodes[i]
            if n.id == id and n.state == state then
                return n
            end
            i = _next[i]
        end
        return nil
    end

    function pool:findNodes(id, maxN)
        local result = {}
        local bucket = band(dtHashRef(id), self._hashMask) + 1
        local i = self.first[bucket]
        while i ~= 0 do
            local n = self.nodes[i]
            if n.id == id then
                result[#result+1] = n
                if #result >= maxN then break end
            end
            i = self.next[i]
        end
        return result
    end

    return pool
end

-- ---------------------------------------------------------------------------
-- NodeQueue: binary min-heap on node.total
-- ---------------------------------------------------------------------------
local function newNodeQueue(capacity)
    local q = {
        heap     = {},
        capacity = capacity,
        size     = 0,
    }

    -- 1-based index math and 1-based array storage: h[1]=root, parent=floor(i/2), children=2i,2i+1.
    -- Each node carries _hidx (1-based heap position) so modify() is O(1) instead of O(n).
    function q:_bubbleUp(i, node)
        local h    = self.heap
        local ntot = node.total  -- cache loop-invariant: saves 1 GETTABLE per iteration
        while i > 1 do
            local parent = _floor(i / 2)
            local ph = h[parent]            -- no +1 needed
            if ph.total <= ntot then break end
            h[i] = ph
            ph._hidx = i   -- update displaced node's heap index
            i = parent
        end
        h[i] = node
        node._hidx = i
    end

    function q:_trickleDown(i, node)
        local h    = self.heap
        local sz   = self.size
        local ntot = node.total
        while true do
            local child = i*2              -- 1-based left child (was i*2+1)
            if child > sz then break end   -- was child >= sz
            local c1 = h[child]            -- no +1
            if child < sz and c1.total > h[child+1].total then   -- right child = child+1
                child = child + 1
                c1 = h[child]
            end
            if ntot <= c1.total then break end
            h[i] = c1
            c1._hidx = i   -- update displaced node's heap index
            i = child
        end
        h[i] = node
        node._hidx = i
    end

    function q:clear() self.size = 0 end

    function q:empty() return self.size == 0 end

    function q:top()
        return self.heap[1]
    end

    function q:push(node)
        self.size = self.size + 1
        self:_bubbleUp(self.size, node)    -- 1-based: last position = new size
    end

    function q:pop()
        local result = self.heap[1]
        local last   = self.heap[self.size]  -- save before decrement
        self.size = self.size - 1
        if self.size > 0 then
            self:_trickleDown(1, last)     -- 1-based root; no +1 needed for last
        end
        return result
    end

    -- O(1) lookup via node._hidx instead of O(n) scan
    function q:modify(node)
        self:_bubbleUp(node._hidx, node)
    end

    return q
end

-- ---------------------------------------------------------------------------
-- Default filter
-- ---------------------------------------------------------------------------
local function newFilter()
    local f = {
        areaCost     = {},
        includeFlags = 0xffff,
        excludeFlags = 0,
    }
    for i = 0, 63 do f.areaCost[i] = 1.0 end

    function f:passFilter(ref, tile, poly)
        local flags = poly.flags
        return band(flags, self.includeFlags) ~= 0 and band(flags, self.excludeFlags) == 0
    end

    function f:getCost(pa, pb, prevRef, prevTile, prevPoly,
                       curRef, curTile, curPoly,
                       nextRef, nextTile, nextPoly)
        local area = curPoly.areaAndtype % 64  -- getArea()
        return dtVdist(pa, pb) * (self.areaCost[area] or 1.0)
    end

    return f
end

-- ---------------------------------------------------------------------------
-- NavMeshQuery
-- ---------------------------------------------------------------------------
function M.new(navmesh, maxNodes)
    maxNodes = maxNodes or 65535
    local q = {
        _nav          = navmesh,
        _maxNodes     = maxNodes,
        _nodePool     = newNodePool(maxNodes, nextPow2(math.max(1, _floor(maxNodes/4)))),
        _tinyNodePool = newNodePool(64, 32),
        _openList     = newNodeQueue(maxNodes),
        -- Pre-allocated reusable buffers to reduce GC pressure
        _qBmin        = {0,0,0},   -- for queryPolygons bmin
        _qBmax        = {0,0,0},   -- for queryPolygons bmax
        _qpTmp        = {},        -- for queryPolygonsInTile output
        _qPolys       = {},        -- for queryPolygons polys output
        _qpiTmp       = {},        -- for _queryPolygonsInTile internal
        -- closestPointOnPolyBoundary scratch (max 6 verts = 18 floats each)
        _cbVerts      = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        _cbEdged      = {0,0,0,0,0,0},
        _cbEdget      = {0,0,0,0,0,0},
        -- findStraightPath reusable output arrays (avoids per-call alloc)
        _spPath       = {},
        _spFlags      = {},
        _spRefs       = {},
        _spCountRef   = {0},
        -- findPath path output buffer (avoids per-call alloc)
        _pathBuf      = {},
    }

    -- Pre-allocate inner position tables for up to 256 straight-path vertices
    do
        local sp = q._spPath
        for i = 1, 256 do sp[i] = {0,0,0} end
    end
    -- Pre-size path output buffer to avoid array growth during findPath
    do
        local pb = q._pathBuf
        for i = 1, 512 do pb[i] = 0 end
    end

    -- closestPointOnPolyBoundary(ref, pos[, dest]) -> closest (dest if provided, else new table)
    -- Uses unsafe tile lookup: callers pass valid path refs (from findPath/findNearestPoly).
    function q:closestPointOnPolyBoundary(ref, pos, dest)
        local tile, poly = self._nav:getTileAndPolyByRefUnsafe(ref)
        if not tile then return nil end

        local verts = self._cbVerts
        local nv = poly.vertCount
        for k = 1, nv do
            local vi = poly.verts[k]
            verts[k*3-2] = tile.verts[vi*3+1]
            verts[k*3-1] = tile.verts[vi*3+2]
            verts[k*3  ] = tile.verts[vi*3+3]
        end

        local edged = self._cbEdged; local edget = self._cbEdget
        local inside = dtDistancePtPolyEdgesSqr(pos, verts, nv, edged, edget)

        if inside then
            if dest then
                dest[1]=pos[1]; dest[2]=pos[2]; dest[3]=pos[3]
                return dest
            end
            return {pos[1], pos[2], pos[3]}
        else
            local dmin = edged[1]; local imin = 1
            for k = 2, nv do
                if edged[k] < dmin then dmin = edged[k]; imin = k end
            end
            local t = edget[imin]
            local vb_idx = (imin % nv) + 1
            -- Inline dtVlerp: eliminates va, vb, closest table allocations
            local v0x = verts[imin*3-2]; local v0y = verts[imin*3-1]; local v0z = verts[imin*3]
            local v1x = verts[vb_idx*3-2]; local v1y = verts[vb_idx*3-1]; local v1z = verts[vb_idx*3]
            if dest then
                dest[1]=v0x+(v1x-v0x)*t; dest[2]=v0y+(v1y-v0y)*t; dest[3]=v0z+(v1z-v0z)*t
                return dest
            end
            return {v0x+(v1x-v0x)*t, v0y+(v1y-v0y)*t, v0z+(v1z-v0z)*t}
        end
    end

    -- closestPointOnPoly(ref, pos) -> closest, posOverPoly
    function q:closestPointOnPoly(ref, pos)
        if not self._nav:isValidPolyRef(ref) then
            return nil, false
        end
        return self._nav:closestPointOnPoly(ref, pos)
    end

    -- queryPolygonsInTile (delegates to navmesh with filter)
    function q:_queryPolygonsInTile(tile, qmin, qmax, filter, outPolys)
        -- Reuse pre-allocated buffer to avoid per-call allocation
        local tmpPolys = self._qpiTmp
        local n = self._nav:queryPolygonsInTile(tile, qmin, qmax, tmpPolys, 512)
        local count = 0
        for k = 1, n do
            local ref = tmpPolys[k]
            local t2, poly = self._nav:getTileAndPolyByRefUnsafe(ref)
            if filter:passFilter(ref, t2, poly) then
                count = count + 1
                outPolys[count] = ref
            end
        end
        return count
    end

    -- queryPolygons(center, halfExtents, filter) -> polys array, count
    function q:queryPolygons(center, halfExtents, filter)
        -- Use pre-allocated bmin/bmax/polys buffers to avoid allocation
        local bmin = self._qBmin
        bmin[1] = center[1]-halfExtents[1]; bmin[2] = center[2]-halfExtents[2]; bmin[3] = center[3]-halfExtents[3]
        local bmax = self._qBmax
        bmax[1] = center[1]+halfExtents[1]; bmax[2] = center[2]+halfExtents[2]; bmax[3] = center[3]+halfExtents[3]

        local minx, miny = self._nav:calcTileLoc(bmin)
        local maxx, maxy = self._nav:calcTileLoc(bmax)

        local polys = self._qPolys; local total = 0
        local tmp   = self._qpTmp
        for ty = miny, maxy do
            for tx = minx, maxx do
                local neis, nc = self._nav:getTilesAt(tx, ty)
                for ti = 1, nc do
                    local cnt = self:_queryPolygonsInTile(neis[ti], bmin, bmax, filter, tmp)
                    for k = 1, cnt do
                        total = total + 1
                        polys[total] = tmp[k]
                    end
                end
            end
        end
        return polys, total
    end

    -- findNearestPoly(center, halfExtents, filter) -> ref, nearestPt
    function q:findNearestPoly(center, halfExtents, filter)
        local polys, polyCount = self:queryPolygons(center, halfExtents, filter)
        local nearest = 0
        local nearestDistSqr = math.huge
        local nearestPt = {center[1], center[2], center[3]}

        for k = 1, polyCount do
            local ref = polys[k]
            local closestPtPoly, posOverPoly = self:closestPointOnPoly(ref, center)
            if closestPtPoly then

            -- Inline diff computation to avoid {x,y,z} allocation
            local d0 = center[1]-closestPtPoly[1]
            local d1 = center[2]-closestPtPoly[2]
            local d2 = center[3]-closestPtPoly[3]
            local d
            if posOverPoly then
                -- Favor polys the point is over, with climb height consideration
                local tile, _ = self._nav:getTileAndPolyByRefUnsafe(ref)
                d = dtAbs(d1) - tile.header.walkableClimb
                d = d > 0 and d*d or 0
            else
                d = d0*d0 + d1*d1 + d2*d2
            end

            if d < nearestDistSqr then
                nearestPt = {closestPtPoly[1], closestPtPoly[2], closestPtPoly[3]}
                nearestDistSqr = d
                nearest = ref
            end
            end -- closestPtPoly
        end

        return nearest, nearestPt
    end

    -- getEdgeMidPoint (two forms)
    function q:_getEdgeMidPointFull(from, fromPoly, fromTile, to, toPoly, toTile)
        if not self:_getPortalPointsFull(from, fromPoly, fromTile, to, toPoly, toTile) then return nil end
        return {(_pLeft[1]+_pRight[1])*0.5, (_pLeft[2]+_pRight[2])*0.5, (_pLeft[3]+_pRight[3])*0.5}
    end

    -- _writeMidPoint: writes portal midpoint into dest using the already-found link.
    -- Caller passes the link directly (avoids redundant link search for GROUND polys).
    -- Signature: (fromPoly, fromTile, toPoly, toTile, link, from, dest)
    -- where 'link' is the link from fromPoly to toPoly, 'from' is fromPoly's ref (for offmesh fallback)
    function q:_writeMidPoint(fromPoly, fromTile, toPoly, toTile, link, from, dest)
        -- areaAndtype >= 64 means type >= 1 (OFFMESH); avoids 2 _floor() C calls
        if fromPoly.areaAndtype < 64 and toPoly.areaAndtype < 64 then
            -- Use link directly — no need to re-search the link list
            local tverts = fromTile.verts
            local bpv = fromPoly.verts
            local v0i = bpv[link.edge + 1]
            local v1i = bpv[((link.edge + 1) % fromPoly.vertCount) + 1]
            local v0x = tverts[v0i*3+1]; local v0y = tverts[v0i*3+2]; local v0z = tverts[v0i*3+3]
            local v1x = tverts[v1i*3+1]; local v1y = tverts[v1i*3+2]; local v1z = tverts[v1i*3+3]
            if link.side ~= 0xff and (link.bmin ~= 0 or link.bmax ~= 255) then
                local s = 1.0/255.0
                local tmin = link.bmin*s; local tmax = link.bmax*s
                dest[1]=(v0x+(v1x-v0x)*tmin + v0x+(v1x-v0x)*tmax)*0.5
                dest[2]=(v0y+(v1y-v0y)*tmin + v0y+(v1y-v0y)*tmax)*0.5
                dest[3]=(v0z+(v1z-v0z)*tmin + v0z+(v1z-v0z)*tmax)*0.5
            else
                dest[1]=(v0x+v1x)*0.5; dest[2]=(v0y+v1y)*0.5; dest[3]=(v0z+v1z)*0.5
            end
            return
        end
        -- Offmesh fallback: link.ref is the neighbour ref
        local mid = self:_getEdgeMidPointFull(from, fromPoly, fromTile, link.ref, toPoly, toTile)
        if mid then dtVcopy(dest, mid) end
    end

    -- getPortalPoints (full form with tiles/polys)
    -- Writes results into module-level _pLeft/_pRight; returns true/false (no table allocs).
    function q:_getPortalPointsFull(from, fromPoly, fromTile, to, toPoly, toTile)
        -- Find link from -> to
        local link = nil
        local li = fromPoly.firstLink
        while li ~= DT_NULL_LINK do
            local lk = fromTile.links[li+1]
            if lk.ref == to then link = lk; break end
            li = lk.next
        end
        if not link then return false end

        -- areaAndtype >= 64 means type >= 1 (OFFMESH); avoids 2 _floor() C calls
        if fromPoly.areaAndtype >= 64 then  -- fromType == DT_POLYTYPE_OFFMESH_CONNECTION
            li = fromPoly.firstLink
            while li ~= DT_NULL_LINK do
                local lk = fromTile.links[li+1]
                if lk.ref == to then
                    local vi = fromPoly.verts[lk.edge+1]
                    local vx = fromTile.verts[vi*3+1]; local vy = fromTile.verts[vi*3+2]; local vz = fromTile.verts[vi*3+3]
                    _pLeft[1]=vx; _pLeft[2]=vy; _pLeft[3]=vz
                    _pRight[1]=vx; _pRight[2]=vy; _pRight[3]=vz
                    return true
                end
                li = lk.next
            end
            return false
        end

        if toPoly.areaAndtype >= 64 then  -- toType == DT_POLYTYPE_OFFMESH_CONNECTION
            li = toPoly.firstLink
            while li ~= DT_NULL_LINK do
                local lk = toTile.links[li+1]
                if lk.ref == from then
                    local vi = toPoly.verts[lk.edge+1]
                    local vx = toTile.verts[vi*3+1]; local vy = toTile.verts[vi*3+2]; local vz = toTile.verts[vi*3+3]
                    _pLeft[1]=vx; _pLeft[2]=vy; _pLeft[3]=vz
                    _pRight[1]=vx; _pRight[2]=vy; _pRight[3]=vz
                    return true
                end
                li = lk.next
            end
            return false
        end

        -- Normal portal: edge e goes from vertex e to vertex (e+1)%nv
        local v0i = fromPoly.verts[link.edge + 1]
        local v1i = fromPoly.verts[((link.edge + 1) % fromPoly.vertCount) + 1]
        local tv = fromTile.verts
        local v0x = tv[v0i*3+1]; local v0y = tv[v0i*3+2]; local v0z = tv[v0i*3+3]
        local v1x = tv[v1i*3+1]; local v1y = tv[v1i*3+2]; local v1z = tv[v1i*3+3]

        if link.side ~= 0xff and (link.bmin ~= 0 or link.bmax ~= 255) then
            local s = 1.0/255.0
            local tmin = link.bmin * s; local tmax = link.bmax * s
            _pLeft[1]  = v0x+(v1x-v0x)*tmin; _pLeft[2]  = v0y+(v1y-v0y)*tmin; _pLeft[3]  = v0z+(v1z-v0z)*tmin
            _pRight[1] = v0x+(v1x-v0x)*tmax; _pRight[2] = v0y+(v1y-v0y)*tmax; _pRight[3] = v0z+(v1z-v0z)*tmax
        else
            _pLeft[1]=v0x; _pLeft[2]=v0y; _pLeft[3]=v0z
            _pRight[1]=v1x; _pRight[2]=v1y; _pRight[3]=v1z
        end
        return true
    end

    -- getPortalPoints (ref form) -> _pLeft, _pRight, fromType, toType  (or nil,nil,nil,nil)
    -- Uses unsafe tile lookup: path refs come from findPath which uses validated refs.
    function q:_getPortalPoints(from, to)
        local fromTile, fromPoly = self._nav:getTileAndPolyByRefUnsafe(from)
        if not fromTile then return nil, nil, nil, nil end
        local toTile, toPoly = self._nav:getTileAndPolyByRefUnsafe(to)
        if not toTile then return nil, nil, nil, nil end

        if not self:_getPortalPointsFull(from, fromPoly, fromTile, to, toPoly, toTile) then
            return nil, nil, nil, nil
        end
        -- areaAndtype >= 64 means type == DT_POLYTYPE_OFFMESH_CONNECTION (1); avoids 2 _floor() C calls
        local fromType = fromPoly.areaAndtype >= 64 and 1 or 0
        local toType   = toPoly.areaAndtype   >= 64 and 1 or 0
        return _pLeft, _pRight, fromType, toType
    end

    -- appendVertex
    function q:_appendVertex(pos, flags, ref, straightPath, straightPathFlags, straightPathRefs, countRef, maxStraight)
        local count = countRef[1]
        if count > 0 and dtVequal(straightPath[count], pos) then
            -- Update flags and ref of last vertex
            straightPathFlags[count] = flags
            straightPathRefs[count]  = ref
        else
            count = count + 1
            countRef[1] = count
            -- Write into pre-allocated table if available; avoids per-vertex alloc
            local pt = straightPath[count]
            if pt then
                pt[1] = pos[1]; pt[2] = pos[2]; pt[3] = pos[3]
            else
                straightPath[count] = {pos[1], pos[2], pos[3]}
            end
            straightPathFlags[count] = flags
            straightPathRefs[count]  = ref

            if count >= maxStraight then
                return DT_SUCCESS + DT_BUFFER_TOO_SMALL
            end
            if flags == DT_STRAIGHTPATH_END then
                return DT_SUCCESS
            end
        end
        return DT_IN_PROGRESS
    end

    -- appendPortals
    function q:_appendPortals(startIdx, endIdx, endPos, path,
                               straightPath, straightPathFlags, straightPathRefs,
                               countRef, maxStraight, options)
        local count = countRef[1]
        local startPos = straightPath[count]

        for i = startIdx, endIdx - 1 do
            local from = path[i]
            local to   = path[i+1]

            local fromTile, fromPoly = self._nav:getTileAndPolyByRef(from)
            if not fromTile then return DT_FAILURE + DT_INVALID_PARAM end
            local toTile, toPoly = self._nav:getTileAndPolyByRef(to)
            if not toTile then return DT_FAILURE + DT_INVALID_PARAM end

            if not self:_getPortalPointsFull(from, fromPoly, fromTile, to, toPoly, toTile) then break end

            -- If DT_STRAIGHTPATH_AREA_CROSSINGS (bit 0) is set, skip portals between same-area polys
            local doPortal = true
            if (options % 2) == 1 then  -- options & DT_STRAIGHTPATH_AREA_CROSSINGS
                local fromArea = fromPoly.areaAndtype % 64
                local toArea   = toPoly.areaAndtype   % 64
                doPortal = (fromArea ~= toArea)
            end

            if doPortal then
            local ok, s, t = dtIntersectSegSeg2D(startPos, endPos, _pLeft, _pRight)
            if ok then
                local pt = {0,0,0}
                dtVlerp(pt, _pLeft, _pRight, t)
                local stat = self:_appendVertex(pt, 0, to,
                    straightPath, straightPathFlags, straightPathRefs, countRef, maxStraight)
                if stat ~= DT_IN_PROGRESS then return stat end
            end
            end -- doPortal
        end
        return DT_IN_PROGRESS
    end

    -- getPathToNode
    function q:_getPathToNode(endNode, maxPath)
        -- Count length
        local _getNodeAtIdx = self._nodePool.getNodeAtIdx
        local _pool = self._nodePool
        local curNode = endNode
        local length = 0
        repeat
            length = length + 1
            curNode = _getNodeAtIdx(_pool, curNode.pidx)
        until not curNode

        -- Trim if too long
        curNode = endNode
        local writeCount = length
        for _ = writeCount, maxPath+1, -1 do
            writeCount = writeCount - 1
            curNode = _getNodeAtIdx(_pool, curNode.pidx)
        end

        local path = self._pathBuf
        for i = writeCount, 1, -1 do
            path[i] = curNode.id
            curNode = _getNodeAtIdx(_pool, curNode.pidx)
        end

        local pathCount = math.min(length, maxPath)
        local status = DT_SUCCESS
        if length > maxPath then status = status + DT_BUFFER_TOO_SMALL end
        return path, pathCount, status
    end

    -- findPath(startRef, endRef, startPos, endPos, filter, maxPath)
    -- Returns path (array of refs), pathCount, status
    function q:findPath(startRef, endRef, startPos, endPos, filter, maxPath)
        maxPath = maxPath or 4096

        if not self._nav:isValidPolyRef(startRef) or
           not self._nav:isValidPolyRef(endRef) then
            return {}, 0, DT_FAILURE + DT_INVALID_PARAM
        end

        if startRef == endRef then
            return {startRef}, 1, DT_SUCCESS
        end

        local _nodePool = self._nodePool
        local _openList = self._openList
        local _nav      = self._nav

        _nodePool:clear()
        _openList:clear()

        local startNode = _nodePool:getNode(startRef, 0)
        dtVcopy(startNode.pos, startPos)
        startNode.pidx  = 0
        startNode.cost  = 0
        startNode.total = dtVdist(startPos, endPos) * H_SCALE
        startNode.id    = startRef
        startNode.flags = DT_NODE_OPEN
        _openList:push(startNode)

        local lastBestNode     = startNode
        local lastBestNodeCost = startNode.total
        local outOfNodes = false
        local filterAreaCost   = filter.areaCost   -- cache for inlined getCost
        local filterInclude    = filter.includeFlags
        local filterExclude    = filter.excludeFlags
        local _navTiles        = _nav._tiles        -- for inlining getTileAndPolyByRefUnsafe
        local _getNode         = _nodePool.getNode  -- cache method ref: saves 1 GETTABLE per inner iter
        local endPosX = endPos[1]; local endPosY = endPos[2]; local endPosZ = endPos[3]

        while not _openList:empty() do
            local bestNode = _openList:pop()
            -- Nodes in heap always have flags=DT_NODE_OPEN(1) when popped; set CLOSED directly
            bestNode.flags = DT_NODE_CLOSED

            if bestNode.id == endRef then
                lastBestNode = bestNode
                break
            end

            local bestRef = bestNode.id
            local bestTile, bestPoly = _nav:getTileAndPolyByRefUnsafe(bestRef)

            local parentRef = 0
            if bestNode.pidx ~= 0 then
                parentRef = _nodePool:getNodeAtIdx(bestNode.pidx).id
            end
            -- parentTile/parentPoly removed: inlined getCost doesn't use them

            -- Cache bestPoly area cost and tile links: both constant for all links of this poly
            local bestPolyAreaCost = filterAreaCost[bestPoly.areaAndtype % 64]
            local bestTileLinks = bestTile.links   -- cache: saves 1 GETTABLE per inner iteration
            local bestNodePos  = bestNode.pos      -- cache: saves 1 GETTABLE per inner iteration
            local bestNodeCost = bestNode.cost     -- cache: saves 1-2 GETTABLE per inner iteration
            local bestNodeIdx  = bestNode._idx     -- cache: saves 1 GETTABLE per inner iteration
            local bnX = bestNodePos[1]; local bnY = bestNodePos[2]; local bnZ = bestNodePos[3]
            local li = bestPoly.firstLink
            while li ~= DT_NULL_LINK do
                local link = bestTileLinks[li+1]
                local neighbourRef = link.ref
                li = link.next

                repeat

                if neighbourRef == 0 or neighbourRef == parentRef then
                    break
                end

                -- Inline getTileAndPolyByRefUnsafe: eliminates method call + RETURN overhead
                local _nip = neighbourRef % DT_TILE_SHIFT
                local neighbourTile = _navTiles[(neighbourRef - _nip) / DT_TILE_SHIFT % DT_TILE_MASK1 + 1]
                local neighbourPoly = neighbourTile.polys[_nip + 1]
                -- Inline passFilter: avoids method call + 2 GETTABLE per neighbor
                -- Short-circuit exclude check: filterExclude is usually 0, skips band() C call
                local npf = neighbourPoly.flags
                if band(npf, filterInclude) == 0 or (filterExclude ~= 0 and band(npf, filterExclude) ~= 0) then
                    break
                end

                local crossSide = 0
                if link.side ~= 0xff then
                    crossSide = _sideToCS[link.side + 1]  -- replaces _floor(link.side/2): no C call
                end

                local neighbourNode = _getNode(_nodePool, neighbourRef, crossSide)
                if not neighbourNode then
                    outOfNodes = true
                    break
                end

                if neighbourNode.flags == 0 then
                    -- Pass link directly: avoids re-searching link list inside _writeMidPoint
                    self:_writeMidPoint(bestPoly, bestTile, neighbourPoly, neighbourTile,
                                        link, bestRef, neighbourNode.pos)
                end

                -- Inline getCost: dtVdist(pa,pb) * areaCost[curPoly.area]
                local np = neighbourNode.pos
                local npX = np[1]; local npY = np[2]; local npZ = np[3]
                local dx = npX-bnX; local dy = npY-bnY; local dz = npZ-bnZ
                local curCost = _sqrt(dx*dx+dy*dy+dz*dz) * bestPolyAreaCost
                local cost, heuristic
                if neighbourRef == endRef then
                    local dx2 = npX-endPosX; local dy2 = npY-endPosY; local dz2 = npZ-endPosZ
                    local endCost = _sqrt(dx2*dx2+dy2*dy2+dz2*dz2) * filterAreaCost[neighbourPoly.areaAndtype % 64]
                    cost = bestNodeCost + curCost + endCost
                    heuristic = 0
                else
                    cost = bestNodeCost + curCost
                    local dx2 = npX-endPosX; local dy2 = npY-endPosY; local dz2 = npZ-endPosZ
                    heuristic = _sqrt(dx2*dx2+dy2*dy2+dz2*dz2) * H_SCALE
                end

                local total = cost + heuristic

                local nf = neighbourNode.flags
                -- Use arithmetic instead of band() C calls: OPEN=bit0, CLOSED=bit1
                if nf % 2 ~= 0 and total >= neighbourNode.total then
                    break  -- in open, worse
                end
                if nf % 4 >= 2 and total >= neighbourNode.total then
                    break  -- in closed, worse
                end

                neighbourNode.pidx  = bestNodeIdx    -- getNodeIdx inlined: bestNode never nil here
                -- neighbourNode.id == neighbourRef already (getNode guarantees it); skip redundant write
                -- clear closed flag (subtract bit 1 if set)
                if nf % 4 >= 2 then
                    nf = nf - DT_NODE_CLOSED
                end
                neighbourNode.cost  = cost
                neighbourNode.total = total

                if nf % 2 ~= 0 then
                    neighbourNode.flags = nf
                    _openList:modify(neighbourNode)
                else
                    neighbourNode.flags = nf + DT_NODE_OPEN
                    _openList:push(neighbourNode)
                end

                if heuristic < lastBestNodeCost then
                    lastBestNodeCost = heuristic
                    lastBestNode = neighbourNode
                end

                until true
            end
        end

        local path, pathCount, status = self:_getPathToNode(lastBestNode, maxPath)

        if lastBestNode.id ~= endRef then
            status = status + DT_PARTIAL_RESULT
        end
        if outOfNodes then
            status = status + DT_OUT_OF_NODES
        end

        return path, pathCount, status
    end

    -- raycast: returns t, hitNormal, path, pathCount, status
    function q:raycast(startRef, startPos, endPos, filter, options, prevRef)
        options = options or 0
        prevRef = prevRef or 0

        if not self._nav:isValidPolyRef(startRef) then
            return 0, {0,0,0}, {}, 0, DT_FAILURE + DT_INVALID_PARAM
        end

        local hitT = 0
        local hitNormal = {0,0,0}
        local hitPath = {}
        local pathCount = 0
        local status = DT_SUCCESS

        local curRef = startRef
        local tile, poly = self._nav:getTileAndPolyByRefUnsafe(curRef)
        local prevTile, prevPoly = tile, poly
        local nextTile, nextPoly = tile, poly

        local pTile, pPoly = nil, nil
        if prevRef ~= 0 then
            pTile, pPoly = self._nav:getTileAndPolyByRefUnsafe(prevRef)
            prevTile, prevPoly = pTile, pPoly
        end

        local curPos = {startPos[1], startPos[2], startPos[3]}
        local dir = {endPos[1]-startPos[1], endPos[2]-startPos[2], endPos[3]-startPos[3]}

        while curRef ~= 0 do
            -- collect vertices
            local nv = poly.vertCount
            local verts = {}
            for k = 1, nv do
                local vi = poly.verts[k]
                verts[k*3-2] = tile.verts[vi*3+1]
                verts[k*3-1] = tile.verts[vi*3+2]
                verts[k*3  ] = tile.verts[vi*3+3]
            end

            local ok, tmin, tmax, segMin, segMax = dtIntersectSegmentPoly2D(startPos, endPos, verts, nv)
            if not ok then
                -- no hit, stop
                break
            end

            if tmax > hitT then hitT = tmax end

            -- store visited poly
            pathCount = pathCount + 1
            hitPath[pathCount] = curRef

            -- ray end inside polygon
            if segMax == -1 then
                hitT = math.huge
                break
            end

            -- Follow neighbours
            local nextRef = 0
            local li = poly.firstLink
            while li ~= DT_NULL_LINK do
                local link = tile.links[li+1]
                li = link.next

                repeat

                if link.edge ~= segMax then break end

                nextTile, nextPoly = self._nav:getTileAndPolyByRefUnsafe(link.ref)
                local ntype = _floor(nextPoly.areaAndtype / 64)
                if ntype == DT_POLYTYPE_OFFMESH_CONNECTION then break end
                if not filter:passFilter(link.ref, nextTile, nextPoly) then break end

                if link.side == 0xff then
                    nextRef = link.ref; break
                end
                if link.bmin == 0 and link.bmax == 255 then
                    nextRef = link.ref; break
                end

                local v0i = poly.verts[link.edge + 1]
                local v1i = poly.verts[(link.edge % nv) + 1]
                local lv = {tile.verts[v0i*3+1], tile.verts[v0i*3+2], tile.verts[v0i*3+3]}
                local rv = {tile.verts[v1i*3+1], tile.verts[v1i*3+2], tile.verts[v1i*3+3]}

                if link.side == 0 or link.side == 4 then
                    local s = 1.0/255.0
                    local lmin = lv[3] + (rv[3]-lv[3])*(link.bmin*s)
                    local lmax = lv[3] + (rv[3]-lv[3])*(link.bmax*s)
                    if lmin > lmax then local tmp=lmin; lmin=lmax; lmax=tmp end
                    local z = startPos[3] + (endPos[3]-startPos[3])*tmax
                    if z >= lmin and z <= lmax then nextRef = link.ref; break end
                elseif link.side == 2 or link.side == 6 then
                    local s = 1.0/255.0
                    local lmin = lv[1] + (rv[1]-lv[1])*(link.bmin*s)
                    local lmax = lv[1] + (rv[1]-lv[1])*(link.bmax*s)
                    if lmin > lmax then local tmp=lmin; lmin=lmax; lmax=tmp end
                    local x = startPos[1] + (endPos[1]-startPos[1])*tmax
                    if x >= lmin and x <= lmax then nextRef = link.ref; break end
                end
                until true
            end

            if nextRef == 0 then
                -- hit wall
                local a = segMax
                local b = (segMax + 1 < nv) and (segMax+1) or 0
                local va = {verts[a*3+1], verts[a*3+2], verts[a*3+3]}
                local vb = {verts[b*3+1], verts[b*3+2], verts[b*3+3]}
                local dx = vb[1]-va[1]; local dz = vb[3]-va[3]
                local len = _sqrt(dx*dx+dz*dz)
                if len > 0 then
                    hitNormal[1] = dz/len; hitNormal[2] = 0; hitNormal[3] = -dx/len
                end
                break
            end

            prevRef  = curRef
            curRef   = nextRef
            prevTile = tile; tile = nextTile
            prevPoly = poly; poly = nextPoly
        end

        return hitT, hitNormal, hitPath, pathCount, status
    end

    -- findStraightPath(startPos, endPos, path, pathSize, maxStraight, options)
    -- Returns straightPath (array of {pos, flags, ref}), count, status
    function q:findStraightPath(startPos, endPos, path, pathSize, maxStraight, options)
        options = options or 0
        maxStraight = maxStraight or 2048

        -- Use pre-allocated buffers to avoid per-call table creation
        local straightPath  = self._spPath
        local spFlags       = self._spFlags
        local spRefs        = self._spRefs
        local countRef      = self._spCountRef
        countRef[1] = 0  -- reset count

        if not path or pathSize <= 0 or not path[1] then
            return straightPath, 0, DT_FAILURE + DT_INVALID_PARAM
        end

        -- Write directly into pre-alloc buffers: _fspApex (also portalApex) and _fspClosestEnd
        local closestStart = self:closestPointOnPolyBoundary(path[1], startPos, _fspApex)
        if not closestStart then
            return straightPath, 0, DT_FAILURE + DT_INVALID_PARAM
        end

        local closestEnd = self:closestPointOnPolyBoundary(path[pathSize], endPos, _fspClosestEnd)
        if not closestEnd then
            return straightPath, 0, DT_FAILURE + DT_INVALID_PARAM
        end

        -- Add start point
        local stat = self:_appendVertex(closestStart, DT_STRAIGHTPATH_START, path[1],
            straightPath, spFlags, spRefs, countRef, maxStraight)
        if stat ~= DT_IN_PROGRESS then
            return straightPath, countRef[1], stat
        end

        if pathSize > 1 then
            -- Use module-level pre-allocated buffers for portal apex/left/right.
            -- closestStart IS _fspApex (written by closestPointOnPolyBoundary), so portalApex copy is a no-op.
            local portalApex  = _fspApex
            local portalLeft  = _fspLeft
            local portalRight = _fspRight
            portalLeft[1]=closestStart[1]; portalLeft[2]=closestStart[2]; portalLeft[3]=closestStart[3]
            portalRight[1]=closestStart[1]; portalRight[2]=closestStart[2]; portalRight[3]=closestStart[3]
            local apexIndex   = 1
            local leftIndex   = 1
            local rightIndex  = 1
            local leftPolyType  = 0
            local rightPolyType = 0
            local leftPolyRef   = path[1]
            local rightPolyRef  = path[1]

            local i = 1
            while i <= pathSize do
                repeat  -- repeat...until true: use break as "continue_i"

                local left, right
                local toType

                if i + 1 <= pathSize then
                    local l2, r2, fromType2, toType2 = self:_getPortalPoints(path[i], path[i+1])
                    if not l2 then
                        -- Failed portal: clamp end to path[i] (reuse _fspClosestEnd buffer)
                        local ce2 = self:closestPointOnPolyBoundary(path[i], endPos, _fspClosestEnd)
                        if not ce2 then return straightPath, 0, DT_FAILURE + DT_INVALID_PARAM end
                        closestEnd = ce2

                        if (options % 4) >= 1 then  -- area or all crossings
                            self:_appendPortals(apexIndex, i, closestEnd, path,
                                straightPath, spFlags, spRefs, countRef, maxStraight, options)
                        end
                        self:_appendVertex(closestEnd, 0, path[i],
                            straightPath, spFlags, spRefs, countRef, maxStraight)
                        stat = DT_SUCCESS + DT_PARTIAL_RESULT
                        if countRef[1] >= maxStraight then stat = stat + DT_BUFFER_TOO_SMALL end
                        return straightPath, countRef[1], stat
                    end
                    left = l2; right = r2; toType = toType2

                    -- If starting very close to the first portal, skip it (C++: if i==0)
                    if i == 1 then
                        local dsq = dtDistancePtSegSqr2D(portalApex, left, right)
                        if dsq < dtSqr(0.001) then
                            i = i + 1
                            break  -- goto continue_i (then i=i+1 at bottom → i+=2 total)
                        end
                    end
                else
                    -- End of path: write into shared portal buffers (no alloc)
                    _pLeft[1]=closestEnd[1]; _pLeft[2]=closestEnd[2]; _pLeft[3]=closestEnd[3]
                    _pRight[1]=closestEnd[1]; _pRight[2]=closestEnd[2]; _pRight[3]=closestEnd[3]
                    left = _pLeft; right = _pRight
                    toType = DT_POLYTYPE_GROUND
                end

                -- Right vertex
                if dtTriArea2D(portalApex, portalRight, right) <= 0 then
                    if dtVequal(portalApex, portalRight) or
                       dtTriArea2D(portalApex, portalLeft, right) > 0 then
                        dtVcopy(portalRight, right)
                        rightPolyRef  = (i+1 <= pathSize) and path[i+1] or 0
                        rightPolyType = toType
                        rightIndex    = i
                    else
                        if (options % 4) >= 1 then
                            stat = self:_appendPortals(apexIndex, leftIndex, portalLeft, path,
                                straightPath, spFlags, spRefs, countRef, maxStraight, options)
                            if stat ~= DT_IN_PROGRESS then return straightPath, countRef[1], stat end
                        end

                        dtVcopy(portalApex, portalLeft)
                        apexIndex = leftIndex

                        local flags = 0
                        if leftPolyRef == 0 then
                            flags = DT_STRAIGHTPATH_END
                        elseif leftPolyType == DT_POLYTYPE_OFFMESH_CONNECTION then
                            flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION
                        end
                        local ref = leftPolyRef

                        stat = self:_appendVertex(portalApex, flags, ref,
                            straightPath, spFlags, spRefs, countRef, maxStraight)
                        if stat ~= DT_IN_PROGRESS then return straightPath, countRef[1], stat end

                        dtVcopy(portalLeft,  portalApex)
                        dtVcopy(portalRight, portalApex)
                        leftIndex  = apexIndex
                        rightIndex = apexIndex

                        i = apexIndex
                        break  -- goto continue_i
                    end
                end

                -- Left vertex
                if dtTriArea2D(portalApex, portalLeft, left) >= 0 then
                    if dtVequal(portalApex, portalLeft) or
                       dtTriArea2D(portalApex, portalRight, left) < 0 then
                        dtVcopy(portalLeft, left)
                        leftPolyRef  = (i+1 <= pathSize) and path[i+1] or 0
                        leftPolyType = toType
                        leftIndex    = i
                    else
                        if (options % 4) >= 1 then
                            stat = self:_appendPortals(apexIndex, rightIndex, portalRight, path,
                                straightPath, spFlags, spRefs, countRef, maxStraight, options)
                            if stat ~= DT_IN_PROGRESS then return straightPath, countRef[1], stat end
                        end

                        dtVcopy(portalApex, portalRight)
                        apexIndex = rightIndex

                        local flags = 0
                        if rightPolyRef == 0 then
                            flags = DT_STRAIGHTPATH_END
                        elseif rightPolyType == DT_POLYTYPE_OFFMESH_CONNECTION then
                            flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION
                        end
                        local ref = rightPolyRef

                        stat = self:_appendVertex(portalApex, flags, ref,
                            straightPath, spFlags, spRefs, countRef, maxStraight)
                        if stat ~= DT_IN_PROGRESS then return straightPath, countRef[1], stat end

                        dtVcopy(portalLeft,  portalApex)
                        dtVcopy(portalRight, portalApex)
                        leftIndex  = apexIndex
                        rightIndex = apexIndex

                        i = apexIndex
                        break  -- goto continue_i
                    end
                end

                until true  -- end repeat (break acts as continue_i)
                i = i + 1
            end

            -- Append portals to end
            if (options % 4) >= 1 then
                stat = self:_appendPortals(apexIndex, pathSize, closestEnd, path,
                    straightPath, spFlags, spRefs, countRef, maxStraight, options)
                if stat ~= DT_IN_PROGRESS then return straightPath, countRef[1], stat end
            end
        end

        -- Append end
        self:_appendVertex(closestEnd, DT_STRAIGHTPATH_END, 0,
            straightPath, spFlags, spRefs, countRef, maxStraight)

        stat = DT_SUCCESS
        if countRef[1] >= maxStraight then stat = stat + DT_BUFFER_TOO_SMALL end


        -- Return straightPath directly (positions as {x,y,z} tables, no wrapper alloc)
        return straightPath, countRef[1], stat
    end

    return q
end

M.newFilter = newFilter

return M
