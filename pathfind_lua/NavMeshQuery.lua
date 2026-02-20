-- NavMeshQuery.lua
-- Port of DetourNavMeshQuery.cpp to Lua 5.1

local nm = require("NavMesh")

local DT_NULL_LINK    = nm.DT_NULL_LINK
local DT_EXT_LINK     = nm.DT_EXT_LINK
local DT_POLYTYPE_GROUND              = nm.DT_POLYTYPE_GROUND
local DT_POLYTYPE_OFFMESH_CONNECTION  = nm.DT_POLYTYPE_OFFMESH_CONNECTION

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
-- Based on Thomas Wang hash for large integers
-- ---------------------------------------------------------------------------
local function dtHashRef(a)
    -- For Lua doubles representing 64-bit values, keep it simple:
    -- Use a combination of lower and upper 32 bits
    local lo = a % 4294967296
    local hi = math.floor(a / 4294967296) % 4294967296
    -- Simple integer hash
    local v = lo + hi * 1000003
    v = v % 4294967296
    v = v + (65536 - v % 65536) * 65536  -- mix high bits
    v = v % 4294967296
    -- Final mix
    return v % 4294967296
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
        nodeCount = 0,
    }
    -- Initialize buckets to 0 (empty)
    for i = 1, hashSize do pool.first[i] = 0 end
    for i = 1, maxNodes do pool.next[i]  = 0 end

    function pool:clear()
        for i = 1, self.hashSize do self.first[i] = 0 end
        self.nodeCount = 0
    end

    function pool:getNodeIdx(node)
        if not node then return 0 end
        return node._idx
    end

    function pool:getNodeAtIdx(idx)
        if idx == 0 then return nil end
        return self.nodes[idx]
    end

    function pool:getNode(id, state)
        state = state or 0
        local bucket = dtHashRef(id) % self.hashSize + 1
        local i = self.first[bucket]
        while i ~= 0 do
            local n = self.nodes[i]
            if n.id == id and n.state == state then
                return n
            end
            i = self.next[i]
        end

        if self.nodeCount >= self.maxNodes then
            return nil
        end

        self.nodeCount = self.nodeCount + 1
        local idx = self.nodeCount
        local node = {
            _idx   = idx,
            pos    = {0,0,0},
            cost   = 0,
            total  = 0,
            pidx   = 0,
            state  = state,
            flags  = 0,
            id     = id,
        }
        self.nodes[idx] = node
        self.next[idx]  = self.first[bucket]
        self.first[bucket] = idx
        return node
    end

    function pool:findNode(id, state)
        local bucket = dtHashRef(id) % self.hashSize + 1
        local i = self.first[bucket]
        while i ~= 0 do
            local n = self.nodes[i]
            if n.id == id and n.state == state then
                return n
            end
            i = self.next[i]
        end
        return nil
    end

    function pool:findNodes(id, maxN)
        local result = {}
        local bucket = dtHashRef(id) % self.hashSize + 1
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

    local function bubbleUp(q, i, node)
        while i > 1 do
            local parent = math.floor((i-1)/2) + 1  -- 1-based parent: floor((i-2)/2)+1
            -- Actually: for 1-based heap, parent of i is floor(i/2)
            -- Let's use 0-based internally:
            -- We'll handle this below
            break
        end
        -- Redo with 0-based indexing internally (heap[] is 1-based storage but 0-based logic)
    end

    -- Use 0-based index math but 1-based array storage
    function q:_bubbleUp(i, node)
        while i > 0 do
            local parent = math.floor((i-1) / 2)
            if self.heap[parent+1].total <= node.total then break end
            self.heap[i+1] = self.heap[parent+1]
            i = parent
        end
        self.heap[i+1] = node
    end

    function q:_trickleDown(i, node)
        while true do
            local child = i*2 + 1
            if child >= self.size then break end
            if child+1 < self.size and
               self.heap[child+1].total > self.heap[child+2].total then
                child = child + 1
            end
            if node.total <= self.heap[child+1].total then break end
            self.heap[i+1] = self.heap[child+1]
            i = child
        end
        self.heap[i+1] = node
    end

    function q:clear() self.size = 0 end

    function q:empty() return self.size == 0 end

    function q:top()
        return self.heap[1]
    end

    function q:push(node)
        self.size = self.size + 1
        self:_bubbleUp(self.size - 1, node)
    end

    function q:pop()
        local result = self.heap[1]
        self.size = self.size - 1
        if self.size > 0 then
            self:_trickleDown(0, self.heap[self.size+1])
        end
        return result
    end

    function q:modify(node)
        for i = 1, self.size do
            if self.heap[i] == node then
                self:_bubbleUp(i-1, node)
                return
            end
        end
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
        -- (flags & includeFlags) != 0 && (flags & excludeFlags) == 0
        -- Bitwise AND simulation using modular arithmetic
        local function band(a, b)
            local result = 0
            local bit = 1
            while bit <= a or bit <= b do
                if (a % (bit*2)) >= bit and (b % (bit*2)) >= bit then
                    result = result + bit
                end
                bit = bit * 2
            end
            return result
        end
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
        _nodePool     = newNodePool(maxNodes, nextPow2(math.max(1, math.floor(maxNodes/4)))),
        _tinyNodePool = newNodePool(64, 32),
        _openList     = newNodeQueue(maxNodes),
    }

    -- closestPointOnPolyBoundary(ref, pos) -> closest
    function q:closestPointOnPolyBoundary(ref, pos)
        local tile, poly = self._nav:getTileAndPolyByRef(ref)
        if not tile then return nil end

        local verts = {}
        local nv = poly.vertCount
        for k = 1, nv do
            local vi = poly.verts[k]
            verts[k*3-2] = tile.verts[vi*3+1]
            verts[k*3-1] = tile.verts[vi*3+2]
            verts[k*3  ] = tile.verts[vi*3+3]
        end

        local edged = {}; local edget = {}
        local inside = dtDistancePtPolyEdgesSqr(pos, verts, nv, edged, edget)

        if inside then
            return {pos[1], pos[2], pos[3]}
        else
            local dmin = edged[1]; local imin = 1
            for k = 2, nv do
                if edged[k] < dmin then dmin = edged[k]; imin = k end
            end
            local va = {verts[imin*3-2], verts[imin*3-1], verts[imin*3]}
            local vb_idx = (imin % nv) + 1
            local vb = {verts[vb_idx*3-2], verts[vb_idx*3-1], verts[vb_idx*3]}
            local closest = {0,0,0}
            dtVlerp(closest, va, vb, edget[imin])
            return closest
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
        -- We collect refs from navmesh and filter here
        local tmpPolys = {}
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
        local bmin = {center[1]-halfExtents[1], center[2]-halfExtents[2], center[3]-halfExtents[3]}
        local bmax = {center[1]+halfExtents[1], center[2]+halfExtents[2], center[3]+halfExtents[3]}

        local minx, miny = self._nav:calcTileLoc(bmin)
        local maxx, maxy = self._nav:calcTileLoc(bmax)

        local polys = {}; local total = 0
        for ty = miny, maxy do
            for tx = minx, maxx do
                local neis = self._nav:getTilesAt(tx, ty)
                for _, tile in ipairs(neis) do
                    local tmp = {}
                    local cnt = self:_queryPolygonsInTile(tile, bmin, bmax, filter, tmp)
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
            if not closestPtPoly then goto continue_k end

            local diff = {center[1]-closestPtPoly[1], center[2]-closestPtPoly[2], center[3]-closestPtPoly[3]}
            local d
            if posOverPoly then
                -- Favor polys the point is over, with climb height consideration
                local tile, _ = self._nav:getTileAndPolyByRefUnsafe(ref)
                d = dtAbs(diff[2]) - tile.header.walkableClimb
                d = d > 0 and d*d or 0
            else
                d = dtVlenSqr(diff)
            end

            if d < nearestDistSqr then
                nearestPt = {closestPtPoly[1], closestPtPoly[2], closestPtPoly[3]}
                nearestDistSqr = d
                nearest = ref
            end
            ::continue_k::
        end

        return nearest, nearestPt
    end

    -- getEdgeMidPoint (two forms)
    function q:_getEdgeMidPointFull(from, fromPoly, fromTile, to, toPoly, toTile)
        local left, right = self:_getPortalPointsFull(from, fromPoly, fromTile, to, toPoly, toTile)
        if not left then return nil end
        return {(left[1]+right[1])*0.5, (left[2]+right[2])*0.5, (left[3]+right[3])*0.5}
    end

    -- getPortalPoints (full form with tiles/polys)
    function q:_getPortalPointsFull(from, fromPoly, fromTile, to, toPoly, toTile)
        -- Find link from -> to
        local link = nil
        local li = fromPoly.firstLink
        while li ~= DT_NULL_LINK do
            local lk = fromTile.links[li+1]
            if lk.ref == to then link = lk; break end
            li = lk.next
        end
        if not link then return nil, nil end

        local fromType = math.floor(fromPoly.areaAndtype / 64)
        local toType   = math.floor(toPoly.areaAndtype   / 64)

        if fromType == DT_POLYTYPE_OFFMESH_CONNECTION then
            li = fromPoly.firstLink
            while li ~= DT_NULL_LINK do
                local lk = fromTile.links[li+1]
                if lk.ref == to then
                    local v = lk.edge
                    local vi = fromPoly.verts[v+1]
                    local left  = {fromTile.verts[vi*3+1], fromTile.verts[vi*3+2], fromTile.verts[vi*3+3]}
                    local right = {left[1], left[2], left[3]}
                    return left, right
                end
                li = lk.next
            end
            return nil, nil
        end

        if toType == DT_POLYTYPE_OFFMESH_CONNECTION then
            li = toPoly.firstLink
            while li ~= DT_NULL_LINK do
                local lk = toTile.links[li+1]
                if lk.ref == from then
                    local v = lk.edge
                    local vi = toPoly.verts[v+1]
                    local left  = {toTile.verts[vi*3+1], toTile.verts[vi*3+2], toTile.verts[vi*3+3]}
                    local right = {left[1], left[2], left[3]}
                    return left, right
                end
                li = lk.next
            end
            return nil, nil
        end

        -- Normal portal: edge e goes from vertex e to vertex (e+1)%nv
        local v0i = fromPoly.verts[link.edge + 1]
        local v1i = fromPoly.verts[((link.edge + 1) % fromPoly.vertCount) + 1]
        local left  = {fromTile.verts[v0i*3+1], fromTile.verts[v0i*3+2], fromTile.verts[v0i*3+3]}
        local right = {fromTile.verts[v1i*3+1], fromTile.verts[v1i*3+2], fromTile.verts[v1i*3+3]}

        if link.side ~= 0xff and (link.bmin ~= 0 or link.bmax ~= 255) then
            local s = 1.0/255.0
            local tmin = link.bmin * s
            local tmax = link.bmax * s
            local lv = {fromTile.verts[v0i*3+1], fromTile.verts[v0i*3+2], fromTile.verts[v0i*3+3]}
            local rv = {fromTile.verts[v1i*3+1], fromTile.verts[v1i*3+2], fromTile.verts[v1i*3+3]}
            dtVlerp(left,  lv, rv, tmin)
            dtVlerp(right, lv, rv, tmax)
        end

        return left, right
    end

    -- getPortalPoints (ref form)
    function q:_getPortalPoints(from, to)
        local fromTile, fromPoly = self._nav:getTileAndPolyByRef(from)
        if not fromTile then return nil, nil, nil, nil end
        local toTile, toPoly = self._nav:getTileAndPolyByRef(to)
        if not toTile then return nil, nil, nil, nil end

        local left, right = self:_getPortalPointsFull(from, fromPoly, fromTile, to, toPoly, toTile)
        local fromType = math.floor(fromPoly.areaAndtype / 64)
        local toType   = math.floor(toPoly.areaAndtype   / 64)
        return left, right, fromType, toType
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
            straightPath[count]      = {pos[1], pos[2], pos[3]}
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

            local left, right = self:_getPortalPointsFull(from, fromPoly, fromTile, to, toPoly, toTile)
            if not left then break end

            -- If DT_STRAIGHTPATH_AREA_CROSSINGS (bit 0) is set, skip portals between same-area polys
            if (options % 2) == 1 then  -- options & DT_STRAIGHTPATH_AREA_CROSSINGS
                local fromArea = fromPoly.areaAndtype % 64
                local toArea   = toPoly.areaAndtype   % 64
                if fromArea == toArea then goto continue_i end
            end

            local ok, s, t = dtIntersectSegSeg2D(startPos, endPos, left, right)
            if ok then
                local pt = {0,0,0}
                dtVlerp(pt, left, right, t)
                local stat = self:_appendVertex(pt, 0, to,
                    straightPath, straightPathFlags, straightPathRefs, countRef, maxStraight)
                if stat ~= DT_IN_PROGRESS then return stat end
            end
            ::continue_i::
        end
        return DT_IN_PROGRESS
    end

    -- getPathToNode
    function q:_getPathToNode(endNode, maxPath)
        -- Count length
        local curNode = endNode
        local length = 0
        repeat
            length = length + 1
            curNode = self._nodePool:getNodeAtIdx(curNode.pidx)
        until not curNode

        -- Trim if too long
        curNode = endNode
        local writeCount = length
        for _ = writeCount, maxPath+1, -1 do
            writeCount = writeCount - 1
            curNode = self._nodePool:getNodeAtIdx(curNode.pidx)
        end

        local path = {}
        for i = writeCount, 1, -1 do
            path[i] = curNode.id
            curNode = self._nodePool:getNodeAtIdx(curNode.pidx)
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

        self._nodePool:clear()
        self._openList:clear()

        local startNode = self._nodePool:getNode(startRef, 0)
        dtVcopy(startNode.pos, startPos)
        startNode.pidx  = 0
        startNode.cost  = 0
        startNode.total = dtVdist(startPos, endPos) * H_SCALE
        startNode.id    = startRef
        startNode.flags = DT_NODE_OPEN
        self._openList:push(startNode)

        local lastBestNode     = startNode
        local lastBestNodeCost = startNode.total
        local outOfNodes = false

        while not self._openList:empty() do
            local bestNode = self._openList:pop()
            bestNode.flags = bestNode.flags - (bestNode.flags % 2)  -- clear OPEN bit
            bestNode.flags = bestNode.flags + DT_NODE_CLOSED

            if bestNode.id == endRef then
                lastBestNode = bestNode
                break
            end

            local bestRef = bestNode.id
            local bestTile, bestPoly = self._nav:getTileAndPolyByRefUnsafe(bestRef)

            local parentRef = 0
            local parentTile, parentPoly = nil, nil
            if bestNode.pidx ~= 0 then
                parentRef = self._nodePool:getNodeAtIdx(bestNode.pidx).id
            end
            if parentRef ~= 0 then
                parentTile, parentPoly = self._nav:getTileAndPolyByRefUnsafe(parentRef)
            end

            local li = bestPoly.firstLink
            while li ~= DT_NULL_LINK do
                local link = bestTile.links[li+1]
                local neighbourRef = link.ref
                li = link.next

                if neighbourRef == 0 or neighbourRef == parentRef then
                    goto continue_link
                end

                local neighbourTile, neighbourPoly = self._nav:getTileAndPolyByRefUnsafe(neighbourRef)
                if not filter:passFilter(neighbourRef, neighbourTile, neighbourPoly) then
                    goto continue_link
                end

                local crossSide = 0
                if link.side ~= 0xff then
                    crossSide = math.floor(link.side / 2)
                end

                local neighbourNode = self._nodePool:getNode(neighbourRef, crossSide)
                if not neighbourNode then
                    outOfNodes = true
                    goto continue_link
                end

                if neighbourNode.flags == 0 then
                    local mid = self:_getEdgeMidPointFull(bestRef, bestPoly, bestTile,
                                                           neighbourRef, neighbourPoly, neighbourTile)
                    if mid then dtVcopy(neighbourNode.pos, mid) end
                end

                local cost, heuristic
                if neighbourRef == endRef then
                    local curCost = filter:getCost(bestNode.pos, neighbourNode.pos,
                        parentRef, parentTile, parentPoly,
                        bestRef, bestTile, bestPoly,
                        neighbourRef, neighbourTile, neighbourPoly)
                    local endCost = filter:getCost(neighbourNode.pos, endPos,
                        bestRef, bestTile, bestPoly,
                        neighbourRef, neighbourTile, neighbourPoly,
                        0, nil, nil)
                    cost = bestNode.cost + curCost + endCost
                    heuristic = 0
                else
                    local curCost = filter:getCost(bestNode.pos, neighbourNode.pos,
                        parentRef, parentTile, parentPoly,
                        bestRef, bestTile, bestPoly,
                        neighbourRef, neighbourTile, neighbourPoly)
                    cost = bestNode.cost + curCost
                    heuristic = dtVdist(neighbourNode.pos, endPos) * H_SCALE
                end

                local total = cost + heuristic

                if (neighbourNode.flags % 2 == 1) and total >= neighbourNode.total then
                    goto continue_link  -- in open, worse
                end
                if (math.floor(neighbourNode.flags / 2) % 2 == 1) and total >= neighbourNode.total then
                    goto continue_link  -- in closed, worse
                end

                neighbourNode.pidx  = self._nodePool:getNodeIdx(bestNode)
                neighbourNode.id    = neighbourRef
                -- clear closed flag
                if math.floor(neighbourNode.flags / 2) % 2 == 1 then
                    neighbourNode.flags = neighbourNode.flags - 2
                end
                neighbourNode.cost  = cost
                neighbourNode.total = total

                if (neighbourNode.flags % 2) == 1 then
                    self._openList:modify(neighbourNode)
                else
                    neighbourNode.flags = neighbourNode.flags + DT_NODE_OPEN
                    self._openList:push(neighbourNode)
                end

                if heuristic < lastBestNodeCost then
                    lastBestNodeCost = heuristic
                    lastBestNode = neighbourNode
                end

                ::continue_link::
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

                if link.edge ~= segMax then goto continue_link end

                nextTile, nextPoly = self._nav:getTileAndPolyByRefUnsafe(link.ref)
                local ntype = math.floor(nextPoly.areaAndtype / 64)
                if ntype == DT_POLYTYPE_OFFMESH_CONNECTION then goto continue_link end
                if not filter:passFilter(link.ref, nextTile, nextPoly) then goto continue_link end

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
                ::continue_link::
            end

            if nextRef == 0 then
                -- hit wall
                local a = segMax
                local b = (segMax + 1 < nv) and (segMax+1) or 0
                local va = {verts[a*3+1], verts[a*3+2], verts[a*3+3]}
                local vb = {verts[b*3+1], verts[b*3+2], verts[b*3+3]}
                local dx = vb[1]-va[1]; local dz = vb[3]-va[3]
                local len = math.sqrt(dx*dx+dz*dz)
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

        local straightPath  = {}  -- array of {x,y,z} positions (1-indexed)
        local spFlags       = {}  -- array of uint8 flags
        local spRefs        = {}  -- array of poly refs
        local countRef      = {0}

        if not path or pathSize <= 0 or not path[1] then
            return {}, 0, DT_FAILURE + DT_INVALID_PARAM
        end

        local closestStart = self:closestPointOnPolyBoundary(path[1], startPos)
        if not closestStart then
            return {}, 0, DT_FAILURE + DT_INVALID_PARAM
        end

        local closestEnd = self:closestPointOnPolyBoundary(path[pathSize], endPos)
        if not closestEnd then
            return {}, 0, DT_FAILURE + DT_INVALID_PARAM
        end

        -- Add start point
        local stat = self:_appendVertex(closestStart, DT_STRAIGHTPATH_START, path[1],
            straightPath, spFlags, spRefs, countRef, maxStraight)
        if stat ~= DT_IN_PROGRESS then
            goto done
        end

        if pathSize > 1 then
            local portalApex  = {closestStart[1], closestStart[2], closestStart[3]}
            local portalLeft  = {closestStart[1], closestStart[2], closestStart[3]}
            local portalRight = {closestStart[1], closestStart[2], closestStart[3]}
            local apexIndex   = 1
            local leftIndex   = 1
            local rightIndex  = 1
            local leftPolyType  = 0
            local rightPolyType = 0
            local leftPolyRef   = path[1]
            local rightPolyRef  = path[1]

            local i = 1
            while i <= pathSize do
                local left, right
                local toType

                if i + 1 <= pathSize then
                    local l2, r2, fromType2, toType2 = self:_getPortalPoints(path[i], path[i+1])
                    if not l2 then
                        -- Failed portal: clamp end to path[i]
                        local ce2 = self:closestPointOnPolyBoundary(path[i], endPos)
                        if not ce2 then return {}, 0, DT_FAILURE + DT_INVALID_PARAM end
                        closestEnd = ce2

                        if (options % 4) >= 1 then  -- area or all crossings
                            self:_appendPortals(apexIndex, i, closestEnd, path,
                                straightPath, spFlags, spRefs, countRef, maxStraight, options)
                        end
                        self:_appendVertex(closestEnd, 0, path[i],
                            straightPath, spFlags, spRefs, countRef, maxStraight)
                        stat = DT_SUCCESS + DT_PARTIAL_RESULT
                        if countRef[1] >= maxStraight then stat = stat + DT_BUFFER_TOO_SMALL end
                        goto done
                    end
                    left = l2; right = r2; toType = toType2

                    -- If starting very close to the first portal, skip it (C++: if i==0)
                    if i == 1 then
                        local dsq = dtDistancePtSegSqr2D(portalApex, left, right)
                        if dsq < dtSqr(0.001) then
                            i = i + 1
                            goto continue_i
                        end
                    end
                else
                    -- End of path
                    left     = {closestEnd[1], closestEnd[2], closestEnd[3]}
                    right    = {closestEnd[1], closestEnd[2], closestEnd[3]}
                    toType   = DT_POLYTYPE_GROUND
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
                            if stat ~= DT_IN_PROGRESS then goto done end
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
                        if stat ~= DT_IN_PROGRESS then goto done end

                        dtVcopy(portalLeft,  portalApex)
                        dtVcopy(portalRight, portalApex)
                        leftIndex  = apexIndex
                        rightIndex = apexIndex

                        i = apexIndex
                        goto continue_i
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
                            if stat ~= DT_IN_PROGRESS then goto done end
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
                        if stat ~= DT_IN_PROGRESS then goto done end

                        dtVcopy(portalLeft,  portalApex)
                        dtVcopy(portalRight, portalApex)
                        leftIndex  = apexIndex
                        rightIndex = apexIndex

                        i = apexIndex
                        goto continue_i
                    end
                end

                ::continue_i::
                i = i + 1
            end

            -- Append portals to end
            if (options % 4) >= 1 then
                stat = self:_appendPortals(apexIndex, pathSize, closestEnd, path,
                    straightPath, spFlags, spRefs, countRef, maxStraight, options)
                if stat ~= DT_IN_PROGRESS then goto done end
            end
        end

        -- Append end
        self:_appendVertex(closestEnd, DT_STRAIGHTPATH_END, 0,
            straightPath, spFlags, spRefs, countRef, maxStraight)

        stat = DT_SUCCESS
        if countRef[1] >= maxStraight then stat = stat + DT_BUFFER_TOO_SMALL end

        ::done::

        -- Build output table
        local count = countRef[1]
        local out = {}
        for k = 1, count do
            out[k] = {pos = straightPath[k], flags = spFlags[k], ref = spRefs[k]}
        end
        return out, count, stat
    end

    return q
end

M.newFilter = newFilter

return M
