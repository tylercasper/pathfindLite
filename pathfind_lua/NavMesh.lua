-- NavMesh.lua
-- Port of DetourNavMesh.cpp to Lua 5.1
-- DT_POLYREF64 bit layout: salt=16 bits, tile=28 bits, poly=20 bits (per instructions)
-- NOTE: The C++ source defines DT_SALT_BITS=12, DT_TILE_BITS=21, DT_POLY_BITS=31 for DT_POLYREF64,
--       but the task instructions override these as salt=16, tile=28, poly=20.

local bin = require("binary")

local M = {}

-- Constants
local DT_VERTS_PER_POLYGON    = 6
-- DT_NAVMESH_MAGIC = 'D'<<24 | 'N'<<16 | 'A'<<8 | 'V' = 0x444E4156
local DT_NAVMESH_MAGIC        = 0x44 * 16777216 + 0x4E * 65536 + 0x41 * 256 + 0x56  -- 'DNAV'
local DT_NAVMESH_VERSION      = 7
local DT_NULL_LINK            = 0xffffffff
local DT_EXT_LINK             = 0x8000
local DT_OFFMESH_CON_BIDIR    = 1
local DT_POLYTYPE_GROUND      = 0
local DT_POLYTYPE_OFFMESH_CONNECTION = 1
local DT_DETAIL_EDGE_BOUNDARY = 0x01

-- Poly ref bit layout (per task instructions)
local DT_SALT_BITS = 16
local DT_TILE_BITS = 28
local DT_POLY_BITS = 20
local DT_SALT_MASK = (2^DT_SALT_BITS) - 1   -- 0xFFFF
local DT_TILE_MASK = (2^DT_TILE_BITS) - 1   -- 0x0FFFFFFF
local DT_POLY_MASK = (2^DT_POLY_BITS) - 1   -- 0x000FFFFF
-- Shift values:
--   poly is in bits [0..19]
--   tile is in bits [20..47]
--   salt is in bits [48..63]
local DT_POLY_SHIFT = 1               -- placeholder (handled below)
local DT_TILE_SHIFT = 2^DT_POLY_BITS  -- 2^20 = 1048576
local DT_SALT_SHIFT = 2^(DT_POLY_BITS + DT_TILE_BITS)  -- 2^48

-- MMAP constants
local MMAP_MAGIC   = 0x4d4d4150
local MMAP_VERSION = 8

-- ---------------------------------------------------------------------------
-- Poly ref encoding/decoding
-- ---------------------------------------------------------------------------

local function encodePolyId(salt, it, ip)
    -- ref = salt * 2^(POLY+TILE) + it * 2^POLY + ip
    return salt * DT_SALT_SHIFT + it * DT_TILE_SHIFT + ip
end

local function decodePolyId(ref)
    local ip   = ref % DT_TILE_SHIFT                           -- lower POLY_BITS
    local rest = (ref - ip) / DT_TILE_SHIFT
    local it   = rest % (DT_TILE_MASK + 1)                    -- next TILE_BITS
    local salt = (rest - it) / (DT_TILE_MASK + 1)             -- upper SALT_BITS
    return salt, it, ip
end

local function decodePolyIdTile(ref)
    local ip   = ref % DT_TILE_SHIFT
    local rest = (ref - ip) / DT_TILE_SHIFT
    return rest % (DT_TILE_MASK + 1)
end

local function decodePolyIdSalt(ref)
    local ip   = ref % DT_TILE_SHIFT
    local rest = (ref - ip) / DT_TILE_SHIFT
    local it   = rest % (DT_TILE_MASK + 1)
    return (rest - it) / (DT_TILE_MASK + 1)
end

-- ---------------------------------------------------------------------------
-- Math helpers (inline translations of DetourCommon.h)
-- ---------------------------------------------------------------------------

local function dtMax(a, b) return a > b and a or b end
local function dtMin(a, b) return a < b and a or b end
local function dtAbs(a) return a < 0 and -a or a end
local function dtSqr(a) return a*a end
local function dtClamp(v, mn, mx) return v < mn and mn or (v > mx and mx or v) end

local function dtVdist(v1, v2)
    local dx = v2[1]-v1[1]; local dy = v2[2]-v1[2]; local dz = v2[3]-v1[3]
    return math.sqrt(dx*dx + dy*dy + dz*dz)
end

local function dtVdistSqr(v1, v2)
    local dx = v2[1]-v1[1]; local dy = v2[2]-v1[2]; local dz = v2[3]-v1[3]
    return dx*dx + dy*dy + dz*dz
end

local function dtVlenSqr(v)
    return v[1]*v[1] + v[2]*v[2] + v[3]*v[3]
end

local function dtVequal(p0, p1)
    local thr = dtSqr(1.0/16384.0)
    return dtVdistSqr(p0, p1) < thr
end

local function dtVcopy(dst, src)
    dst[1] = src[1]; dst[2] = src[2]; dst[3] = src[3]
end

local function dtVsub(dst, v1, v2)
    dst[1] = v1[1]-v2[1]; dst[2] = v1[2]-v2[2]; dst[3] = v1[3]-v2[3]
end

local function dtVadd(dst, v1, v2)
    dst[1] = v1[1]+v2[1]; dst[2] = v1[2]+v2[2]; dst[3] = v1[3]+v2[3]
end

local function dtVlerp(dst, v1, v2, t)
    dst[1] = v1[1] + (v2[1]-v1[1])*t
    dst[2] = v1[2] + (v2[2]-v1[2])*t
    dst[3] = v1[3] + (v2[3]-v1[3])*t
end

local function dtVmin(mn, v)
    if v[1] < mn[1] then mn[1] = v[1] end
    if v[2] < mn[2] then mn[2] = v[2] end
    if v[3] < mn[3] then mn[3] = v[3] end
end

local function dtVmax(mx, v)
    if v[1] > mx[1] then mx[1] = v[1] end
    if v[2] > mx[2] then mx[2] = v[2] end
    if v[3] > mx[3] then mx[3] = v[3] end
end

local function dtTriArea2D(a, b, c)
    local abx = b[1]-a[1]; local abz = b[3]-a[3]
    local acx = c[1]-a[1]; local acz = c[3]-a[3]
    return acx*abz - abx*acz
end

local function dtOverlapBounds(amin, amax, bmin, bmax)
    if amin[1] > bmax[1] or amax[1] < bmin[1] then return false end
    if amin[2] > bmax[2] or amax[2] < bmin[2] then return false end
    if amin[3] > bmax[3] or amax[3] < bmin[3] then return false end
    return true
end

local function dtOverlapQuantBounds(amin, amax, bmin, bmax)
    if amin[1] > bmax[1] or amax[1] < bmin[1] then return false end
    if amin[2] > bmax[2] or amax[2] < bmin[2] then return false end
    if amin[3] > bmax[3] or amax[3] < bmin[3] then return false end
    return true
end

local function nextPow2(v)
    if v <= 1 then return 1 end
    local p = 1
    while p < v do p = p * 2 end
    return p
end

local function dtOppositeTile(side)
    return (side + 4) % 8
end

local function dtAlign4(x)
    return math.floor((x + 3) / 4) * 4
end

-- dtPointInPolygon: point-in-polygon test on xz plane
-- verts is a flat array {x,y,z, x,y,z, ...} 1-indexed
local function dtPointInPolygon(pt, verts, nverts)
    local c = false
    local j = nverts
    for i = 1, nverts do
        local vi_x = verts[i*3-2]; local vi_z = verts[i*3]
        local vj_x = verts[j*3-2]; local vj_z = verts[j*3]
        if ((vi_z > pt[3]) ~= (vj_z > pt[3])) and
           (pt[1] < (vj_x - vi_x) * (pt[3] - vi_z) / (vj_z - vi_z) + vi_x) then
            c = not c
        end
        j = i
    end
    return c
end

-- dtClosestHeightPointTriangle
local function dtClosestHeightPointTriangle(p, a, b, c)
    local EPS = 1e-6
    -- v0 = c - a, v1 = b - a, v2 = p - a
    local v0x = c[1]-a[1]; local v0y = c[2]-a[2]; local v0z = c[3]-a[3]
    local v1x = b[1]-a[1]; local v1y = b[2]-a[2]; local v1z = b[3]-a[3]
    local v2x = p[1]-a[1];                         local v2z = p[3]-a[3]

    local denom = v0x * v1z - v0z * v1x
    if math.abs(denom) < EPS then return false, 0 end

    local u = v1z * v2x - v1x * v2z
    local v = v0x * v2z - v0z * v2x

    if denom < 0 then
        denom = -denom; u = -u; v = -v
    end

    if u >= 0 and v >= 0 and (u + v) <= denom then
        local h = a[2] + (v0y * u + v1y * v) / denom
        return true, h
    end
    return false, 0
end

-- dtDistancePtSegSqr2D
local function dtDistancePtSegSqr2D(pt, p, q)
    local pqx = q[1]-p[1]; local pqz = q[3]-p[3]
    local dx  = pt[1]-p[1]; local dz  = pt[3]-p[3]
    local d = pqx*pqx + pqz*pqz
    local t = pqx*dx + pqz*dz
    if d > 0 then t = t/d end
    if t < 0 then t = 0 elseif t > 1 then t = 1 end
    dx = p[1] + t*pqx - pt[1]
    dz = p[3] + t*pqz - pt[3]
    return dx*dx + dz*dz, t
end

-- dtDistancePtPolyEdgesSqr
local function dtDistancePtPolyEdgesSqr(pt, verts, nverts, ed, et)
    local c = false
    local j = nverts
    for i = 1, nverts do
        local vi_x = verts[i*3-2]; local vi_z = verts[i*3]
        local vj_x = verts[j*3-2]; local vj_z = verts[j*3]
        if ((vi_z > pt[3]) ~= (vj_z > pt[3])) and
           (pt[1] < (vj_x - vi_x) * (pt[3] - vi_z) / (vj_z - vi_z) + vi_x) then
            c = not c
        end
        -- store dist to edge j
        local vj = {verts[j*3-2], verts[j*3-1], verts[j*3]}
        local vi = {verts[i*3-2], verts[i*3-1], verts[i*3]}
        local d, t = dtDistancePtSegSqr2D(pt, vj, vi)
        ed[j] = d
        et[j] = t
        j = i
    end
    return c
end

-- ---------------------------------------------------------------------------
-- Tile hash (for spatial lookup)
-- ---------------------------------------------------------------------------

local function computeTileHash(x, y, mask)
    local H1 = 0x8da6b343
    local H2 = 0xd8163841
    -- simulate unsigned 32-bit multiply with wrapping
    local function umul32(a, b)
        -- keep only lower 32 bits
        return (a * b) % 4294967296
    end
    local n = (umul32(H1, x % 4294967296) + umul32(H2, y % 4294967296)) % 4294967296
    return n % (mask + 1)
end

-- ---------------------------------------------------------------------------
-- Slab helpers for connectExtLinks
-- ---------------------------------------------------------------------------

local function getSlabCoord(va, side)
    if side == 0 or side == 4 then return va[1] end
    if side == 2 or side == 6 then return va[3] end
    return 0
end

local function calcSlabEndPoints(va, vb, side)
    local bmin = {0,0}; local bmax = {0,0}
    if side == 0 or side == 4 then
        if va[3] < vb[3] then
            bmin[1] = va[3]; bmin[2] = va[2]; bmax[1] = vb[3]; bmax[2] = vb[2]
        else
            bmin[1] = vb[3]; bmin[2] = vb[2]; bmax[1] = va[3]; bmax[2] = va[2]
        end
    elseif side == 2 or side == 6 then
        if va[1] < vb[1] then
            bmin[1] = va[1]; bmin[2] = va[2]; bmax[1] = vb[1]; bmax[2] = vb[2]
        else
            bmin[1] = vb[1]; bmin[2] = vb[2]; bmax[1] = va[1]; bmax[2] = va[2]
        end
    end
    return bmin, bmax
end

local function overlapSlabs(amin, amax, bmin, bmax, px, py)
    local minx = dtMax(amin[1]+px, bmin[1]+px)
    local maxx = dtMin(amax[1]-px, bmax[1]-px)
    if minx > maxx then return false end

    local ad = (amax[2]-amin[2]) / (amax[1]-amin[1])
    local ak = amin[2] - ad*amin[1]
    local bd = (bmax[2]-bmin[2]) / (bmax[1]-bmin[1])
    local bk = bmin[2] - bd*bmin[1]
    local aminy = ad*minx + ak; local amaxy = ad*maxx + ak
    local bminy = bd*minx + bk; local bmaxy = bd*maxx + bk
    local dmin = bminy - aminy; local dmax = bmaxy - amaxy

    if dmin*dmax < 0 then return true end

    local thr = dtSqr(py*2)
    if dmin*dmin <= thr or dmax*dmax <= thr then return true end
    return false
end

-- ---------------------------------------------------------------------------
-- closestPointOnDetailEdges (template<bool onlyBoundary>)
-- ---------------------------------------------------------------------------

local function closestPointOnDetailEdges(tile, poly, pos, onlyBoundary)
    local ip = poly._index  -- 0-based index of poly in tile.polys
    local pd = tile.detailMeshes[ip + 1]  -- 1-based
    if not pd then return {pos[1], pos[2], pos[3]} end

    local dmin = math.huge
    local tmin_out = 0
    local pmin_out = nil
    local pmax_out = nil

    local function getDetailVert(idx)
        if idx < poly.vertCount then
            local vi = poly.verts[idx + 1]
            return {tile.verts[vi*3+1], tile.verts[vi*3+2], tile.verts[vi*3+3]}
        else
            local dvi = pd.vertBase + (idx - poly.vertCount)
            return {tile.detailVerts[dvi*3+1], tile.detailVerts[dvi*3+2], tile.detailVerts[dvi*3+3]}
        end
    end

    for ti = 0, pd.triCount - 1 do
        local triIdx = (pd.triBase + ti) * 4 + 1  -- 1-based index into detailTris
        local t0 = tile.detailTris[triIdx]
        local t1 = tile.detailTris[triIdx+1]
        local t2 = tile.detailTris[triIdx+2]
        local t3 = tile.detailTris[triIdx+3]

        if onlyBoundary then
            -- ANY_BOUNDARY_EDGE: bits 0, 2, 4 of t3
            local bit0 = t3 % 2
            local bit2 = math.floor(t3 / 4) % 2
            local bit4 = math.floor(t3 / 16) % 2
            if bit0 == 0 and bit2 == 0 and bit4 == 0 then
                -- no boundary edges in this triangle, skip
                goto continue_tri
            end
        end

        local tidx = {t0, t1, t2}
        local tv = {}
        for k = 1, 3 do
            tv[k] = getDetailVert(tidx[k])
        end

        -- Iterate over 3 edges: (j=3,k=1), (j=1,k=2), (j=2,k=3) in 1-based
        -- C++ iterates k=0..2 with j starting at 2 (0-based).
        -- Edge j->k where j goes: 2,0,1 and k goes: 0,1,2 (0-based)
        -- In 1-based: j goes: 3,1,2 and k goes: 1,2,3
        local jseq = {3, 1, 2}
        local kseq = {1, 2, 3}
        for ei = 1, 3 do
            local j1 = jseq[ei]  -- 1-based
            local k1 = kseq[ei]  -- 1-based
            local j0 = j1 - 1    -- 0-based (for C++ edge index)
            -- edgeFlag = (t3 >> (j0*2)) & 0x3
            local edgeFlag = math.floor(t3 / (4^j0)) % 4
            local isBoundary = (edgeFlag % 2) == 1

            local skip = false
            if not isBoundary then
                if onlyBoundary then
                    skip = true
                else
                    -- Inner edge: only visit once, skip if tris[j] >= tris[k]
                    -- (0-based comparison)
                    if tidx[j1] >= tidx[k1] then
                        skip = true
                    end
                end
            end

            if not skip then
                local d, t = dtDistancePtSegSqr2D(pos, tv[j1], tv[k1])
                if d < dmin then
                    dmin = d; tmin_out = t
                    pmin_out = tv[j1]; pmax_out = tv[k1]
                end
            end
        end
        ::continue_tri::
    end

    if pmin_out == nil then
        return {pos[1], pos[2], pos[3]}
    end

    local closest = {0,0,0}
    dtVlerp(closest, pmin_out, pmax_out, tmin_out)
    return closest
end

-- ---------------------------------------------------------------------------
-- Binary parsing of tile data
-- ---------------------------------------------------------------------------

-- Sizes (must match C++ structs with dtAlign4)
-- dtMeshHeader: 15 ints + 10 floats
-- Fields: magic(i32) version(i32) x(i32) y(i32) layer(i32) userId(u32)
--         polyCount(i32) vertCount(i32) maxLinkCount(i32) detailMeshCount(i32)
--         detailVertCount(i32) detailTriCount(i32) bvNodeCount(i32) offMeshConCount(i32) offMeshBase(i32)
--         walkableHeight(f) walkableRadius(f) walkableClimb(f)
--         bmin[3](f,f,f) bmax[3](f,f,f) bvQuantFactor(f)
-- Total: 15*4 + (3+3+3+1)*4 = 60 + 40 = 100 bytes, aligned to 4 = 100
local HEADER_SIZE = dtAlign4(15*4 + (3+3+3+1)*4)  -- = 100

-- dtPoly: firstLink(u32) + verts[6](u16*6) + neis[6](u16*6) + flags(u16) + vertCount(u8) + areaAndtype(u8)
-- = 4 + 12 + 12 + 2 + 1 + 1 = 32 bytes
local POLY_SIZE = 32

-- dtPolyDetail: vertBase(u32) + triBase(u32) + vertCount(u8) + triCount(u8) [+ 2 pad bytes]
-- = 4 + 4 + 1 + 1 + 2 = 12 bytes
local POLYDETAIL_SIZE = 12

-- dtLink (DT_POLYREF64): ref(u64=8) + next(u32) + edge(u8) + side(u8) + bmin(u8) + bmax(u8)
-- = 8 + 4 + 4 = 16 bytes
local LINK_SIZE = 16

-- float triplet = 12 bytes
-- dtBVNode: bmin[3](u16*3) + bmax[3](u16*3) + i(i32) = 6+6+4 = 16 bytes
local BVNODE_SIZE = 16

-- dtOffMeshConnection: pos[6](f*6) + rad(f) + poly(u16) + flags(u8) + side(u8) + userId(u32)
-- = 24 + 4 + 2 + 1 + 1 + 4 = 36 bytes
local OFFMESHCON_SIZE = 36

local function parseMeshHeader(data, offset)
    local s = data
    local i = offset
    local hdr = {}
    hdr.magic,         i = bin.readI32(s, i)
    hdr.version,       i = bin.readI32(s, i)
    hdr.x,             i = bin.readI32(s, i)
    hdr.y,             i = bin.readI32(s, i)
    hdr.layer,         i = bin.readI32(s, i)
    hdr.userId,        i = bin.readU32(s, i)
    hdr.polyCount,     i = bin.readI32(s, i)
    hdr.vertCount,     i = bin.readI32(s, i)
    hdr.maxLinkCount,  i = bin.readI32(s, i)
    hdr.detailMeshCount, i = bin.readI32(s, i)
    hdr.detailVertCount, i = bin.readI32(s, i)
    hdr.detailTriCount,  i = bin.readI32(s, i)
    hdr.bvNodeCount,   i = bin.readI32(s, i)
    hdr.offMeshConCount, i = bin.readI32(s, i)
    hdr.offMeshBase,   i = bin.readI32(s, i)
    hdr.walkableHeight,i = bin.readFloat(s, i)
    hdr.walkableRadius,i = bin.readFloat(s, i)
    hdr.walkableClimb, i = bin.readFloat(s, i)
    local bmin0, bmin1, bmin2
    bmin0, i = bin.readFloat(s, i)
    bmin1, i = bin.readFloat(s, i)
    bmin2, i = bin.readFloat(s, i)
    hdr.bmin = {bmin0, bmin1, bmin2}
    local bmax0, bmax1, bmax2
    bmax0, i = bin.readFloat(s, i)
    bmax1, i = bin.readFloat(s, i)
    bmax2, i = bin.readFloat(s, i)
    hdr.bmax = {bmax0, bmax1, bmax2}
    hdr.bvQuantFactor, i = bin.readFloat(s, i)
    return hdr, i
end

local function parseTileData(nm, data)
    -- data is a Lua string containing the raw tile bytes
    -- Parse dtMeshHeader starting at byte 1
    local hdr, pos = parseMeshHeader(data, 1)

    if hdr.magic ~= DT_NAVMESH_MAGIC then
        return nil, "wrong magic"
    end
    if hdr.version ~= DT_NAVMESH_VERSION then
        return nil, "wrong version"
    end

    -- Advance to aligned position after header
    pos = HEADER_SIZE + 1

    local tile = {
        header       = hdr,
        salt         = 1,
        linksFreeList= 0,
        verts        = {},
        polys        = {},
        links        = {},
        detailMeshes = {},
        detailVerts  = {},
        detailTris   = {},
        bvTree       = nil,
        offMeshCons  = {},
        _index       = nil,  -- set when inserted
        next         = nil,
    }

    -- Calculate section sizes (aligned to 4)
    local vertsSize      = dtAlign4(12 * hdr.vertCount)
    local polysSize      = dtAlign4(POLY_SIZE * hdr.polyCount)
    local linksSize      = dtAlign4(LINK_SIZE * hdr.maxLinkCount)
    local detailMeshSize = dtAlign4(POLYDETAIL_SIZE * hdr.detailMeshCount)
    local detailVertsSize= dtAlign4(12 * hdr.detailVertCount)
    local detailTrisSize = dtAlign4(4 * hdr.detailTriCount)
    local bvtreeSize     = dtAlign4(BVNODE_SIZE * hdr.bvNodeCount)
    local offMeshSize    = dtAlign4(OFFMESHCON_SIZE * hdr.offMeshConCount)

    -- Parse verts (floats, triplets)
    local vertsStart = pos
    for k = 0, hdr.vertCount - 1 do
        local vx, vy, vz
        vx, pos = bin.readFloat(data, pos)
        vy, pos = bin.readFloat(data, pos)
        vz, pos = bin.readFloat(data, pos)
        tile.verts[k*3+1] = vx
        tile.verts[k*3+2] = vy
        tile.verts[k*3+3] = vz
    end
    pos = vertsStart + vertsSize

    -- Parse polys
    local polysStart = pos
    for k = 0, hdr.polyCount - 1 do
        local pstart = pos
        local p = {}
        p._index = k
        p.firstLink, pos = bin.readU32(data, pos)
        p.verts = {}
        for vi = 1, DT_VERTS_PER_POLYGON do
            local v; v, pos = bin.readU16(data, pos)
            p.verts[vi] = v
        end
        p.neis = {}
        for ni = 1, DT_VERTS_PER_POLYGON do
            local n; n, pos = bin.readU16(data, pos)
            p.neis[ni] = n
        end
        p.flags,       pos = bin.readU16(data, pos)
        p.vertCount,   pos = bin.readU8(data, pos)
        p.areaAndtype, pos = bin.readU8(data, pos)
        tile.polys[k+1] = p
        pos = pstart + POLY_SIZE
    end
    pos = polysStart + polysSize

    -- Parse links (allocate maxLinkCount slots)
    local linksStart = pos
    for k = 0, hdr.maxLinkCount - 1 do
        local lstart = pos
        local lk = {}
        lk.ref,  pos = bin.readU64(data, pos)   -- dtPolyRef (64-bit)
        lk.next, pos = bin.readU32(data, pos)
        lk.edge, pos = bin.readU8(data, pos)
        lk.side, pos = bin.readU8(data, pos)
        lk.bmin, pos = bin.readU8(data, pos)
        lk.bmax, pos = bin.readU8(data, pos)
        tile.links[k+1] = lk
        pos = lstart + LINK_SIZE
    end
    pos = linksStart + linksSize

    -- Parse detailMeshes
    local dmStart = pos
    for k = 0, hdr.detailMeshCount - 1 do
        local dstart = pos
        local dm = {}
        dm.vertBase,  pos = bin.readU32(data, pos)
        dm.triBase,   pos = bin.readU32(data, pos)
        dm.vertCount, pos = bin.readU8(data, pos)
        dm.triCount,  pos = bin.readU8(data, pos)
        -- 2 padding bytes
        pos = pos + 2
        tile.detailMeshes[k+1] = dm
        pos = dstart + POLYDETAIL_SIZE
    end
    pos = dmStart + detailMeshSize

    -- Parse detailVerts
    local dvStart = pos
    for k = 0, hdr.detailVertCount - 1 do
        local vx, vy, vz
        vx, pos = bin.readFloat(data, pos)
        vy, pos = bin.readFloat(data, pos)
        vz, pos = bin.readFloat(data, pos)
        tile.detailVerts[k*3+1] = vx
        tile.detailVerts[k*3+2] = vy
        tile.detailVerts[k*3+3] = vz
    end
    pos = dvStart + detailVertsSize

    -- Parse detailTris (4 uint8 each)
    local dtStart = pos
    for k = 0, hdr.detailTriCount - 1 do
        local v0, v1, v2, flags
        v0,    pos = bin.readU8(data, pos)
        v1,    pos = bin.readU8(data, pos)
        v2,    pos = bin.readU8(data, pos)
        flags, pos = bin.readU8(data, pos)
        tile.detailTris[k*4+1] = v0
        tile.detailTris[k*4+2] = v1
        tile.detailTris[k*4+3] = v2
        tile.detailTris[k*4+4] = flags
    end
    pos = dtStart + detailTrisSize

    -- Parse BV nodes
    if hdr.bvNodeCount > 0 then
        tile.bvTree = {}
        local bvStart = pos
        for k = 0, hdr.bvNodeCount - 1 do
            local nstart = pos
            local node = {}
            local b0,b1,b2
            b0, pos = bin.readU16(data, pos)
            b1, pos = bin.readU16(data, pos)
            b2, pos = bin.readU16(data, pos)
            node.bmin = {b0, b1, b2}
            b0, pos = bin.readU16(data, pos)
            b1, pos = bin.readU16(data, pos)
            b2, pos = bin.readU16(data, pos)
            node.bmax = {b0, b1, b2}
            node.i, pos = bin.readI32(data, pos)
            tile.bvTree[k+1] = node
            pos = nstart + BVNODE_SIZE
        end
        pos = bvStart + bvtreeSize
    else
        pos = pos + bvtreeSize
    end

    -- Parse offMeshCons
    local omStart = pos
    for k = 0, hdr.offMeshConCount - 1 do
        local ostart = pos
        local con = {}
        con.pos = {}
        for pi = 1, 6 do con.pos[pi], pos = bin.readFloat(data, pos) end
        con.rad,    pos = bin.readFloat(data, pos)
        con.poly,   pos = bin.readU16(data, pos)
        con.flags,  pos = bin.readU8(data, pos)
        con.side,   pos = bin.readU8(data, pos)
        con.userId, pos = bin.readU32(data, pos)
        tile.offMeshCons[k+1] = con
        pos = ostart + OFFMESHCON_SIZE
    end

    return tile
end

-- ---------------------------------------------------------------------------
-- NavMesh object
-- ---------------------------------------------------------------------------

function M.new(paramsData)
    -- paramsData is a Lua string containing dtNavMeshParams (binary)
    -- dtNavMeshParams: orig[3](f,f,f) tileWidth(f) tileHeight(f) maxTiles(i32) maxPolys(i32)
    -- = 3*4 + 4 + 4 + 4 + 4 = 28 bytes
    local i = 1
    local p = {}
    local ox, oy, oz
    ox, i = bin.readFloat(paramsData, i)
    oy, i = bin.readFloat(paramsData, i)
    oz, i = bin.readFloat(paramsData, i)
    p.orig = {ox, oy, oz}
    p.tileWidth,  i = bin.readFloat(paramsData, i)
    p.tileHeight, i = bin.readFloat(paramsData, i)
    p.maxTiles,   i = bin.readI32(paramsData, i)
    p.maxPolys,   i = bin.readI32(paramsData, i)

    local nm = {}
    nm._params      = p
    nm._orig        = {p.orig[1], p.orig[2], p.orig[3]}
    nm._tileWidth   = p.tileWidth
    nm._tileHeight  = p.tileHeight
    nm._maxTiles    = p.maxTiles

    -- Tile lookup table (LUT), size = next power of 2 of maxTiles/4
    local lutSize = nextPow2(math.max(1, math.floor(p.maxTiles / 4)))
    nm._tileLutSize = lutSize
    nm._tileLutMask = lutSize - 1
    nm._posLookup   = {}   -- hash -> tile (linked via .next)
    nm._tiles       = {}   -- index 1..maxTiles, each slot is a tile or nil

    -- Initialize tile array with free list
    nm._nextFree = nil
    for idx = p.maxTiles, 1, -1 do
        nm._tiles[idx] = {salt=1, next=nm._nextFree, header=nil, _tileIndex=idx-1}
        nm._nextFree = nm._tiles[idx]
    end

    -- Methods
    local self = nm

    -- getPolyRefBase(tile)
    function self:getPolyRefBase(tile)
        if not tile then return 0 end
        local it = tile._tileIndex
        return encodePolyId(tile.salt, it, 0)
    end

    -- getTileRef(tile)
    function self:getTileRef(tile)
        if not tile then return 0 end
        local it = tile._tileIndex
        return encodePolyId(tile.salt, it, 0)
    end

    -- getTileAt(x, y, layer)
    function self:getTileAt(x, y, layer)
        local h = computeTileHash(x, y, self._tileLutMask)
        local tile = self._posLookup[h]
        while tile do
            if tile.header and
               tile.header.x == x and
               tile.header.y == y and
               tile.header.layer == layer then
                return tile
            end
            tile = tile.next
        end
        return nil
    end

    -- getTilesAt(x, y) -> array
    function self:getTilesAt(x, y)
        local result = {}
        local h = computeTileHash(x, y, self._tileLutMask)
        local tile = self._posLookup[h]
        while tile do
            if tile.header and
               tile.header.x == x and
               tile.header.y == y then
                result[#result+1] = tile
            end
            tile = tile.next
        end
        return result
    end

    -- getNeighbourTilesAt(x, y, side)
    function self:getNeighbourTilesAt(x, y, side)
        local nx, ny = x, y
        if     side == 0 then nx = nx+1
        elseif side == 1 then nx = nx+1; ny = ny+1
        elseif side == 2 then ny = ny+1
        elseif side == 3 then nx = nx-1; ny = ny+1
        elseif side == 4 then nx = nx-1
        elseif side == 5 then nx = nx-1; ny = ny-1
        elseif side == 6 then ny = ny-1
        elseif side == 7 then nx = nx+1; ny = ny-1
        end
        return self:getTilesAt(nx, ny)
    end

    -- calcTileLoc(pos) -> tx, ty
    function self:calcTileLoc(pos)
        local tx = math.floor((pos[1] - self._orig[1]) / self._tileWidth)
        local ty = math.floor((pos[3] - self._orig[3]) / self._tileHeight)
        return tx, ty
    end

    -- isValidPolyRef(ref)
    function self:isValidPolyRef(ref)
        if not ref or ref == 0 then return false end
        local salt, it, ip = decodePolyId(ref)
        it = it + 1  -- 1-based tile index
        if it > self._maxTiles then return false end
        local tile = self._tiles[it]
        if not tile or tile.salt ~= salt or not tile.header then return false end
        if ip >= tile.header.polyCount then return false end
        return true
    end

    -- getTileAndPolyByRef(ref) -> tile, poly (or nil, err)
    function self:getTileAndPolyByRef(ref)
        if not ref or ref == 0 then return nil, nil end
        local salt, it, ip = decodePolyId(ref)
        it = it + 1
        if it > self._maxTiles then return nil, nil end
        local tile = self._tiles[it]
        if not tile or tile.salt ~= salt or not tile.header then return nil, nil end
        if ip >= tile.header.polyCount then return nil, nil end
        return tile, tile.polys[ip+1]
    end

    -- getTileAndPolyByRefUnsafe(ref) -> tile, poly
    function self:getTileAndPolyByRefUnsafe(ref)
        local salt, it, ip = decodePolyId(ref)
        it = it + 1
        local tile = self._tiles[it]
        return tile, tile.polys[ip+1]
    end

    -- allocLink(tile) -> link index (0-based) or DT_NULL_LINK
    local function allocLink(tile)
        if tile.linksFreeList == DT_NULL_LINK then return DT_NULL_LINK end
        local link = tile.linksFreeList
        tile.linksFreeList = tile.links[link+1].next
        return link
    end

    local function freeLink(tile, link)
        tile.links[link+1].next = tile.linksFreeList
        tile.linksFreeList = link
    end

    -- queryPolygonsInTile (used internally for nearest poly / off-mesh)
    function self:queryPolygonsInTile(tile, qmin, qmax, polys, maxPolys)
        local n = 0
        local base = self:getPolyRefBase(tile)

        if tile.bvTree then
            local tbmin = tile.header.bmin
            local tbmax = tile.header.bmax
            local qfac  = tile.header.bvQuantFactor

            local minx = dtClamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1]
            local miny = dtClamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2]
            local minz = dtClamp(qmin[3], tbmin[3], tbmax[3]) - tbmin[3]
            local maxx = dtClamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1]
            local maxy = dtClamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2]
            local maxz = dtClamp(qmax[3], tbmin[3], tbmax[3]) - tbmin[3]

            -- Quantize: bmin &= 0xfffe (clear bit 0), bmax |= 1 (set bit 0)
            -- C++: bmin[i] = (unsigned short)(qfac * min) & 0xfffe
            --      bmax[i] = (unsigned short)(qfac * max + 1) | 1
            local function quantMin(v)
                local u = math.min(math.floor(qfac * v), 65535)
                return u - (u % 2)  -- & 0xfffe
            end
            local function quantMax(v)
                local u = math.min(math.floor(qfac * v + 1), 65535)
                if u % 2 == 0 then u = u + 1 end  -- | 1
                return u
            end
            local bmin = { quantMin(minx), quantMin(miny), quantMin(minz) }
            local bmax = { quantMax(maxx), quantMax(maxy), quantMax(maxz) }

            local nodeIdx = 1
            local nodeEnd = tile.header.bvNodeCount + 1
            while nodeIdx < nodeEnd do
                local node = tile.bvTree[nodeIdx]
                local isLeaf = node.i >= 0
                local overlap = dtOverlapQuantBounds(bmin, bmax, node.bmin, node.bmax)

                if isLeaf and overlap then
                    if n < maxPolys then
                        n = n + 1
                        polys[n] = base + node.i
                    end
                end

                if overlap or isLeaf then
                    nodeIdx = nodeIdx + 1
                else
                    local escapeIndex = -node.i
                    nodeIdx = nodeIdx + escapeIndex
                end
            end
        else
            for k = 0, tile.header.polyCount - 1 do
                local p = tile.polys[k+1]
                if p.areaAndtype ~= nil then
                    local ptype = math.floor(p.areaAndtype / 64)
                    if ptype ~= DT_POLYTYPE_OFFMESH_CONNECTION then
                        -- compute bounds
                        local vi0 = p.verts[1]
                        local bmin = {tile.verts[vi0*3+1], tile.verts[vi0*3+2], tile.verts[vi0*3+3]}
                        local bmax = {bmin[1], bmin[2], bmin[3]}
                        for vi = 2, p.vertCount do
                            local vii = p.verts[vi]
                            local vv = {tile.verts[vii*3+1], tile.verts[vii*3+2], tile.verts[vii*3+3]}
                            dtVmin(bmin, vv)
                            dtVmax(bmax, vv)
                        end
                        if dtOverlapBounds(qmin, qmax, bmin, bmax) then
                            if n < maxPolys then
                                n = n + 1
                                polys[n] = base + k
                            end
                        end
                    end
                end
            end
        end
        return n
    end

    -- getPolyHeight(tile, poly, pos) -> bool, height
    function self:getPolyHeight(tile, poly, pos)
        local ptype = math.floor(poly.areaAndtype / 64)
        if ptype == DT_POLYTYPE_OFFMESH_CONNECTION then
            return false, 0
        end

        local ip = poly._index
        local pd = tile.detailMeshes[ip+1]
        if not pd then return false, 0 end

        -- Build flat verts array for dtPointInPolygon
        local verts = {}
        local nv = poly.vertCount
        for vi = 1, nv do
            local vidx = poly.verts[vi]
            verts[vi*3-2] = tile.verts[vidx*3+1]
            verts[vi*3-1] = tile.verts[vidx*3+2]
            verts[vi*3  ] = tile.verts[vidx*3+3]
        end

        if not dtPointInPolygon(pos, verts, nv) then
            return false, 0
        end

        -- Helper: get a vertex from detail tri index (0-based idx)
        local function getTriVert(idx)
            if idx < poly.vertCount then
                local vi = poly.verts[idx+1]
                return {tile.verts[vi*3+1], tile.verts[vi*3+2], tile.verts[vi*3+3]}
            else
                local dvi = pd.vertBase + (idx - poly.vertCount)
                return {tile.detailVerts[dvi*3+1], tile.detailVerts[dvi*3+2], tile.detailVerts[dvi*3+3]}
            end
        end

        -- Find height from detail triangles
        for ti = 0, pd.triCount - 1 do
            local tidx = (pd.triBase + ti) * 4 + 1
            local t0 = tile.detailTris[tidx]
            local t1 = tile.detailTris[tidx+1]
            local t2 = tile.detailTris[tidx+2]

            local va = getTriVert(t0)
            local vb = getTriVert(t1)
            local vc = getTriVert(t2)

            local ok, h = dtClosestHeightPointTriangle(pos, va, vb, vc)
            if ok then
                return true, h
            end
        end

        -- Fallback: closest point on detail edges (non-boundary)
        local closest = closestPointOnDetailEdges(tile, poly, pos, false)
        return true, closest[2]
    end

    -- closestPointOnPoly(ref, pos) -> closest, posOverPoly
    function self:closestPointOnPoly(ref, pos)
        local tile, poly = self:getTileAndPolyByRefUnsafe(ref)

        local closest = {pos[1], pos[2], pos[3]}
        local ok, h = self:getPolyHeight(tile, poly, pos)
        if ok then
            closest[2] = h
            return closest, true
        end

        local ptype = math.floor(poly.areaAndtype / 64)
        if ptype == DT_POLYTYPE_OFFMESH_CONNECTION then
            local v0i = poly.verts[1]; local v1i = poly.verts[2]
            local v0 = {tile.verts[v0i*3+1], tile.verts[v0i*3+2], tile.verts[v0i*3+3]}
            local v1 = {tile.verts[v1i*3+1], tile.verts[v1i*3+2], tile.verts[v1i*3+3]}
            local _, t = dtDistancePtSegSqr2D(pos, v0, v1)
            dtVlerp(closest, v0, v1, t)
            return closest, false
        end

        local c = closestPointOnDetailEdges(tile, poly, pos, true)
        return c, false
    end

    -- findNearestPolyInTile(tile, center, halfExtents) -> ref, nearestPt
    function self:findNearestPolyInTile(tile, center, halfExtents)
        local bmin = {center[1]-halfExtents[1], center[2]-halfExtents[2], center[3]-halfExtents[3]}
        local bmax = {center[1]+halfExtents[1], center[2]+halfExtents[2], center[3]+halfExtents[3]}
        local polys = {}
        local polyCount = self:queryPolygonsInTile(tile, bmin, bmax, polys, 128)

        local nearest = 0
        local nearestDistSqr = math.huge
        local nearestPt = {center[1], center[2], center[3]}

        for k = 1, polyCount do
            local ref = polys[k]
            local closestPtPoly, posOverPoly = self:closestPointOnPoly(ref, center)
            local diff = {center[1]-closestPtPoly[1], center[2]-closestPtPoly[2], center[3]-closestPtPoly[3]}
            local d
            if posOverPoly then
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
        end

        return nearest, nearestPt
    end

    -- findConnectingPolys(va, vb, tile, side) -> con[], conarea[], n
    function self:findConnectingPolys(va, vb, tile, side)
        if not tile then return {}, {}, 0 end
        local amin, amax = calcSlabEndPoints(va, vb, side)
        local apos = getSlabCoord(va, side)
        -- m = DT_EXT_LINK | side  (matches the neis[] value for this side)
        local m = DT_EXT_LINK + side  -- DT_EXT_LINK=0x8000, side in [0..7]
        local con = {}; local conarea = {}; local n = 0
        local base = self:getPolyRefBase(tile)

        for k = 0, tile.header.polyCount - 1 do
            local poly = tile.polys[k+1]
            local nv = poly.vertCount
            local found = false
            for j = 1, nv do
                if not found and poly.neis[j] == m then
                    local vci = poly.verts[j]
                    local vdi = poly.verts[(j % nv) + 1]
                    local vc = {tile.verts[vci*3+1], tile.verts[vci*3+2], tile.verts[vci*3+3]}
                    local vd = {tile.verts[vdi*3+1], tile.verts[vdi*3+2], tile.verts[vdi*3+3]}
                    local bpos = getSlabCoord(vc, side)
                    if dtAbs(apos - bpos) <= 0.01 then
                        local bmin2, bmax2 = calcSlabEndPoints(vc, vd, side)
                        if overlapSlabs(amin, amax, bmin2, bmax2, 0.01, tile.header.walkableClimb) then
                            n = n + 1
                            conarea[n*2-1] = dtMax(amin[1], bmin2[1])
                            conarea[n*2  ] = dtMin(amax[1], bmax2[1])
                            con[n] = base + k
                            found = true  -- break equivalent
                        end
                    end
                end
            end
        end
        return con, conarea, n
    end

    -- connectIntLinks(tile)
    function self:connectIntLinks(tile)
        local base = self:getPolyRefBase(tile)
        for k = 0, tile.header.polyCount - 1 do
            local poly = tile.polys[k+1]
            poly.firstLink = DT_NULL_LINK

            local ptype = math.floor(poly.areaAndtype / 64)
            if ptype == DT_POLYTYPE_OFFMESH_CONNECTION then
                -- skip off-mesh polys
            else
                for j = poly.vertCount - 1, 0, -1 do
                    local nei = poly.neis[j+1]
                    -- Skip: nei==0 (no connection), or nei has DT_EXT_LINK bit set (0x8000)
                    if nei ~= 0 and nei < DT_EXT_LINK then
                        -- internal link: neighbour poly index is (nei - 1)
                        local idx = allocLink(tile)
                        if idx ~= DT_NULL_LINK then
                            local lk = tile.links[idx+1]
                            lk.ref  = base + (nei - 1)
                            lk.edge = j
                            lk.side = 0xff
                            lk.bmin = 0; lk.bmax = 0
                            lk.next = poly.firstLink
                            poly.firstLink = idx
                        end
                    end
                end
            end
        end
    end

    -- baseOffMeshLinks(tile)
    function self:baseOffMeshLinks(tile)
        local base = self:getPolyRefBase(tile)
        for k = 1, tile.header.offMeshConCount do
            local con = tile.offMeshCons[k]
            local poly = tile.polys[con.poly + 1]  -- poly index is 0-based

            local halfExtents = {con.rad, tile.header.walkableClimb, con.rad}
            local p = {con.pos[1], con.pos[2], con.pos[3]}  -- first vertex
            local ref, nearestPt = self:findNearestPolyInTile(tile, p, halfExtents)
            if ref == 0 then goto continue_con end
            local dx = nearestPt[1]-p[1]; local dz = nearestPt[3]-p[3]
            if dx*dx + dz*dz > con.rad * con.rad then goto continue_con end

            -- Update vertex 0 of the poly to nearestPt
            local vi = poly.verts[1]
            tile.verts[vi*3+1] = nearestPt[1]
            tile.verts[vi*3+2] = nearestPt[2]
            tile.verts[vi*3+3] = nearestPt[3]

            -- Link off-mesh con -> landing poly
            local idx = allocLink(tile)
            if idx ~= DT_NULL_LINK then
                local lk = tile.links[idx+1]
                lk.ref = ref; lk.edge = 0; lk.side = 0xff
                lk.bmin = 0; lk.bmax = 0
                lk.next = poly.firstLink
                poly.firstLink = idx
            end

            -- Link landing poly -> off-mesh con
            local tidx = allocLink(tile)
            if tidx ~= DT_NULL_LINK then
                local _, _, ip = decodePolyId(ref)
                local landPoly = tile.polys[ip+1]
                local lk2 = tile.links[tidx+1]
                lk2.ref = base + con.poly
                lk2.edge = 0xff; lk2.side = 0xff
                lk2.bmin = 0; lk2.bmax = 0
                lk2.next = landPoly.firstLink
                landPoly.firstLink = tidx
            end
            ::continue_con::
        end
    end

    -- connectExtLinks(tile, target, side)
    function self:connectExtLinks(tile, target, side)
        if not tile then return end
        for k = 0, tile.header.polyCount - 1 do
            local poly = tile.polys[k+1]
            local nv = poly.vertCount
            for j = 1, nv do
                local nei = poly.neis[j]  -- neis[j] is 1-based, corresponds to edge j-1 (0-based)
                -- Only process edges with DT_EXT_LINK set
                if nei >= DT_EXT_LINK then
                    local dir = nei % 256  -- lower byte = direction
                    if side == -1 or dir == side then
                        local vai = poly.verts[j]
                        local vbi_idx = (j % nv) + 1
                        local vbi = poly.verts[vbi_idx]
                        local va = {tile.verts[vai*3+1], tile.verts[vai*3+2], tile.verts[vai*3+3]}
                        local vb = {tile.verts[vbi*3+1], tile.verts[vbi*3+2], tile.verts[vbi*3+3]}
                        local con, neia, nnei = self:findConnectingPolys(va, vb, target, dtOppositeTile(dir))

                        for kk = 1, nnei do
                            local idx = allocLink(tile)
                            if idx ~= DT_NULL_LINK then
                                local lk = tile.links[idx+1]
                                lk.ref  = con[kk]
                                lk.edge = j - 1  -- 0-based edge index
                                lk.side = dir
                                lk.bmin = 0; lk.bmax = 0

                                if dir == 0 or dir == 4 then
                                    local tmin = (neia[kk*2-1] - va[3]) / (vb[3] - va[3])
                                    local tmax = (neia[kk*2  ] - va[3]) / (vb[3] - va[3])
                                    if tmin > tmax then local tmp=tmin; tmin=tmax; tmax=tmp end
                                    lk.bmin = math.floor(dtClamp(tmin, 0, 1) * 255 + 0.5)
                                    lk.bmax = math.floor(dtClamp(tmax, 0, 1) * 255 + 0.5)
                                elseif dir == 2 or dir == 6 then
                                    local tmin = (neia[kk*2-1] - va[1]) / (vb[1] - va[1])
                                    local tmax = (neia[kk*2  ] - va[1]) / (vb[1] - va[1])
                                    if tmin > tmax then local tmp=tmin; tmin=tmax; tmax=tmp end
                                    lk.bmin = math.floor(dtClamp(tmin, 0, 1) * 255 + 0.5)
                                    lk.bmax = math.floor(dtClamp(tmax, 0, 1) * 255 + 0.5)
                                end

                                lk.next = poly.firstLink
                                poly.firstLink = idx
                            end
                        end
                    end
                end
            end
        end
    end

    -- connectExtOffMeshLinks(tile, target, side)
    function self:connectExtOffMeshLinks(tile, target, side)
        if not tile then return end
        local oppositeSide = (side == -1) and 0xff or dtOppositeTile(side)

        for k = 1, target.header.offMeshConCount do
            local targetCon = target.offMeshCons[k]
            if targetCon.side ~= oppositeSide then goto continue_con end
            local targetPoly = target.polys[targetCon.poly + 1]
            if targetPoly.firstLink == DT_NULL_LINK then goto continue_con end

            local halfExtents = {targetCon.rad, target.header.walkableClimb, targetCon.rad}
            local p = {targetCon.pos[4], targetCon.pos[5], targetCon.pos[6]}
            local ref, nearestPt = self:findNearestPolyInTile(tile, p, halfExtents)
            if ref == 0 then goto continue_con end
            local dx = nearestPt[1]-p[1]; local dz = nearestPt[3]-p[3]
            if dx*dx + dz*dz > targetCon.rad * targetCon.rad then goto continue_con end

            -- Update vertex 1 of targetPoly
            local vi1 = targetPoly.verts[2]
            target.verts[vi1*3+1] = nearestPt[1]
            target.verts[vi1*3+2] = nearestPt[2]
            target.verts[vi1*3+3] = nearestPt[3]

            -- Link target poly -> tile poly
            local idx = allocLink(target)
            if idx ~= DT_NULL_LINK then
                local lk = target.links[idx+1]
                lk.ref = ref; lk.edge = 1; lk.side = oppositeSide
                lk.bmin = 0; lk.bmax = 0
                lk.next = targetPoly.firstLink
                targetPoly.firstLink = idx
            end

            -- Bidirectional: link tile poly -> target poly
            if (targetCon.flags % 2) == DT_OFFMESH_CON_BIDIR then
                local tidx = allocLink(tile)
                if tidx ~= DT_NULL_LINK then
                    local _, _, ip = decodePolyId(ref)
                    local landPoly = tile.polys[ip+1]
                    local base = self:getPolyRefBase(target)
                    local lk2 = tile.links[tidx+1]
                    lk2.ref = base + targetCon.poly
                    lk2.edge = 0xff
                    lk2.side = (side == -1) and 0xff or side
                    lk2.bmin = 0; lk2.bmax = 0
                    lk2.next = landPoly.firstLink
                    landPoly.firstLink = tidx
                end
            end
            ::continue_con::
        end
    end

    -- addTile(data) -> success, error
    -- data is a Lua string starting with MmapTileHeader then tile blob
    function self:addTile(data)
        -- Parse MmapTileHeader: magic(u32) dtVersion(u32) mmapVersion(u32) size(u32) usesLiquids(u32)
        local i = 1
        local magic, dtVer, mmapVer, sz, usesLiq
        magic,    i = bin.readU32(data, i)
        dtVer,    i = bin.readU32(data, i)
        mmapVer,  i = bin.readU32(data, i)
        sz,       i = bin.readU32(data, i)
        usesLiq,  i = bin.readU32(data, i)

        if magic ~= MMAP_MAGIC then return false, "bad mmap magic" end
        if mmapVer ~= MMAP_VERSION then return false, "bad mmap version" end

        -- The actual tile data follows after the 20-byte MmapTileHeader
        local tileData = data:sub(i)

        local tile, err = parseTileData(self, tileData)
        if not tile then return false, err end
        local hdr = tile.header

        -- Check if slot occupied
        if self:getTileAt(hdr.x, hdr.y, hdr.layer) then
            return false, "tile already occupied"
        end

        -- Get a free tile slot
        local tileSlot = self._nextFree
        if not tileSlot then return false, "no free tile slots" end
        self._nextFree = tileSlot.next
        tileSlot.next = nil

        -- Copy parsed data into tile slot
        for k, v in pairs(tile) do
            tileSlot[k] = v
        end
        -- tileSlot._tileIndex is already set; preserve salt
        -- salt stays as-is (incremented on remove, but we keep it)

        -- Insert into spatial LUT
        local h = computeTileHash(hdr.x, hdr.y, self._tileLutMask)
        tileSlot.next = self._posLookup[h]
        self._posLookup[h] = tileSlot

        -- Build links free list
        tileSlot.linksFreeList = 0
        tileSlot.links[hdr.maxLinkCount].next = DT_NULL_LINK
        for li = 0, hdr.maxLinkCount - 2 do
            tileSlot.links[li+1].next = li + 1
        end

        -- Build internal links
        self:connectIntLinks(tileSlot)

        -- Base off-mesh links and connect within same tile
        self:baseOffMeshLinks(tileSlot)
        self:connectExtOffMeshLinks(tileSlot, tileSlot, -1)

        -- Connect with layers in current cell
        local sameCell = self:getTilesAt(hdr.x, hdr.y)
        for _, nb in ipairs(sameCell) do
            if nb ~= tileSlot then
                self:connectExtLinks(tileSlot, nb, -1)
                self:connectExtLinks(nb, tileSlot, -1)
                self:connectExtOffMeshLinks(tileSlot, nb, -1)
                self:connectExtOffMeshLinks(nb, tileSlot, -1)
            end
        end

        -- Connect with neighbours (8 directions)
        for sideDir = 0, 7 do
            local neis = self:getNeighbourTilesAt(hdr.x, hdr.y, sideDir)
            for _, nb in ipairs(neis) do
                self:connectExtLinks(tileSlot, nb, sideDir)
                self:connectExtLinks(nb, tileSlot, dtOppositeTile(sideDir))
                self:connectExtOffMeshLinks(tileSlot, nb, sideDir)
                self:connectExtOffMeshLinks(nb, tileSlot, dtOppositeTile(sideDir))
            end
        end

        return true
    end

    return self
end

-- Expose helpers for NavMeshQuery
M.encodePolyId         = encodePolyId
M.decodePolyId         = decodePolyId
M.decodePolyIdTile     = decodePolyIdTile
M.decodePolyIdSalt     = decodePolyIdSalt
M.DT_NULL_LINK         = DT_NULL_LINK
M.DT_EXT_LINK          = DT_EXT_LINK
M.DT_POLYTYPE_GROUND   = DT_POLYTYPE_GROUND
M.DT_POLYTYPE_OFFMESH_CONNECTION = DT_POLYTYPE_OFFMESH_CONNECTION
M.dtVdist              = dtVdist
M.dtVdistSqr           = dtVdistSqr
M.dtVlenSqr            = dtVlenSqr
M.dtVequal             = dtVequal
M.dtVcopy              = dtVcopy
M.dtVsub               = dtVsub
M.dtVadd               = dtVadd
M.dtVlerp              = dtVlerp
M.dtVmin               = dtVmin
M.dtVmax               = dtVmax
M.dtTriArea2D          = dtTriArea2D
M.dtOverlapBounds      = dtOverlapBounds
M.dtOverlapQuantBounds = dtOverlapQuantBounds
M.dtDistancePtSegSqr2D = dtDistancePtSegSqr2D
M.dtDistancePtPolyEdgesSqr = dtDistancePtPolyEdgesSqr
M.dtClamp              = dtClamp
M.dtMax                = dtMax
M.dtMin                = dtMin
M.dtAbs                = dtAbs
M.dtSqr                = dtSqr
M.nextPow2             = nextPow2
M.dtOppositeTile       = dtOppositeTile
M.MMAP_MAGIC           = MMAP_MAGIC
M.MMAP_VERSION         = MMAP_VERSION
M.DT_TILE_SHIFT        = DT_TILE_SHIFT
M.DT_TILE_MASK         = DT_TILE_MASK

-- dtIntersectSegSeg2D
function M.dtIntersectSegSeg2D(ap, aq, bp, bq)
    local function vperpXZ(a, b) return a[1]*b[3] - a[3]*b[1] end
    local u = {aq[1]-ap[1], aq[2]-ap[2], aq[3]-ap[3]}
    local v = {bq[1]-bp[1], bq[2]-bp[2], bq[3]-bp[3]}
    local w = {ap[1]-bp[1], ap[2]-bp[2], ap[3]-bp[3]}
    local d = vperpXZ(u, v)
    if math.abs(d) < 1e-6 then return false, 0, 0 end
    local s = vperpXZ(v, w) / d
    local t = vperpXZ(u, w) / d
    return true, s, t
end

-- dtIntersectSegmentPoly2D
-- verts: flat 1-based array {x,y,z, x,y,z, ...}
-- nverts: number of vertices
-- Returns: hit(bool), tmin, tmax, segMin(0-based), segMax(0-based)
function M.dtIntersectSegmentPoly2D(p0, p1, verts, nverts)
    local EPS = 0.000001
    local tmin = 0; local tmax = 1
    local segMin = -1; local segMax = -1

    -- dir = p1 - p0
    local dirx = p1[1]-p0[1]
    local dirz = p1[3]-p0[3]

    -- Iterate edges (j -> i), j starts at nverts (last vertex)
    -- C++ uses 0-based: for(int i=0, j=nverts-1; i<nverts; j=i++)
    -- Edge is from verts[j] to verts[i]
    local j = nverts  -- 1-based last vertex
    for i = 1, nverts do
        -- edge = verts[i] - verts[j]
        local edgex = verts[i*3-2] - verts[j*3-2]
        local edgez = verts[i*3  ] - verts[j*3  ]
        -- diff = p0 - verts[j]
        local diffx = p0[1] - verts[j*3-2]
        local diffz = p0[3] - verts[j*3  ]

        -- n = dtVperp2D(edge, diff) = edge.z*diff.x - edge.x*diff.z
        local n = edgez * diffx - edgex * diffz
        -- d = dtVperp2D(dir, edge) = dir.z*edge.x - dir.x*edge.z
        local d = dirz * edgex - dirx * edgez

        if math.abs(d) < EPS then
            if n < 0 then return false, 0, 0, -1, -1 end
            -- else: parallel and inside, continue
        else
            local t = n / d
            if d < 0 then
                -- entering
                if t > tmin then
                    tmin = t
                    segMin = j - 1  -- 0-based edge index
                    if tmin > tmax then return false, 0, 0, -1, -1 end
                end
            else
                -- leaving
                if t < tmax then
                    tmax = t
                    segMax = j - 1  -- 0-based edge index
                    if tmax < tmin then return false, 0, 0, -1, -1 end
                end
            end
        end
        j = i
    end

    return true, tmin, tmax, segMin, segMax
end

return M
