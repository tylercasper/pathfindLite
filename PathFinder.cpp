#include "PathFinder.h"

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"

#include <cstdio>
#include <cmath>
#include <algorithm>

// Must match the header written by MoveMapGen into each .mmtile file.
#define MMAP_MAGIC   0x4d4d4150u  // 'MMAP'
#define MMAP_VERSION 8u

// Valid WoW world coordinate range (yards from map center)
static constexpr float WORLD_MAX = 17066.666f;

struct MmapTileHeader
{
    uint32_t mmapMagic;
    uint32_t dtVersion;
    uint32_t mmapVersion;
    uint32_t size;
    uint32_t usesLiquids;
};

static inline uint32_t packTileID(uint32_t tx, uint32_t ty)
{
    return (tx << 16) | ty;
}

// ---------------------------------------------------------------------------

PathFinder::PathFinder(const std::string& dataDir, uint32_t mapId)
    : m_dataDir(dataDir)
    , m_mapId(mapId)
    , m_navMesh(nullptr)
    , m_navQuery(nullptr)
    , m_initialized(false)
{
    char path[512];
    snprintf(path, sizeof(path), "%s/mmaps/%03u.mmap", dataDir.c_str(), mapId);
    fprintf(stderr, "[pathfind] loading navmesh params: %s\n", path);

    FILE* fp = fopen(path, "rb");
    if (!fp)
    {
        fprintf(stderr, "[pathfind] ERROR: could not open %s\n", path);
        return;
    }

    dtNavMeshParams params;
    fread(&params, sizeof(params), 1, fp);
    fclose(fp);

    fprintf(stderr, "[pathfind] navmesh params: orig=(%.2f,%.2f,%.2f) tileW=%.2f tileH=%.2f maxTiles=%d maxPolys=%d\n",
            params.orig[0], params.orig[1], params.orig[2],
            params.tileWidth, params.tileHeight,
            params.maxTiles, params.maxPolys);

    m_navMesh = dtAllocNavMesh();
    if (!m_navMesh || dtStatusFailed(m_navMesh->init(&params)))
    {
        fprintf(stderr, "[pathfind] ERROR: dtNavMesh::init failed\n");
        return;
    }

    m_navQuery = dtAllocNavMeshQuery();
    if (!m_navQuery || dtStatusFailed(m_navQuery->init(m_navMesh, 65535)))
    {
        fprintf(stderr, "[pathfind] ERROR: dtNavMeshQuery::init failed\n");
        return;
    }

    fprintf(stderr, "[pathfind] navmesh ready\n");
    m_initialized = true;
}

PathFinder::~PathFinder()
{
    if (m_navQuery) dtFreeNavMeshQuery(m_navQuery);
    if (m_navMesh)  dtFreeNavMesh(m_navMesh);
}

// ---------------------------------------------------------------------------

// WoW world space (x, y, z) -> Recast space (y, z, x)
// Matches the conversion in cmangos PathFinder::BuildPolyPath (PathFinder.cpp:411).
void PathFinder::toRecast(float x, float y, float z, float* out)
{
    out[0] = y;
    out[1] = z;
    out[2] = x;
}

// Tile coords are computed from WoW x/y directly, same formula as the server's
// MoveMap loader and RecastDemoMod's GetGridCoord.
bool PathFinder::worldToTile(float x, float y, uint32_t& tx, uint32_t& ty) const
{
    float ftx = TILE_ORIGIN - x / BLOCK_SIZE;
    float fty = TILE_ORIGIN - y / BLOCK_SIZE;
    if (ftx < 0.0f || ftx >= 64.0f || fty < 0.0f || fty >= 64.0f)
    {
        fprintf(stderr, "[pathfind] ERROR: coordinates (%.2f, %.2f) out of world bounds "
                        "(tile %.2f, %.2f — expected [0,64))\n", x, y, ftx, fty);
        return false;
    }
    tx = static_cast<uint32_t>(ftx);
    ty = static_cast<uint32_t>(fty);
    return true;
}

bool PathFinder::loadNavTile(uint32_t tx, uint32_t ty)
{
    uint32_t id = packTileID(tx, ty);
    if (m_loadedNavTiles.count(id))
        return true;

    char path[512];
    snprintf(path, sizeof(path), "%s/mmaps/%03u%02u%02u.mmtile",
             m_dataDir.c_str(), m_mapId, tx, ty);
    fprintf(stderr, "[pathfind] loading nav tile (%u,%u): %s\n", tx, ty, path);

    FILE* fp = fopen(path, "rb");
    if (!fp)
    {
        fprintf(stderr, "[pathfind] WARNING: nav tile not found\n");
        return false;
    }

    MmapTileHeader header;
    fread(&header, sizeof(header), 1, fp);

    if (header.mmapMagic != MMAP_MAGIC || header.mmapVersion != MMAP_VERSION)
    {
        fprintf(stderr, "[pathfind] ERROR: bad mmtile header (magic=0x%08x ver=%u)\n",
                header.mmapMagic, header.mmapVersion);
        fclose(fp);
        return false;
    }

    unsigned char* data = static_cast<unsigned char*>(dtAlloc(header.size, DT_ALLOC_PERM));
    if (!data)
    {
        fclose(fp);
        return false;
    }

    fread(data, header.size, 1, fp);
    fclose(fp);

    dtTileRef tileRef = 0;
    if (dtStatusFailed(m_navMesh->addTile(data, header.size, DT_TILE_FREE_DATA, 0, &tileRef)))
    {
        fprintf(stderr, "[pathfind] ERROR: addTile failed\n");
        dtFree(data);
        return false;
    }

    fprintf(stderr, "[pathfind] nav tile (%u,%u) loaded OK (%u bytes)\n", tx, ty, header.size);
    m_loadedNavTiles.insert(id);
    return true;
}

void PathFinder::ensureNavTilesLoaded(float x1, float y1, float x2, float y2)
{
    uint32_t tx1, ty1, tx2, ty2;
    if (!worldToTile(x1, y1, tx1, ty1)) return;
    if (!worldToTile(x2, y2, tx2, ty2)) return;

    uint32_t txMin = std::min(tx1, tx2);
    uint32_t txMax = std::max(tx1, tx2);
    uint32_t tyMin = std::min(ty1, ty2);
    uint32_t tyMax = std::max(ty1, ty2);

    fprintf(stderr, "[pathfind] tile range x:[%u,%u] y:[%u,%u]\n", txMin, txMax, tyMin, tyMax);

    for (uint32_t tx = txMin; tx <= txMax; ++tx)
        for (uint32_t ty = tyMin; ty <= tyMax; ++ty)
            loadNavTile(tx, ty);
}

float PathFinder::getTerrainHeight(float x, float y)
{
    uint32_t tx, ty;
    if (!worldToTile(x, y, tx, ty))
        return TERRAIN_INVALID_HEIGHT;

    uint32_t id = packTileID(tx, ty);

    auto it = m_terrainTiles.find(id);
    if (it == m_terrainTiles.end())
    {
        TerrainMap& tile = m_terrainTiles[id];

        char path[512];
        snprintf(path, sizeof(path), "%s/maps/%03u%02u%02u.map",
                 m_dataDir.c_str(), m_mapId, tx, ty);
        fprintf(stderr, "[pathfind] loading terrain tile (%u,%u): %s\n", tx, ty, path);

        bool ok = tile.load(path);
        fprintf(stderr, "[pathfind] terrain tile (%u,%u) %s\n", tx, ty, ok ? "loaded OK" : "FAILED (no .map file?)");

        float h = tile.getHeight(x, y);
        fprintf(stderr, "[pathfind] terrain height at (%.2f, %.2f) = %.4f\n", x, y, h);
        return h;
    }

    return it->second.getHeight(x, y);
}

// ---------------------------------------------------------------------------

float PathFinder::computeDistance(float x1, float y1, float x2, float y2)
{
    fprintf(stderr, "[pathfind] computeDistance (%.4f, %.4f) -> (%.4f, %.4f)\n", x1, y1, x2, y2);

    if (fabsf(x1) > WORLD_MAX || fabsf(y1) > WORLD_MAX ||
        fabsf(x2) > WORLD_MAX || fabsf(y2) > WORLD_MAX)
    {
        fprintf(stderr, "[pathfind] ERROR: coordinates exceed world bounds (±%.0f yards)\n", WORLD_MAX);
        return -1.0f;
    }

    if (!m_initialized)
    {
        fprintf(stderr, "[pathfind] ERROR: not initialized\n");
        return -1.0f;
    }

    ensureNavTilesLoaded(x1, y1, x2, y2);

    float z1 = getTerrainHeight(x1, y1);
    float z2 = getTerrainHeight(x2, y2);

    if (z1 == TERRAIN_INVALID_HEIGHT || z2 == TERRAIN_INVALID_HEIGHT)
    {
        fprintf(stderr, "[pathfind] ERROR: terrain height lookup failed (z1=%.2f z2=%.2f)\n", z1, z2);
        return -1.0f;
    }

    float startPos[3], endPos[3];
    toRecast(x1, y1, z1, startPos);
    toRecast(x2, y2, z2, endPos);
    fprintf(stderr, "[pathfind] recast start=(%.2f,%.2f,%.2f) end=(%.2f,%.2f,%.2f)\n",
            startPos[0], startPos[1], startPos[2],
            endPos[0],   endPos[1],   endPos[2]);

    const float extents[3] = { 2.0f, 4.0f, 2.0f };

    dtQueryFilter filter;
    dtPolyRef startRef = 0, endRef = 0;
    m_navQuery->findNearestPoly(startPos, extents, &filter, &startRef, nullptr);
    m_navQuery->findNearestPoly(endPos,   extents, &filter, &endRef,   nullptr);
    fprintf(stderr, "[pathfind] startRef=%llu endRef=%llu\n",
            (unsigned long long)startRef, (unsigned long long)endRef);

    if (!startRef || !endRef)
    {
        fprintf(stderr, "[pathfind] ERROR: could not find nearest poly (try widening extents?)\n");
        return -1.0f;
    }

    dtPolyRef polys[MAX_POLYS];
    int npolys = 0;
    m_navQuery->findPath(startRef, endRef, startPos, endPos,
                         &filter, polys, &npolys, MAX_POLYS);
    fprintf(stderr, "[pathfind] findPath: %d polys\n", npolys);

    if (!npolys)
        return -1.0f;

    // If the path doesn't reach the target polygon, clamp end to the last reachable poly.
    float endPosAdj[3];
    dtVcopy(endPosAdj, endPos);
    if (polys[npolys - 1] != endRef)
    {
        fprintf(stderr, "[pathfind] path incomplete — clamping to last reachable poly\n");
        m_navQuery->closestPointOnPoly(polys[npolys - 1], endPos, endPosAdj, nullptr);
    }

    float straightPath[MAX_POLYS * 3];
    unsigned char straightPathFlags[MAX_POLYS];
    dtPolyRef straightPathPolys[MAX_POLYS];
    int nstraight = 0;
    m_navQuery->findStraightPath(startPos, endPosAdj, polys, npolys,
                                 straightPath, straightPathFlags, straightPathPolys,
                                 &nstraight, MAX_POLYS);
    fprintf(stderr, "[pathfind] findStraightPath: %d points\n", nstraight);

    if (nstraight < 2)
        return -1.0f;

    float total = 0.0f;
    for (int i = 0; i < nstraight - 1; ++i)
    {
        const float* a = &straightPath[i * 3];
        const float* b = &straightPath[(i + 1) * 3];
        float dx = b[0] - a[0], dy = b[1] - a[1], dz = b[2] - a[2];
        total += sqrtf(dx*dx + dy*dy + dz*dz);
    }

    fprintf(stderr, "[pathfind] total distance: %.4f\n", total);
    return total;
}
