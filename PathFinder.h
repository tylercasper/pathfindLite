#pragma once

#include "TerrainMap.h"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>

struct dtNavMesh;
struct dtNavMeshQuery;

// Computes navmesh path distances between WoW world coordinates using pre-built .mmap/.mmtile files.
//
// Accepts WoW horizontal (x, y) coordinates.  The z (height) is resolved
// automatically from the .map terrain files stored under dataDir/maps/.
//
// Usage:
//   PathFinder pf("/path/to/data", 0);    // 0 = Eastern Kingdoms
//   if (pf.isValid())
//       float dist = pf.computeDistance(x1, y1, x2, y2);
class PathFinder
{
public:
    // dataDir: directory containing the mmaps/ and maps/ subdirectories
    // mapId:   WoW map ID (0=Eastern Kingdoms, 1=Kalimdor, 530=Outland, etc.)
    PathFinder(const std::string& dataDir, uint32_t mapId);
    ~PathFinder();

    PathFinder(const PathFinder&) = delete;
    PathFinder& operator=(const PathFinder&) = delete;

    // Coordinates in WoW world space (x/y are the horizontal plane).
    // Z is resolved from .map terrain files automatically.
    // Returns total path distance in world units, or -1.0f if no path was found.
    float computeDistance(float x1, float y1, float x2, float y2);

    bool isValid() const { return m_initialized; }

private:
    static constexpr float    BLOCK_SIZE  = 533.33333f;
    static constexpr uint32_t TILE_ORIGIN = 32;
    static constexpr int      MAX_POLYS   = 4096;

    // WoW (x, y, z) -> Recast (y, z, x)
    static void toRecast(float x, float y, float z, float* out);

    // Tile coords are derived from WoW x/y directly (same formula as the server).
    bool worldToTile(float x, float y, uint32_t& tx, uint32_t& ty) const;

    bool loadNavTile(uint32_t tx, uint32_t ty);
    void ensureNavTilesLoaded(float x1, float y1, float x2, float y2);

    // Returns the terrain height at WoW (x, y), loading the .map tile if needed.
    float getTerrainHeight(float x, float y);

    std::string                               m_dataDir;
    uint32_t                                  m_mapId;
    dtNavMesh*                                m_navMesh;
    dtNavMeshQuery*                           m_navQuery;
    std::unordered_set<uint32_t>              m_loadedNavTiles;
    std::unordered_map<uint32_t, TerrainMap>  m_terrainTiles;
    bool                                      m_initialized;
};
