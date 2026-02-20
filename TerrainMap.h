#pragma once

#include <cstdint>
#include <cstdio>

static constexpr float TERRAIN_INVALID_HEIGHT = -200000.0f;

// Reads terrain height from a single .map grid tile.
// Ported from cmangos GridMap, stripped of all server dependencies.
class TerrainMap
{
public:
    TerrainMap();
    ~TerrainMap();

    bool load(const char* path);

    // Returns ground height at WoW world coords (x, y), or TERRAIN_INVALID_HEIGHT on failure.
    float getHeight(float x, float y) const;

    bool isLoaded() const { return m_loaded; }

private:
    static constexpr float    GRID_SIZE       = 533.33333f;
    static constexpr int      MAP_RESOLUTION  = 128;
    static constexpr int      TILE_ORIGIN     = 32;

    bool loadHeightData(FILE* f, uint32_t offset);

    bool isHole(int row, int col) const;

    float getHeightFromFlat  (float x, float y) const;
    float getHeightFromFloat (float x, float y) const;
    float getHeightFromUint8 (float x, float y) const;
    float getHeightFromUint16(float x, float y) const;

    typedef float (TerrainMap::*GetHeightFn)(float, float) const;
    GetHeightFn m_getHeight;

    // Flat / base height (used when no per-cell data)
    float m_gridHeight;
    float m_gridIntHeightMultiplier;

    // Height grids — only one set is allocated depending on encoding
    float*    m_V9;        // 129x129 outer grid
    float*    m_V8;        // 128x128 inner grid
    uint16_t* m_uint16_V9;
    uint16_t* m_uint16_V8;
    uint8_t*  m_uint8_V9;
    uint8_t*  m_uint8_V8;

    // Holes
    uint16_t m_holes[16][16];

    bool m_loaded;
};
