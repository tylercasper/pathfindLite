#include "TerrainMap.h"
#include <cstring>
#include <cmath>

// .map file format constants — must match what the extractor writes.
static const uint32_t MAP_MAGIC          = 0x5350414d; // 'MAPS'
static const uint32_t MAP_VERSION_MAGIC  = 0x342e3173; // 's1.4'
static const uint32_t MAP_HEIGHT_MAGIC   = 0x5447484d; // 'MHGT'

static const uint32_t MAP_HEIGHT_NO_HEIGHT = 0x0001;
static const uint32_t MAP_HEIGHT_AS_INT16  = 0x0002;
static const uint32_t MAP_HEIGHT_AS_INT8   = 0x0004;

static const uint16_t holetab_h[4] = { 0x1111, 0x2222, 0x4444, 0x8888 };
static const uint16_t holetab_v[4] = { 0x000F, 0x00F0, 0x0F00, 0xF000 };

// File headers — layout must exactly match GridMapDefines.h
struct MapFileHeader
{
    uint32_t mapMagic;
    uint32_t versionMagic;
    uint32_t areaMapOffset;
    uint32_t areaMapSize;
    uint32_t heightMapOffset;
    uint32_t heightMapSize;
    uint32_t liquidMapOffset;
    uint32_t liquidMapSize;
    uint32_t holesOffset;
    uint32_t holesSize;
};

struct MapHeightHeader
{
    uint32_t fourcc;
    uint32_t flags;
    float    gridHeight;
    float    gridMaxHeight;
};

// ---------------------------------------------------------------------------

TerrainMap::TerrainMap()
    : m_getHeight(&TerrainMap::getHeightFromFlat)
    , m_gridHeight(TERRAIN_INVALID_HEIGHT)
    , m_gridIntHeightMultiplier(0.0f)
    , m_V9(nullptr), m_V8(nullptr)
    , m_uint16_V9(nullptr), m_uint16_V8(nullptr)
    , m_uint8_V9(nullptr),  m_uint8_V8(nullptr)
    , m_loaded(false)
{
    memset(m_holes, 0, sizeof(m_holes));
}

TerrainMap::~TerrainMap()
{
    delete[] m_V9;        delete[] m_V8;
    delete[] m_uint16_V9; delete[] m_uint16_V8;
    delete[] m_uint8_V9;  delete[] m_uint8_V8;
}

bool TerrainMap::load(const char* path)
{
    FILE* f = fopen(path, "rb");
    if (!f)
        return false;

    MapFileHeader header;
    fread(&header, sizeof(header), 1, f);

    if (header.mapMagic != MAP_MAGIC || header.versionMagic != MAP_VERSION_MAGIC)
    {
        fclose(f);
        return false;
    }

    // Load holes so height lookups can detect them
    if (header.holesOffset)
    {
        fseek(f, header.holesOffset, SEEK_SET);
        fread(m_holes, sizeof(m_holes), 1, f);
    }

    bool ok = true;
    if (header.heightMapOffset)
        ok = loadHeightData(f, header.heightMapOffset);

    fclose(f);
    m_loaded = ok;
    return ok;
}

bool TerrainMap::loadHeightData(FILE* f, uint32_t offset)
{
    fseek(f, offset, SEEK_SET);

    MapHeightHeader hdr;
    fread(&hdr, sizeof(hdr), 1, f);

    if (hdr.fourcc != MAP_HEIGHT_MAGIC)
        return false;

    m_gridHeight = hdr.gridHeight;

    if (hdr.flags & MAP_HEIGHT_NO_HEIGHT)
    {
        m_getHeight = &TerrainMap::getHeightFromFlat;
        return true;
    }

    if (hdr.flags & MAP_HEIGHT_AS_INT16)
    {
        m_uint16_V9 = new uint16_t[129 * 129];
        m_uint16_V8 = new uint16_t[128 * 128];
        fread(m_uint16_V9, sizeof(uint16_t), 129 * 129, f);
        fread(m_uint16_V8, sizeof(uint16_t), 128 * 128, f);
        m_gridIntHeightMultiplier = (hdr.gridMaxHeight - hdr.gridHeight) / 65535.0f;
        m_getHeight = &TerrainMap::getHeightFromUint16;
    }
    else if (hdr.flags & MAP_HEIGHT_AS_INT8)
    {
        m_uint8_V9 = new uint8_t[129 * 129];
        m_uint8_V8 = new uint8_t[128 * 128];
        fread(m_uint8_V9, sizeof(uint8_t), 129 * 129, f);
        fread(m_uint8_V8, sizeof(uint8_t), 128 * 128, f);
        m_gridIntHeightMultiplier = (hdr.gridMaxHeight - hdr.gridHeight) / 255.0f;
        m_getHeight = &TerrainMap::getHeightFromUint8;
    }
    else
    {
        m_V9 = new float[129 * 129];
        m_V8 = new float[128 * 128];
        fread(m_V9, sizeof(float), 129 * 129, f);
        fread(m_V8, sizeof(float), 128 * 128, f);
        m_getHeight = &TerrainMap::getHeightFromFloat;
    }

    return true;
}

// ---------------------------------------------------------------------------

bool TerrainMap::isHole(int row, int col) const
{
    int cellRow = row / 8;
    int cellCol = col / 8;
    int holeRow = (row % 8) / 2;
    int holeCol = (col - cellCol * 8) / 2;
    uint16_t hole = m_holes[cellRow][cellCol];
    return (hole & holetab_h[holeCol] & holetab_v[holeRow]) != 0;
}

float TerrainMap::getHeight(float x, float y) const
{
    return (this->*m_getHeight)(x, y);
}

float TerrainMap::getHeightFromFlat(float /*x*/, float /*y*/) const
{
    return m_gridHeight;
}

float TerrainMap::getHeightFromFloat(float x, float y) const
{
    if (!m_V8 || !m_V9)
        return TERRAIN_INVALID_HEIGHT;

    x = MAP_RESOLUTION * (TILE_ORIGIN - x / GRID_SIZE);
    y = MAP_RESOLUTION * (TILE_ORIGIN - y / GRID_SIZE);

    int x_int = (int)x;
    int y_int = (int)y;
    x -= x_int;
    y -= y_int;
    x_int &= (MAP_RESOLUTION - 1);
    y_int &= (MAP_RESOLUTION - 1);

    if (isHole(x_int, y_int))
        return TERRAIN_INVALID_HEIGHT;

    float a, b, c;
    if (x + y < 1.0f)
    {
        if (x > y)
        {
            float h1 = m_V9[ x_int      * 129 + y_int];
            float h2 = m_V9[(x_int + 1) * 129 + y_int];
            float h5 = 2.0f * m_V8[x_int * 128 + y_int];
            a = h2 - h1; b = h5 - h1 - h2; c = h1;
        }
        else
        {
            float h1 = m_V9[x_int * 129 + y_int    ];
            float h3 = m_V9[x_int * 129 + y_int + 1];
            float h5 = 2.0f * m_V8[x_int * 128 + y_int];
            a = h5 - h1 - h3; b = h3 - h1; c = h1;
        }
    }
    else
    {
        if (x > y)
        {
            float h2 = m_V9[(x_int + 1) * 129 + y_int    ];
            float h4 = m_V9[(x_int + 1) * 129 + y_int + 1];
            float h5 = 2.0f * m_V8[x_int * 128 + y_int];
            a = h2 + h4 - h5; b = h4 - h2; c = h5 - h4;
        }
        else
        {
            float h3 = m_V9[ x_int      * 129 + y_int + 1];
            float h4 = m_V9[(x_int + 1) * 129 + y_int + 1];
            float h5 = 2.0f * m_V8[x_int * 128 + y_int];
            a = h4 - h3; b = h3 + h4 - h5; c = h5 - h4;
        }
    }

    return a * x + b * y + c;
}

float TerrainMap::getHeightFromUint8(float x, float y) const
{
    if (!m_uint8_V8 || !m_uint8_V9)
        return m_gridHeight;

    x = MAP_RESOLUTION * (TILE_ORIGIN - x / GRID_SIZE);
    y = MAP_RESOLUTION * (TILE_ORIGIN - y / GRID_SIZE);

    int x_int = (int)x;
    int y_int = (int)y;
    x -= x_int;
    y -= y_int;
    x_int &= (MAP_RESOLUTION - 1);
    y_int &= (MAP_RESOLUTION - 1);

    if (isHole(x_int, y_int))
        return TERRAIN_INVALID_HEIGHT;

    uint8_t* ptr = &m_uint8_V9[x_int * 128 + x_int + y_int];
    int32_t a, b, c;
    if (x + y < 1.0f)
    {
        if (x > y)
        {
            a = ptr[129] - ptr[0];
            b = 2 * m_uint8_V8[x_int * 128 + y_int] - ptr[0] - ptr[129];
            c = ptr[0];
        }
        else
        {
            a = 2 * m_uint8_V8[x_int * 128 + y_int] - ptr[0] - ptr[1];
            b = ptr[1] - ptr[0];
            c = ptr[0];
        }
    }
    else
    {
        if (x > y)
        {
            a = ptr[129] + ptr[130] - 2 * m_uint8_V8[x_int * 128 + y_int];
            b = ptr[130] - ptr[129];
            c = 2 * m_uint8_V8[x_int * 128 + y_int] - ptr[130];
        }
        else
        {
            a = ptr[130] - ptr[1];
            b = ptr[1] + ptr[130] - 2 * m_uint8_V8[x_int * 128 + y_int];
            c = 2 * m_uint8_V8[x_int * 128 + y_int] - ptr[130];
        }
    }

    return (a * x + b * y + c) * m_gridIntHeightMultiplier + m_gridHeight;
}

float TerrainMap::getHeightFromUint16(float x, float y) const
{
    if (!m_uint16_V8 || !m_uint16_V9)
        return m_gridHeight;

    x = MAP_RESOLUTION * (TILE_ORIGIN - x / GRID_SIZE);
    y = MAP_RESOLUTION * (TILE_ORIGIN - y / GRID_SIZE);

    int x_int = (int)x;
    int y_int = (int)y;
    x -= x_int;
    y -= y_int;
    x_int &= (MAP_RESOLUTION - 1);
    y_int &= (MAP_RESOLUTION - 1);

    if (isHole(x_int, y_int))
        return TERRAIN_INVALID_HEIGHT;

    uint16_t* ptr = &m_uint16_V9[x_int * 128 + x_int + y_int];
    int32_t a, b, c;
    if (x + y < 1.0f)
    {
        if (x > y)
        {
            a = ptr[129] - ptr[0];
            b = 2 * m_uint16_V8[x_int * 128 + y_int] - ptr[0] - ptr[129];
            c = ptr[0];
        }
        else
        {
            a = 2 * m_uint16_V8[x_int * 128 + y_int] - ptr[0] - ptr[1];
            b = ptr[1] - ptr[0];
            c = ptr[0];
        }
    }
    else
    {
        if (x > y)
        {
            a = ptr[129] + ptr[130] - 2 * m_uint16_V8[x_int * 128 + y_int];
            b = ptr[130] - ptr[129];
            c = 2 * m_uint16_V8[x_int * 128 + y_int] - ptr[130];
        }
        else
        {
            a = ptr[130] - ptr[1];
            b = ptr[1] + ptr[130] - 2 * m_uint16_V8[x_int * 128 + y_int];
            c = 2 * m_uint16_V8[x_int * 128 + y_int] - ptr[130];
        }
    }

    return (a * x + b * y + c) * m_gridIntHeightMultiplier + m_gridHeight;
}
