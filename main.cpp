#include "PathFinder.h"

#include <cstdio>
#include <cstdlib>

// Usage: pathfind <dataDir> <mapId> <x1> <y1> <x2> <y2>
//
// dataDir  – directory containing the mmaps/ and maps/ subdirectories
// mapId    – WoW map ID (0=Eastern Kingdoms, 1=Kalimdor, 530=Outland, …)
// x1 y1   – start position in WoW world coordinates
// x2 y2   – end   position in WoW world coordinates
//
// Prints the path distance in world units, or -1 if no path was found.

int main(int argc, char* argv[])
{
    if (argc != 7)
    {
        fprintf(stderr, "Usage: %s <dataDir> <mapId> <x1> <y1> <x2> <y2>\n", argv[0]);
        return 1;
    }

    const char* dataDir = argv[1];
    uint32_t    mapId   = static_cast<uint32_t>(atoi(argv[2]));
    float       x1      = static_cast<float>(atof(argv[3]));
    float       y1      = static_cast<float>(atof(argv[4]));
    float       x2      = static_cast<float>(atof(argv[5]));
    float       y2      = static_cast<float>(atof(argv[6]));

    PathFinder pf(dataDir, mapId);
    if (!pf.isValid())
    {
        fprintf(stderr, "Failed to load navmesh for map %u from '%s'\n", mapId, dataDir);
        return 1;
    }

    float dist = pf.computeDistance(x1, y1, x2, y2);
    printf("%.4f\n", dist);

    return 0;
}
