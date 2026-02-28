cmake_minimum_required(VERSION 3.10)
project(pathfind CXX)

set(CMAKE_CXX_STANDARD 14)

# DT_POLYREF64 must match how the .mmtile files were built by MoveMapGen.
add_compile_definitions(DT_POLYREF64)

set(DETOUR_SOURCES
    Detour/DetourAlloc.cpp
    Detour/DetourAssert.cpp
    Detour/DetourCommon.cpp
    Detour/DetourNavMesh.cpp
    Detour/DetourNavMeshQuery.cpp
    Detour/DetourNode.cpp
)

add_executable(pathfind
    main.cpp
    PathFinder.cpp
    TerrainMap.cpp
    ${DETOUR_SOURCES}
)

target_include_directories(pathfind PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/Detour"
)
