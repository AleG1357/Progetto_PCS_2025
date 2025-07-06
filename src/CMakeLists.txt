list(APPEND src_headers ${CMAKE_CURRENT_SOURCE_DIR}/Polyhedron.hpp)
list(APPEND src_headers ${CMAKE_CURRENT_SOURCE_DIR}/Utils.hpp)
list(APPEND src_headers ${CMAKE_CURRENT_SOURCE_DIR}/Topology.hpp)
list(APPEND src_headers ${CMAKE_CURRENT_SOURCE_DIR}/Triangulation.hpp)
list(APPEND src_headers ${CMAKE_CURRENT_SOURCE_DIR}/ShortPath.hpp)

list(APPEND src_sources ${CMAKE_CURRENT_SOURCE_DIR}/Utils.cpp)
list(APPEND src_sources ${CMAKE_CURRENT_SOURCE_DIR}/Triangulation.cpp)
list(APPEND src_sources ${CMAKE_CURRENT_SOURCE_DIR}/Topology.cpp)
list(APPEND src_sources ${CMAKE_CURRENT_SOURCE_DIR}/ShortPath.cpp)

list(APPEND src_includes ${CMAKE_CURRENT_SOURCE_DIR})

set(src_headers ${src_headers} PARENT_SCOPE)
set(src_sources ${src_sources} PARENT_SCOPE)
set(src_includes ${src_includes} PARENT_SCOPE)