
cmake_minimum_required(VERSION 3.15)

set(LIBRARY_NAME realm_ortho)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

################################################################################
# Dependencies
################################################################################

# Add custom Find cmake files
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Fix to avoid OpenCV package confusion with ROS melodic
find_package(OpenCV 3.3.1 EXACT QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 3 QUIET)
endif()
if (NOT OpenCV_FOUND)
    find_package(OpenCV 4 REQUIRED)
    message(WARNING "OpenCV 4 Support is experimental, use at your own risk!")
endif()

find_package(GDAL REQUIRED)

################################################################################
# Optional Packages (LGPL / Other License Restrictions)
################################################################################
option(WITH_CGAL "Enable CGAL support for Mesh generation" ON)
set(CGAL_ENABLED FALSE)
if(WITH_CGAL)
    find_package(CGAL REQUIRED)
    if(CGAL_FOUND)
        set(CGAL_ENABLED TRUE)
    else()
        message(SEND_ERROR "CGAL dependency not found!")
    endif()
endif()


################################################################################
# Sources
################################################################################


set(root ${CMAKE_CURRENT_SOURCE_DIR})

set(HEADER_FILES
        ${root}/include/realm_ortho/dsm.h
        ${root}/include/realm_ortho/gdal_warper.h
        ${root}/include/realm_ortho/map_tiler.h
        ${root}/include/realm_ortho/nanoflann.h
        ${root}/include/realm_ortho/nearest_neighbor.h
        ${root}/include/realm_ortho/rectification.h
        ${root}/include/realm_ortho/tile.h
        ${root}/include/realm_ortho/tile_cache.h
)

set(SOURCE_FILES
        ${root}/src/dsm.cpp
        ${root}/src/gdal_warper.cpp
        ${root}/src/map_tiler.cpp
        ${root}/src/rectification.cpp
        ${root}/src/tile.cpp
        ${root}/src/tile_cache.cpp
)

# delaunay relies on CGAL to work
if(CGAL_ENABLED)
    list(APPEND HEADER_FILES ${root}/include/realm_ortho/delaunay_2d.h)
    list(APPEND HEADER_FILES ${root}/src/delaunay_2d.cpp)
endif()

# Organize the source and header files into groups
source_group("Headers" FILES ${HEADER_FILES})
source_group("Source" FILES ${SOURCE_FILES})
source_group("Forms" FILES ${FORM_FILES})
source_group("Resources" FILES ${RESOURCE_FILES})

if(CMAKE_VERSION VERSION_GREATER 3.8)
    source_group(TREE ${root} FILES ${HEADER_FILES} ${SOURCE_FILES})
endif()

# Define the folder containing the header files for this library
set(realm_ortho_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)


################################################################################
# Build
################################################################################


include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}

        ${realm_core_INCLUDE_DIR}
        ${realm_io_INCLUDE_DIR}
)

add_library(${LIBRARY_NAME}
        ${SOURCE_FILES}
        ${HEADER_FILES}
        ${FORM_HEADERS}
        ${HEADERS_MOC}
        ${RESOURCES_RCC}
)

target_include_directories(${LIBRARY_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # for headers when building
        $<INSTALL_INTERFACE:${OpenREALM_INC_INSTALL_DIR}> # for client in install mode
        )

target_link_libraries(${LIBRARY_NAME}
        PUBLIC
            realm_core
            realm_io
        PRIVATE
            ${OpenCV_LIBRARIES}
)

if (CGAL_ENABLED)
    target_link_libraries(${LIBRARY_NAME}
            PUBLIC
            CGAL
            )
    include_directories(${CGAL_INCLUDE_DIRS})
endif()

add_definitions(
        -Wno-deprecated-declarations
)


################################################################################
# Install
################################################################################


set_target_properties(${LIBRARY_NAME} PROPERTIES OUTPUT_NAME "open_${LIBRARY_NAME}-${OpenREALM_VERSION}")

install(TARGETS ${LIBRARY_NAME} EXPORT OpenREALMTargets
        RUNTIME       DESTINATION ${OpenREALM_RUNTIME_INSTALL_DIR}
        LIBRARY       DESTINATION ${OpenREALM_LIBRARY_INSTALL_DIR}
        ARCHIVE       DESTINATION ${OpenREALM_ARCHIVE_INSTALL_DIR}
        FRAMEWORK     DESTINATION ${OpenREALM_FRAMEWORK_INSTALL_DIR})

# Headers
install(
        DIRECTORY include/${LIBRARY_NAME}
        DESTINATION ${OpenREALM_INC_INSTALL_DIR}
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp"
)


################################################################################
# Test
################################################################################

