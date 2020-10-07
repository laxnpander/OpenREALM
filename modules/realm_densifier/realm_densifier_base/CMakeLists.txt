# This file is part of OpenREALM.
#
# Copyright (C) 2018 Alexander Kern <laxnpander at gmail dot com> (Braunschweig University of Technology)
# For more information see <https://github.com/laxnpander/OpenREALM>
#
# OpenREALM is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# OpenREALM is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with OpenREALM. If not, see <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.9)

set(LIBRARY_NAME realm_densifier_base)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


################################################################################
# Dependencies
################################################################################


if(CMAKE_CUDA_COMPILER)
    message(STATUS "CUDA found. Compiling densification with CUDA support...\n")
    set(DENSIFIER_WITH_CUDA True)
else()
    message(STATUS "CUDA not found. Skipping PSL package...\n")
    set(DENSIFIER_WITH_CUDA False)
endif()

# Fix to avoid OpenCV package confusion with ROS melodic
find_package(OpenCV 3.3.1 EXACT QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 3 REQUIRED)
endif()


################################################################################
# Sources
################################################################################


set(root ${CMAKE_CURRENT_SOURCE_DIR})

if (DENSIFIER_WITH_CUDA)
    add_definitions(-DUSE_CUDA)
    set(DENSIFIER_IMPL_LIBS psl)
    set(DENSIFIER_IMPL_LIBS_HEADER ${psl_INCLUDE_DIR})
    set(DENSIFIER_IMPL_HEADERS ${root}/include/realm_densifier_base/plane_sweep.h)
    set(DENSIFIER_IMPL_SOURCES ${root}/src/plane_sweep.cpp)
endif()

set(HEADER_FILES
        ${root}/include/realm_densifier_base/densifier_dummy.h
        ${root}/include/realm_densifier_base/densifier_factory.h
        ${root}/include/realm_densifier_base/densifier_IF.h
        ${root}/include/realm_densifier_base/densifier_settings.h
        ${root}/include/realm_densifier_base/densifier_settings_factory.h
        ${root}/include/realm_densifier_base/sparse_depth.h
        ${DENSIFIER_IMPL_HEADERS}
)

set(SOURCE_FILES
        ${root}/src/densifier_factory.cpp
        ${root}/src/densifier_settings_factory.cpp
        ${root}/src/densifier_dummy.cpp
        ${root}/src/sparse_depth.cpp
        ${DENSIFIER_IMPL_SOURCES}
        src/depth_consistency_filter.cpp)

# Organize the source and header files into groups
source_group("Headers" FILES ${HEADER_FILES})
source_group("Source" FILES ${SOURCE_FILES})
source_group("Forms" FILES ${FORM_FILES})
source_group("Resources" FILES ${RESOURCE_FILES})

if(CMAKE_VERSION VERSION_GREATER 3.8)
    source_group(TREE ${root} FILES ${HEADER_FILES} ${SOURCE_FILES})
endif()

# Define the folder containing the header files for this library
set(realm_densifier_base_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)


################################################################################
# Build
################################################################################


## realm densifier
include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${realm_core_INCLUDE_DIR}
        ${realm_io_INCLUDE_DIR}
        ${DENSIFIER_IMPL_LIBS_HEADER}
)

add_library(${LIBRARY_NAME} SHARED
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
            ${DENSIFIER_IMPL_LIBS}
        PRIVATE
            ${OpenCV_LIBRARIES}
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

