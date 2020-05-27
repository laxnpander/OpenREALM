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

set(LIBRARY_NAME realm_stages)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


################################################################################
# Dependencies
################################################################################


# Fix to avoid OpenCV package confusion with ROS melodic
find_package(OpenCV 3.2 EXACT QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 3 REQUIRED)
endif()


################################################################################
# Sources
################################################################################


set(root ${CMAKE_CURRENT_SOURCE_DIR})

set(HEADER_FILES
        include/realm_stages/conversions.h
        include/realm_stages/densification.h
        include/realm_stages/mosaicing.h
        include/realm_stages/ortho_rectification.h
        include/realm_stages/pose_estimation.h
        include/realm_stages/stage_base.h
        include/realm_stages/stage_settings.h
        include/realm_stages/stage_settings_factory.h
        include/realm_stages/surface_generation.h
)

set(SOURCE_FILES
        src/conversions.cpp
        src/densification.cpp
        src/mosaicing.cpp
        src/ortho_rectification.cpp
        src/pose_estimation.cpp
        src/stage_base.cpp
        src/stage_settings_factory.cpp
        src/surface_generation.cpp
)

# Organize the source and header files into groups
source_group("Headers" FILES ${HEADER_FILES})
source_group("Source" FILES ${SOURCE_FILES})
source_group("Forms" FILES ${FORM_FILES})
source_group("Resources" FILES ${RESOURCE_FILES})

if(CMAKE_VERSION VERSION_GREATER 3.8)
    source_group(TREE ${root} FILES ${HEADER_FILES} ${SOURCE_FILES})
endif()

# Define the folder containing the header files for this library
set(realm_stages_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)


################################################################################
# Build
################################################################################


include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}

)

add_library(${LIBRARY_NAME} SHARED
        ${SOURCE_FILES}
        ${HEADER_FILES}
        ${FORM_HEADERS}
        ${HEADERS_MOC}
        ${RESOURCES_RCC}
)

target_include_directories(${LIBRARY_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${LIBRARY_NAME}

        ${OpenCV_LIBRARIES}
        realm_core
        realm_io
        realm_vslam_base
        realm_densifier_base
        realm_ortho
)

add_dependencies(${LIBRARY_NAME}
        realm_core
        realm_io
        realm_vslam_base
        realm_densifier_base
        realm_ortho
)


################################################################################
# Install
################################################################################


################################################################################
# Test
################################################################################

