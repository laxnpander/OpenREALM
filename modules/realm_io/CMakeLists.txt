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

set(LIBRARY_NAME realm_io)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


################################################################################
# Dependencies
################################################################################


# Fix to avoid OpenCV package confusion with ROS melodic
find_package(OpenCV 3.2 EXACT QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 3 REQUIRED)
endif()

find_package(Exiv2 REQUIRED)
find_package(GDAL REQUIRED)
find_package(PCL 1.7 REQUIRED)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
find_package(Boost REQUIRED COMPONENTS
        filesystem
)


################################################################################
# Sources
################################################################################


set(root ${CMAKE_CURRENT_SOURCE_DIR})

set(HEADER_FILES
        ${root}/include/realm_io/cv_export.h
        ${root}/include/realm_io/exif_export.h
        ${root}/include/realm_io/exif_import.h
        ${root}/include/realm_io/gis_export.h
        ${root}/include/realm_io/pcl_export.h
        ${root}/include/realm_io/realm_export.h
        ${root}/include/realm_io/realm_import.h
        ${root}/include/realm_io/utilities.h
)

set(SOURCE_FILES
        ${root}/src/cv_export.cpp
        ${root}/src/exif_export.cpp
        ${root}/src/exif_import.cpp
        ${root}/src/gis_export.cpp
        ${root}/src/pcl_export.cpp
        ${root}/src/realm_export.cpp
        ${root}/src/realm_import.cpp
        ${root}/src/utilities.cpp
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
set(realm_io_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)

################################################################################
# Build
################################################################################

include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${PCL_LIBRARY_DIRS}
        ${GDAL_INCLUDE_DIR}
        ${Exiv2_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${cmake_modules_INCLUDE_DIR}
        ${realm_core_INCLUDE_DIR}
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
        ${PCL_COMMON_LIBRARIES}
        ${PCL_IO_LIBRARIES}
        ${GDAL_LIBRARY}
        ${Exiv2_LIBRARIES}
        ${Boost_LIBRARIES}

        realm_core
        -lexiv2
)

add_dependencies(${LIBRARY_NAME}
         realm_core
)

add_definitions(
        ${PCL_DEFINITIONS}
)

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################