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

set(LIBRARY_NAME realm_vslam_base)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


################################################################################
# Dependencies
################################################################################


SET(VSLAM_IMPL_TYPE "ORB_SLAM2")

# Fix to avoid OpenCV package confusion with ROS melodic
find_package(OpenCV 3.2 EXACT QUIET)
if (NOT OpenCV_FOUND)
	find_package(OpenCV 3 REQUIRED)
endif()

#fix to find eigen3
find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
	find_package(PkgConfig REQUIRED)
	pkg_check_modules(Eigen3 REQUIRED eigen3)
	set(Eigen3_INCLUDE_DIR ${Eigen3_INCLUDE_DIRS})
endif()


################################################################################
# Sources
################################################################################


set(root ${CMAKE_CURRENT_SOURCE_DIR})

if(VSLAM_IMPL_TYPE STREQUAL "ORB_SLAM2")
	add_definitions(-DUSE_ORB_SLAM2)
	set(VSLAM_IMPL_LIB orb_slam_2)
	set(VSLAM_IMPL_LIB_HEADERS ${orb_slam_2_INCLUDE_DIR})
	set(VSLAM_IF_HEADERS ${root}/include/realm_vslam_base/orb_slam.h)
	set(VSLAM_IF_SOURCES ${root}/src/orb_slam.cpp)
	message(STATUS "Selected ORB SLAM 2 for visual pose estimation")

	#ELSEIF(VSLAM_TYPE STREQUAL "SVO")
	#	add_definitions(-DUSE_SVO)
	#	SET(VSLAM_LIB svo)
	#	SET(VSLAM_SRC "src/svo.cpp")
	#ELSEIF(VSLAM_TYPE STREQUAL "SVO2")
	#	add_definitions(-DUSE_SVO2)
	#	SET(VSLAM_LIB svo)
	#	SET(VSLAM_SRC "src/svo2.cpp")
	#	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
	#	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3 -mno-avx")
	#	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	#	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops -ffast-math -fno-finite-math-only")
	#ELSEIF(VSLAM_TYPE STREQUAL "DSO")
	#	add_definitions(-DUSE_DSO)
	#	SET(VSLAM_LIB dso)
	#	SET(VSLAM_SRC "src/dso.cpp")
endif()

set(HEADER_FILES
		${root}/include/realm_vslam_base/dummy_referencer.h
		${root}/include/realm_vslam_base/geometric_referencer.h
		${root}/include/realm_vslam_base/geospatial_referencer_IF.h
		${root}/include/realm_vslam_base/orb_slam.h
		${root}/include/realm_vslam_base/visual_slam_factory.h
		${root}/include/realm_vslam_base/visual_slam_IF.h
		${root}/include/realm_vslam_base/visual_slam_settings.h
		${root}/include/realm_vslam_base/visual_slam_settings_factory.h
		${VSLAM_IF_HEADERS}
)

set(SOURCE_FILES
		${root}/src/visual_slam_factory.cpp
		${root}/src/visual_slam_settings_factory.cpp
		${root}/src/dummy_referencer.cpp
		${root}/src/geometric_referencer.cpp
		${VSLAM_IF_SOURCES}
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
set(realm_vslam_base_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)


################################################################################
# Build
################################################################################


## realm library
include_directories(
		${CMAKE_CURRENT_BINARY_DIR}
		${CMAKE_BINARY_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/include
		${OpenCV_INCLUDE_DIRS}

		${Eigen3_INCLUDE_DIR}
		${VSLAM_IMPL_LIB_HEADERS}
		${realm_core_INCLUDE_DIR}
		${realm_io_INCLUDE_DIR}
)

add_library(${LIBRARY_NAME} SHARED
		${SOURCE_FILES}
		${HEADER_FILES}
		${FORM_HEADERS}
		${HEADERS_MOC}
		${RESOURCES_RCC}
)

target_link_libraries(${LIBRARY_NAME}
        ${OpenCV_LIBRARIES}

		${VSLAM_IMPL_LIB}
		realm_core
		realm_io
)

add_dependencies(${LIBRARY_NAME}
		realm_core
		realm_io
		${VSLAM_IMPL_LIB}
)

target_include_directories(${LIBRARY_NAME} PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # for headers when building
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}> # for client in install mode
)

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
		DESTINATION include/${PROJECT_NAME}
		FILES_MATCHING
		PATTERN "*.h"
		PATTERN "*.hpp"
)


################################################################################
# Test
################################################################################

