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

set(LIBRARY_NAME realm_core)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


################################################################################
# Dependencies
################################################################################


# Fix to avoid OpenCV package confusion with ROS melodic
if(IOS)
    set(OpenCV_DIR /Users/zach/Source/quickstitch/apple_root/)
    set(OpenCV_INCLUDE_DIRS /Users/zach/Source/quickstitch/apple_root/include)
    message(WARNING "Using OpenCV Include: ${OpenCV_INCLUDE_DIRS}")
else()
    find_package(OpenCV 3.3.1 EXACT QUIET)
    if (NOT OpenCV_FOUND)
        find_package(OpenCV 3 QUIET)
    endif()
    if (NOT OpenCV_FOUND)
        find_package(OpenCV 4 REQUIRED)
        message(WARNING "OpenCV 4 Support is experimental, use at your own risk!")
    endif()
endif()

# Solution to find Eigen3 with Saucy.
if(IOS)
    set(EIGEN3_INCLUDE_DIRS 3rdparty/eigen)
else()
    find_package(EIGEN3 QUIET)
    if(NOT EIGEN3_FOUND)
        find_package(PkgConfig REQUIRED)
        pkg_check_modules(EIGEN3 REQUIRED eigen3)
        set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
    endif()
    find_package(GDAL REQUIRED)
endif()

################################################################################
# Sources
################################################################################


set(root ${CMAKE_CURRENT_SOURCE_DIR})

set(HEADER_FILES
        ${root}/include/realm_core/analysis.h
        ${root}/include/realm_core/camera.h
        ${root}/include/realm_core/camera_settings.h
        ${root}/include/realm_core/camera_settings_factory.h
        ${root}/include/realm_core/conversions.h
        ${root}/include/realm_core/cv_grid_map.h
        ${root}/include/realm_core/depthmap.h
        ${root}/include/realm_core/enums.h
        ${root}/include/realm_core/frame.h
        ${root}/include/realm_core/inpaint.h
        ${root}/include/realm_core/loguru.h
        ${root}/include/realm_core/plane_fitter.h
        ${root}/include/realm_core/settings_base.h
        ${root}/include/realm_core/stereo.h
        ${root}/include/realm_core/structs.h
        ${root}/include/realm_core/timer.h
        ${root}/include/realm_core/tree_node.h
        ${root}/include/realm_core/utm32.h
        ${root}/include/realm_core/wgs84.h
        ${root}/include/realm_core/worker_thread_base.h
)

set(SOURCE_FILES
        ${root}/src/timer.cpp
        ${root}/src/analysis.cpp
        ${root}/src/stereo.cpp
        ${root}/src/inpaint.cpp
        ${root}/src/conversions.cpp
        ${root}/src/camera.cpp
        ${root}/src/depthmap.cpp
        ${root}/src/frame.cpp
        ${root}/src/settings_base.cpp
        ${root}/src/camera_settings_factory.cpp
        ${root}/src/cv_grid_map.cpp
        ${root}/src/worker_thread_base.cpp
        ${root}/src/plane_fitter.cpp include/realm_core/depthmap.h src/depthmap.cpp)

# Organize the source and header files into groups
source_group("Headers" FILES ${HEADER_FILES})
source_group("Source" FILES ${SOURCE_FILES})
source_group("Forms" FILES ${FORM_FILES})
source_group("Resources" FILES ${RESOURCE_FILES})

if(CMAKE_VERSION VERSION_GREATER 3.8)
    source_group(TREE ${root} FILES ${HEADER_FILES} ${SOURCE_FILES})
endif()

# Define the folder containing the header files for this library
set(realm_core_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)


################################################################################
# Build
################################################################################


include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${GDAL_INCLUDE_DIR}
)

add_library(${LIBRARY_NAME}
        ${SOURCE_FILES}
        ${HEADER_FILES}
        ${FORM_HEADERS}
        ${HEADERS_MOC}
        ${RESOURCES_RCC}
)

target_include_directories(${LIBRARY_NAME}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # for headers when building
            $<INSTALL_INTERFACE:${OpenREALM_INC_INSTALL_DIR}>
)

target_link_libraries(${LIBRARY_NAME}
        PUBLIC
            ${OpenCV_LIBRARIES}
        PRIVATE
            ${GDAL_LIBRARY}
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
        FRAMEWORK     DESTINATION ${OpenREALM_FRAMEWORK_INSTALL_DIR}
)

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


if(TESTS_ENABLED)
    include(GoogleTest)

    add_executable(run_realm_core_tests
            test/test_realm_core.cpp
            test/test_helper.cpp
            test/conversion_test.cpp
            test/cvgridmap_test.cpp
            test/depthmap_test.cpp
            test/frame_test.cpp
            test/pinhole_test.cpp
            test/plane_fitter_test.cpp
            test/settings_test.cpp
            test/stereo_test.cpp
            test/worker_thread_test.cpp
    )

    # Standard linking to gtest stuff.
    target_link_libraries(run_realm_core_tests gtest_main)

    # Extra linking for the project.
    target_link_libraries(run_realm_core_tests ${LIBRARY_NAME})

    add_custom_command(TARGET run_realm_core_tests PRE_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/test/data $<TARGET_FILE_DIR:run_realm_core_tests>
            )

    # This is so you can do 'make test' to see all your tests run, instead of
    # manually running the executable run_unit_tests to see those specific tests.
    add_test(
            NAME
                realm_core_unit_tests
            COMMAND
                ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}/run_realm_core_tests
    )
endif()