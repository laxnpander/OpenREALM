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

set(LIBRARY_NAME orb_slam_2)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


################################################################################
# Dependencies
################################################################################


# Fix to avoid OpenCV package confusion with ROS melodic
find_package(OpenCV 3.2 EXACT QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 3 REQUIRED)
endif()

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Pangolin REQUIRED)

#fix to find eigen3
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(EIGEN3 QUIET)
if(NOT EIGEN3_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
    set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()


################################################################################
# Sources
################################################################################


set(root ${CMAKE_CURRENT_SOURCE_DIR})

set(G2O_SOURCE_FILES
        #types
        ${root}/3rdparty/g2o/g2o_types/types_sba.cpp
        ${root}/3rdparty/g2o/g2o_types/types_six_dof_expmap.cpp
        ${root}/3rdparty/g2o/g2o_types/types_seven_dof_expmap.cpp
        #g2o_core
        ${root}/3rdparty/g2o/g2o_core/hyper_graph_action.cpp
        ${root}/3rdparty/g2o/g2o_core/hyper_graph.cpp
        ${root}/3rdparty/g2o/g2o_core/marginal_covariance_cholesky.cpp
        ${root}/3rdparty/g2o/g2o_core/matrix_structure.cpp
        ${root}/3rdparty/g2o/g2o_core/batch_stats.cpp
        ${root}/3rdparty/g2o/g2o_core/parameter.cpp
        ${root}/3rdparty/g2o/g2o_core/cache.cpp
        ${root}/3rdparty/g2o/g2o_core/optimizable_graph.cpp
        ${root}/3rdparty/g2o/g2o_core/solver.cpp
        ${root}/3rdparty/g2o/g2o_core/optimization_algorithm_factory.cpp
        ${root}/3rdparty/g2o/g2o_core/estimate_propagator.cpp
        ${root}/3rdparty/g2o/g2o_core/factory.cpp
        ${root}/3rdparty/g2o/g2o_core/sparse_optimizer.cpp
        ${root}/3rdparty/g2o/g2o_core/hyper_dijkstra.cpp
        ${root}/3rdparty/g2o/g2o_core/parameter_container.cpp
        ${root}/3rdparty/g2o/g2o_core/optimization_algorithm.cpp
        ${root}/3rdparty/g2o/g2o_core/optimization_algorithm_with_hessian.cpp
        ${root}/3rdparty/g2o/g2o_core/optimization_algorithm_levenberg.cpp
        ${root}/3rdparty/g2o/g2o_core/jacobian_workspace.cpp
        ${root}/3rdparty/g2o/g2o_core/robust_kernel.cpp
        ${root}/3rdparty/g2o/g2o_core/robust_kernel_factory.cpp
        ${root}/3rdparty/g2o/g2o_core/robust_kernel_impl.cpp
        #g2o_stuff
        ${root}/3rdparty/g2o/g2o_stuff/timeutil.cpp
        ${root}/3rdparty/g2o/g2o_stuff/os_specific.c
        ${root}/3rdparty/g2o/g2o_stuff/string_tools.cpp
        ${root}/3rdparty/g2o/g2o_stuff/property.cpp
)

set(DBOW_SOURCE_FILES
        ${root}/3rdparty/DBoW2/DBoW2/BowVector.cpp
        ${root}/3rdparty/DBoW2/DBoW2/FORB.cpp
        ${root}/3rdparty/DBoW2/DBoW2/FeatureVector.cpp
        ${root}/3rdparty/DBoW2/DBoW2/ScoringObject.cpp
        ${root}/3rdparty/DBoW2/DUtils/Random.cpp
        ${root}/3rdparty/DBoW2/DUtils/Timestamp.cpp
)

set(HEADER_FILES
        ${root}/include/orb_slam_2/Converter.h
        ${root}/include/orb_slam_2/Frame.h
        ${root}/include/orb_slam_2/FrameDrawer.h
        ${root}/include/orb_slam_2/Initializer.h
        ${root}/include/orb_slam_2/KeyFrame.h
        ${root}/include/orb_slam_2/KeyFrameDatabase.h
        ${root}/include/orb_slam_2/LocalMapping.h
        ${root}/include/orb_slam_2/LoopClosing.h
        ${root}/include/orb_slam_2/Map.h
        ${root}/include/orb_slam_2/MapDrawer.h
        ${root}/include/orb_slam_2/MapPoint.h
        ${root}/include/orb_slam_2/Optimizer.h
        ${root}/include/orb_slam_2/ORBextractor.h
        ${root}/include/orb_slam_2/ORBmatcher.h
        ${root}/include/orb_slam_2/ORBVocabulary.h
        ${root}/include/orb_slam_2/PnPsolver.h
        ${root}/include/orb_slam_2/Settings.h
        ${root}/include/orb_slam_2/Sim3Solver.h
        ${root}/include/orb_slam_2/System.h
        ${root}/include/orb_slam_2/Tracking.h
        ${root}/include/orb_slam_2/Viewer.h
)

set(SOURCE_FILES
        ${root}/src/Converter.cc
        ${root}/src/Frame.cc
        ${root}/src/FrameDrawer.cc
        ${root}/src/Initializer.cc
        ${root}/src/KeyFrame.cc
        ${root}/src/KeyFrameDatabase.cc
        ${root}/src/LocalMapping.cc
        ${root}/src/LoopClosing.cc
        ${root}/src/Map.cc
        ${root}/src/MapDrawer.cc
        ${root}/src/MapPoint.cc
        ${root}/src/Optimizer.cc
        ${root}/src/ORBextractor.cc
        ${root}/src/ORBmatcher.cc
        ${root}/src/PnPsolver.cc
        ${root}/src/Sim3Solver.cc
        ${root}/src/System.cc
        ${root}/src/Tracking.cc
        ${root}/src/Viewer.cc
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
set(orb_slam_2_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)


################################################################################
# Build
################################################################################


include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/orb_slam_2
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/DBoW2
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/DBoW2/DBoW2
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/DBoW2/DUtils
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/g2o
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/g2o/g2o_core
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/g2o/g2o_solvers
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/g2o/g2o_types
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/g2o/g2o_stuff
        ${EIGEN3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${OpenGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}

)

## g2o Library shared
add_library(g2o
        ${G2O_SOURCE_FILES}
)

set_target_properties(g2o PROPERTIES
        CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native " -Wno-deprecated-declarations
        CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native" -Wno-deprecated-declarations
        )

## DBoW2 Library
add_library(DBoW2
        ${DBOW_SOURCE_FILES}
)

target_link_libraries(DBoW2 ${OpenCV_LIBS})
set_target_properties(DBoW2 PROPERTIES
        CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native " -Wno-deprecated-declarations
        CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native" -Wno-deprecated-declarations
)

## ORB_SLAM2 Library
add_library(${LIBRARY_NAME} SHARED
        ${SOURCE_FILES}
        ${HEADER_FILES}
        ${FORM_HEADERS}
        ${HEADERS_MOC}
        ${RESOURCES_RCC}
)

target_include_directories(${LIBRARY_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # for headers when building
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty> # for headers when building
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/DBoW2> # for headers when building
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/g2o> # for headers when building
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}> # for client in install mode
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/3rdparty> # for client in install mode
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/3rdparty/DBoW2> # for client in install mode
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/3rdparty/g2o> # for client in install mode
)

target_link_libraries(${LIBRARY_NAME}
        ${g2o_LIBRARIES}
        ${DBoW2_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${OpenGL_LIBRARIES}
        ${GLUT_LIBRARIES}
        ${Pangolin_LIBRARIES}

)

add_dependencies(${LIBRARY_NAME}
        g2o
        DBoW2
)

add_definitions(
        -Wno-deprecated-declarations
)

################################################################################
# Install
################################################################################


set_target_properties(${LIBRARY_NAME} PROPERTIES OUTPUT_NAME "open_realm_${LIBRARY_NAME}-${OpenREALM_VERSION}")

# Library
install(TARGETS ${LIBRARY_NAME} EXPORT OpenREALMTargets
        RUNTIME       DESTINATION ${OpenREALM_RUNTIME_INSTALL_DIR}
        LIBRARY       DESTINATION ${OpenREALM_LIBRARY_INSTALL_DIR}
        ARCHIVE       DESTINATION ${OpenREALM_ARCHIVE_INSTALL_DIR}
        FRAMEWORK     DESTINATION ${OpenREALM_FRAMEWORK_INSTALL_DIR})

# Headers
install(
        DIRECTORY include/${LIBRARY_NAME} 3rdparty/g2o 3rdparty/DBoW2
        DESTINATION include/${PROJECT_NAME}/3rdparty
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp"
        PATTERN "*.in"
)


################################################################################
# Test
################################################################################

