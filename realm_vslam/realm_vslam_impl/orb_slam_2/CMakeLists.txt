cmake_minimum_required(VERSION 2.8.3)
project(orb_slam_2)

# GENERAL REQUIRED
find_package(catkin REQUIRED)
find_package(OpenCV 3)
find_package(cmake_modules REQUIRED)
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

####################
## Catkin Package ##
####################

catkin_package(
        INCLUDE_DIRS
            include
        LIBRARIES
            ${PROJECT_NAME}
            g2o
            DBoW2
        CATKIN_DEPENDS
            cmake_modules
        DEPENDS
            OpenCV
            Pangolin
            EIGEN3
)

###########
## Build ##
###########

set(THIRDPARTY_LIBRARIES pangolin)

include_directories(
        include
        include/DBoW2
        include/DBoW2/DBoW2
        include/DBoW2/DUtils
        include/orb_slam_2/
        ${EIGEN3_INCLUDE_DIRS}
        include/g2o
        include/g2o/g2o
        include/g2o/g2o/g2o_core
        include/g2o/g2o/g2o_types
        include/g2o/g2o/g2o_stuff
        ${OpenCV_INCLUDE_DIRS}
        ${OpenGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${cmake_modules_INCLUDE_DIRS}
)

## g2o Library shared
add_library(g2o
        #types
        src/g2o_lib/g2o_types/types_sba.cpp
        src/g2o_lib/g2o_types/types_six_dof_expmap.cpp
        src/g2o_lib/g2o_types/types_seven_dof_expmap.cpp
        #g2o_core
        src/g2o_lib/g2o_core/hyper_graph_action.cpp
        src/g2o_lib/g2o_core/hyper_graph.cpp
        src/g2o_lib/g2o_core/marginal_covariance_cholesky.cpp
        src/g2o_lib/g2o_core/matrix_structure.cpp
        src/g2o_lib/g2o_core/batch_stats.cpp
        src/g2o_lib/g2o_core/parameter.cpp
        src/g2o_lib/g2o_core/cache.cpp
        src/g2o_lib/g2o_core/optimizable_graph.cpp
        src/g2o_lib/g2o_core/solver.cpp
        src/g2o_lib/g2o_core/optimization_algorithm_factory.cpp
        src/g2o_lib/g2o_core/estimate_propagator.cpp
        src/g2o_lib/g2o_core/factory.cpp
        src/g2o_lib/g2o_core/sparse_optimizer.cpp
        src/g2o_lib/g2o_core/hyper_dijkstra.cpp
        src/g2o_lib/g2o_core/parameter_container.cpp
        src/g2o_lib/g2o_core/optimization_algorithm.cpp
        src/g2o_lib/g2o_core/optimization_algorithm_with_hessian.cpp
        src/g2o_lib/g2o_core/optimization_algorithm_levenberg.cpp
        src/g2o_lib/g2o_core/jacobian_workspace.cpp
        src/g2o_lib/g2o_core/robust_kernel.cpp
        src/g2o_lib/g2o_core/robust_kernel_factory.cpp
        src/g2o_lib/g2o_core/robust_kernel_impl.cpp
        #g2o_stuff
        src/g2o_lib/g2o_stuff/timeutil.cpp
        src/g2o_lib/g2o_stuff/os_specific.c
        src/g2o_lib/g2o_stuff/string_tools.cpp
        src/g2o_lib/g2o_stuff/property.cpp
        )
set_target_properties(g2o PROPERTIES
        CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native " -Wno-deprecated-declarations
        CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native" -Wno-deprecated-declarations
        )

## DBoW2 Library
add_library(DBoW2
        src/DBoW2_lib/DBoW2/BowVector.cpp
        src/DBoW2_lib/DBoW2/FORB.cpp
        src/DBoW2_lib/DBoW2/FeatureVector.cpp
        src/DBoW2_lib/DBoW2/ScoringObject.cpp
        src/DBoW2_lib/DUtils/Random.cpp
        src/DBoW2_lib/DUtils/Timestamp.cpp
        )
target_link_libraries(DBoW2 ${OpenCV_LIBS})
set_target_properties(DBoW2 PROPERTIES
        CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native " -Wno-deprecated-declarations
        CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native" -Wno-deprecated-declarations
        )

## ORB_SLAM2 Library
add_library(${PROJECT_NAME} SHARED
        src/orb_slam_2_lib/Converter.cc
        src/orb_slam_2_lib/Frame.cc
        src/orb_slam_2_lib/FrameDrawer.cc
        src/orb_slam_2_lib/Initializer.cc
        src/orb_slam_2_lib/KeyFrame.cc
        src/orb_slam_2_lib/KeyFrameDatabase.cc
        src/orb_slam_2_lib/LocalMapping.cc
        src/orb_slam_2_lib/LoopClosing.cc
        src/orb_slam_2_lib/Map.cc
        src/orb_slam_2_lib/MapDrawer.cc
        src/orb_slam_2_lib/MapPoint.cc
        src/orb_slam_2_lib/Optimizer.cc
        src/orb_slam_2_lib/ORBextractor.cc
        src/orb_slam_2_lib/ORBmatcher.cc
        src/orb_slam_2_lib/PnPsolver.cc
        src/orb_slam_2_lib/Sim3Solver.cc
        src/orb_slam_2_lib/System.cc
        src/orb_slam_2_lib/Tracking.cc
        src/orb_slam_2_lib/Viewer.cc
        )
target_link_libraries(${PROJECT_NAME}
        ${DBoW2_LIBRARIES}
        ${g2o_LIBRARIES}
        ${THIRDPARTY_LIBRARIES}
        ${g2o_LIBRARIES}
        ${DBoW2_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${OpenGL_LIBRARIES}
        ${GLUT_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${cmake_modules_LIBRARIES}
        )
add_definitions(
        -std=c++11
        -Wno-deprecated-declarations
        )

#####################
## Install Library ##
#####################

# Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME} g2o DBoW2
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/ include/g2o include/DBoW2
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING
            PATTERN "*.h"
            PATTERN "*.hpp"
)