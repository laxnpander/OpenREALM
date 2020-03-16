cmake_minimum_required(VERSION 2.8.3)
project(orb_slam_2)

# GENERAL REQUIRED
find_package(catkin REQUIRED)
find_package(OpenCV 3.2)
find_package(cmake_modules REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Pangolin REQUIRED)

#fix to find eigen3
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME} g2o DBoW2
)

###########
## Build ##
###########

set(THIRDPARTY_LIBRARIES pangolin)

## g2o Library shared
add_library(g2o SHARED
        #types
        src/g2o_lib/types/types_sba.cpp
        src/g2o_lib/types/types_six_dof_expmap.cpp
        src/g2o_lib/types/types_seven_dof_expmap.cpp
        #core
        src/g2o_lib/core/hyper_graph_action.cpp
        src/g2o_lib/core/hyper_graph.cpp
        src/g2o_lib/core/marginal_covariance_cholesky.cpp
        src/g2o_lib/core/matrix_structure.cpp
        src/g2o_lib/core/batch_stats.cpp
        src/g2o_lib/core/parameter.cpp
        src/g2o_lib/core/cache.cpp
        src/g2o_lib/core/optimizable_graph.cpp
        src/g2o_lib/core/solver.cpp
        src/g2o_lib/core/optimization_algorithm_factory.cpp
        src/g2o_lib/core/estimate_propagator.cpp
        src/g2o_lib/core/factory.cpp
        src/g2o_lib/core/sparse_optimizer.cpp
        src/g2o_lib/core/hyper_dijkstra.cpp
        src/g2o_lib/core/parameter_container.cpp
        src/g2o_lib/core/optimization_algorithm.cpp
        src/g2o_lib/core/optimization_algorithm_with_hessian.cpp
        src/g2o_lib/core/optimization_algorithm_levenberg.cpp
        src/g2o_lib/core/jacobian_workspace.cpp
        src/g2o_lib/core/robust_kernel.cpp
        src/g2o_lib/core/robust_kernel_factory.cpp
        src/g2o_lib/core/robust_kernel_impl.cpp
        #stuff
        src/g2o_lib/stuff/timeutil.cpp
        src/g2o_lib/stuff/os_specific.c
        src/g2o_lib/stuff/string_tools.cpp
        src/g2o_lib/stuff/property.cpp
        )
target_include_directories(g2o PUBLIC
        "include/g2o/g2o/core"
        "include/g2o/g2o/types"
        "include/g2o/g2o/stuff"
        ${EIGEN3_INCLUDE_DIRS}
        )
set_target_properties(g2o PROPERTIES
        CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native "
        CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native"
        )

## DBoW2 Library shared
add_library(DBoW2 SHARED
        src/DBoW2_lib/DBoW2/BowVector.cpp
        src/DBoW2_lib/DBoW2/FORB.cpp
        src/DBoW2_lib/DBoW2/FeatureVector.cpp
        src/DBoW2_lib/DBoW2/ScoringObject.cpp
        src/DBoW2_lib/DUtils/Random.cpp
        src/DBoW2_lib/DUtils/Timestamp.cpp
        )
target_include_directories(DBoW2 PUBLIC
        include
        include/DBoW2/DBoW2
        include/DBoW2/DUtils
        ${EIGEN3_INCLUDE_DIRS}
        )
target_link_libraries(DBoW2 ${OpenCV_LIBS})
set_target_properties(DBoW2 PROPERTIES
        CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native "
        CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native"
        )

## ORB_SLAM2 Library
add_library(${PROJECT_NAME}
        src/ORB_SLAM2_lib/Converter.cc
        src/ORB_SLAM2_lib/Frame.cc
        src/ORB_SLAM2_lib/FrameDrawer.cc
        src/ORB_SLAM2_lib/Initializer.cc
        src/ORB_SLAM2_lib/KeyFrame.cc
        src/ORB_SLAM2_lib/KeyFrameDatabase.cc
        src/ORB_SLAM2_lib/LocalMapping.cc
        src/ORB_SLAM2_lib/LoopClosing.cc
        src/ORB_SLAM2_lib/Map.cc
        src/ORB_SLAM2_lib/MapDrawer.cc
        src/ORB_SLAM2_lib/MapPoint.cc
        src/ORB_SLAM2_lib/Optimizer.cc
        src/ORB_SLAM2_lib/ORBextractor.cc
        src/ORB_SLAM2_lib/ORBmatcher.cc
        src/ORB_SLAM2_lib/PnPsolver.cc
        src/ORB_SLAM2_lib/Sim3Solver.cc
        src/ORB_SLAM2_lib/System.cc
        src/ORB_SLAM2_lib/Tracking.cc
        src/ORB_SLAM2_lib/Viewer.cc
        )
target_include_directories(${PROJECT_NAME} PUBLIC
        include
        include/ORB_SLAM2/
        ${EIGEN3_INCLUDE_DIRS}
        ${g2o_INCLUDE_DIRS}
        ${DBoW2_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${OpenGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${cmake_modules_INCLUDE_DIRS}
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
add_definitions(-std=c++11)
