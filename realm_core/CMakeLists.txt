cmake_minimum_required(VERSION 2.8.3)
project(realm_core)

find_package(catkin REQUIRED)
find_package(OpenCV 3.2)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
find_package(GDAL REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
)

###################
## Build Library ##
###################

add_library(${PROJECT_NAME} SHARED
        src/realm_cv_lib/mat_overwrite.cpp
        src/realm_cv_lib/analysis.cpp
        src/realm_cv_lib/stereo.cpp
        src/realm_cv_lib/inpaint.cpp
        src/realm_gis_lib/conversions.cpp
        src/realm_types_lib/camera.cpp
        src/realm_types_lib/frame.cpp
        src/realm_types_lib/settings_base.cpp
        src/realm_types_lib/camera_settings_factory.cpp
        src/realm_types_lib/cv_grid_map.cpp
        src/realm_types_lib/worker_thread_base.cpp
        src/realm_maths_lib/sim2_solver.cpp
        src/realm_maths_lib/se3_solver.cpp
        src/realm_maths_lib/plane_fitter.cpp
        )
target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${GDAL_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${GDAL_LIBRARY}
)
add_definitions(-std=c++11)
