cmake_minimum_required(VERSION 2.8.3)
project(realm_core)

find_package(catkin REQUIRED)
find_package(OpenCV 3.2)

# Solution to find Eigen3 with Saucy.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
    set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()
find_package(GDAL REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS cmake_modules
        DEPENDS OpenCV Eigen3 GDAL
)

###################
## Build Library ##
###################

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${GDAL_INCLUDE_DIR}
)
add_library(${PROJECT_NAME} SHARED
        src/realm_common/timer.cpp
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
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${GDAL_LIBRARY}
)
add_definitions(-std=c++11)

#####################
## Install Library ##
#####################

# Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include include/realm_common include/realm_cv include/realm_gis include/realm_maths include/realm_types
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)