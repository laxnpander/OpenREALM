cmake_minimum_required(VERSION 2.8.3)
project(realm_ortho)

find_package(OpenCV 3.2)
find_package(CGAL REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        realm_core
        realm_io
)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS realm_core realm_io cmake_modules
        DEPENDS OpenCV CGAL Eigen3
)

###################
## Build Library ##
##################

add_library(${PROJECT_NAME} SHARED
        src/realm_ortho_lib/dsm.cpp
        src/realm_ortho_lib/delaunay_2d.cpp
        src/realm_ortho_lib/rectification.cpp
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${CGAL_LIBRARIES}
        ${cmake_modules_LIBRARIES}
        )
target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${CGAL_INCLUDE_DIRS}
        ${cmake_modules_INCLUDE_DIRS}
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
        DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)