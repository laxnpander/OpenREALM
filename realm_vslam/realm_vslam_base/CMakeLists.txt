cmake_minimum_required(VERSION 2.8.3)
project(realm_vslam_base)

SET(VSLAM_TYPE "ORB_SLAM2")

IF(VSLAM_TYPE STREQUAL "ORB_SLAM2")
	add_definitions(-DUSE_ORB_SLAM2)
    	SET(VSLAM_LIB orb_slam_2)
	SET(VSLAM_SRC "src/realm_vslam_lib/orb_slam.cpp")
#ELSEIF(VSLAM_TYPE STREQUAL "SVO")
#	add_definitions(-DUSE_SVO)
#	SET(VSLAM_LIB svo)
#	SET(VSLAM_SRC "src/realm_vslam_lib/svo.cpp")
#ELSEIF(VSLAM_TYPE STREQUAL "SVO2")
#	add_definitions(-DUSE_SVO2)
#	SET(VSLAM_LIB svo)
#	SET(VSLAM_SRC "src/realm_vslam_lib/svo2.cpp")
#	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
#	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3 -mno-avx")
#	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops -ffast-math -fno-finite-math-only")
#ELSEIF(VSLAM_TYPE STREQUAL "DSO")
#	add_definitions(-DUSE_DSO)
#	SET(VSLAM_LIB dso)
#	SET(VSLAM_SRC "src/realm_vslam_lib/dso.cpp")
ENDIF()

find_package(catkin REQUIRED COMPONENTS
		realm_core
		realm_io
		${VSLAM_LIB}
		)

# GENERAL REQUIRED
find_package(OpenCV 3.2)
find_package(cmake_modules REQUIRED)

#fix to find eigen3
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME} ${VSLAM_LIB}
		CATKIN_DEPENDS realm_core realm_io ${VSLAM_LIB}
		DEPENDS OpenCV cmake_modules Eigen3
)

###########
## Build ##
###########

## realm library
add_library(${PROJECT_NAME}
        src/realm_vslam_lib/visual_slam_factory.cpp
        src/realm_vslam_lib/visual_slam_settings_factory.cpp
	src/realm_vslam_lib/geometric_referencer.cpp
	${VSLAM_SRC}
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${cmake_modules_LIBRARIES}
        )
target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${cmake_modules_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)
add_definitions(-std=c++11)

#####################
## Install Library ##
#####################
install(
		DIRECTORY include/realm_vslam
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)