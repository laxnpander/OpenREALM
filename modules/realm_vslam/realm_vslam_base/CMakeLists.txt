
cmake_minimum_required(VERSION 3.15)

set(LIBRARY_NAME realm_vslam_base)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

################################################################################
# Dependencies
################################################################################

# Fix to avoid OpenCV package confusion with ROS melodic
find_package(OpenCV 3.3.1 EXACT QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 3 QUIET)
endif()
if (NOT OpenCV_FOUND)
    find_package(OpenCV 4 REQUIRED)
    message(WARNING "OpenCV 4 Support is experimental, use at your own risk!")
endif()

################################################################################
# Sources
################################################################################

set(root ${CMAKE_CURRENT_SOURCE_DIR})

#find_package(ORB_SLAM2 REQUIRED)
#set(VSLAM_IMPL_LIB ORB_SLAM2)
#set(VSLAM_IMPL_LIB_HEADERS ${ORB_SLAM2_INCLUDE_DIR})
#set(VSLAM_IF_HEADERS ${root}/include/realm_vslam_base/orb_slam.h)
#set(VSLAM_IF_SOURCES ${root}/src/orb_slam.cpp)
#set(VSLAM_IF_COMPILE_FLAG "-DUSE_ORB_SLAM2=ON")
#message(STATUS "Selected ORB SLAM 2 for visual pose estimation")

#find_package(ORB_SLAM3 REQUIRED)
#set(VSLAM_IMPL_LIB ORB_SLAM3)
#set(VSLAM_IMPL_LIB_HEADERS ${ORB_SLAM3_INCLUDE_DIR})
#set(VSLAM_IF_HEADERS ${root}/include/realm_vslam_base/orb_slam.h)
#set(VSLAM_IF_SOURCES ${root}/src/orb_slam.cpp)
#set(VSLAM_IF_COMPILE_FLAG "-DUSE_ORB_SLAM3=ON")
#message(STATUS "Selected ORB SLAM 3 for visual pose estimation")

#find_package(openvslam REQUIRED)
#find_package(Eigen3 3.3 REQUIRED NO_MODULE)
#find_package(OpenMP REQUIRED)
#set(VSLAM_IMPL_LIB openvslam::openvslam)
#set(VSLAM_IMPL_LIB_HEADERS ${OpenVSLAM_INCLUDE_DIR})
#set(VSLAM_IF_HEADERS ${root}/include/realm_vslam_base/open_vslam.h)
#set(VSLAM_IF_SOURCES ${root}/src/open_vslam.cpp)
#set(VSLAM_IF_COMPILE_FLAG "-DUSE_OPEN_VSLAM")
#message(STATUS "Selected OpenVSLAM for visual pose estimation")

find_package(ov2slam REQUIRED)
find_package(PCL REQUIRED)
set(VSLAM_IMPL_LIB ov2slam)
set(VSLAM_IMPL_LIB_HEADERS ${ov2slam_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
set(VSLAM_IF_HEADERS ${root}/include/realm_vslam_base/ov2_slam.h)
set(VSLAM_IF_SOURCES ${root}/src/ov2_slam.cpp)
set(VSLAM_IF_COMPILE_FLAG "-DUSE_OV2SLAM=ON")
message(STATUS "Selected ov2slam for visual pose estimation")


set(HEADER_FILES
		${root}/include/realm_vslam_base/dummy_referencer.h
		${root}/include/realm_vslam_base/geometric_referencer.h
		${root}/include/realm_vslam_base/geospatial_referencer_IF.h
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
		${VSLAM_IMPL_LIB_HEADERS}
		${realm_core_INCLUDE_DIR}
		${realm_io_INCLUDE_DIR}
)

add_library(${LIBRARY_NAME}
		${SOURCE_FILES}
		${HEADER_FILES}
		${FORM_HEADERS}
		${HEADERS_MOC}
		${RESOURCES_RCC}
)

target_link_libraries(${LIBRARY_NAME}
		PUBLIC
			realm_core
			realm_io
			${VSLAM_IMPL_LIB}
		PRIVATE
			${OpenCV_LIBRARIES}
)

target_compile_definitions(${LIBRARY_NAME} PUBLIC ${VSLAM_IF_COMPILE_FLAG})

target_include_directories(${LIBRARY_NAME} PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # for headers when building
		$<INSTALL_INTERFACE:${OpenREALM_INC_INSTALL_DIR}> # for client in install mode
)

add_definitions(
		-Wno-deprecated-declarations
		${EIGEN3_DEFINITIONS}
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
		DESTINATION ${OpenREALM_INC_INSTALL_DIR}
		FILES_MATCHING
		PATTERN "*.h"
		PATTERN "*.hpp"
)


################################################################################
# Test
################################################################################

