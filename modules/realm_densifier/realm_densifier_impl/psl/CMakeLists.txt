
cmake_minimum_required(VERSION 3.15)

set(LIBRARY_NAME psl)

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

find_package(Boost REQUIRED COMPONENTS system filesystem)

if(NOT CMAKE_CUDA_COMPILER)
    message(STATUS "CUDA not found. Skipping PSL package...\n")
    return()
endif()

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -O3 -use_fast_math -D_FORCE_INLINES)
set(CUDA_PROPAGATE_HOST_FLAGS OFF)
set(CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE OFF)

add_definitions(-DRMD_CORR_PATCH_SIDE=5)
add_definitions(-DRMD_MAX_EXTENT_EPIPOLAR_SEARCH=100)


################################################################################
# Sources
################################################################################


set(root ${CMAKE_CURRENT_SOURCE_DIR})

set(CUDA_HEADER_FILES
        ${root}/include/psl/deviceBuffer.cuh
        ${root}/include/psl/deviceImage.cuh
)

set(CUDA_SOURCE_FILES
        ${root}/src/cudaPlaneSweep.cu
        ${root}/src/cudaFishEyePlaneSweep.cu
        ${root}/src/deviceBuffer.cu
        ${root}/src/deviceImage.cu
        ${root}/src/cudaFishEyeImageProcessor.cu
)

set(HEADER_FILES
        ${root}/include/psl/cameraMatrix.h
        ${root}/include/psl/colorMapJet.h
        ${root}/include/psl/common.h
        ${root}/include/psl/configFile.h
        ${root}/include/psl/cudaCommon.h
        ${root}/include/psl/cudaFishEyeImageProcessor.h
        ${root}/include/psl/cudaFishEyePlaneSweep.h
        ${root}/include/psl/cudaPlaneSweep.h
        ${root}/include/psl/depthMap.h
        ${root}/include/psl/deviceBuffer.h
        ${root}/include/psl/deviceImage.h
        ${root}/include/psl/exception.h
        ${root}/include/psl/fishEyeCameraMatrix.h
        ${root}/include/psl/fishEyeDepthMap.h
        ${root}/include/psl/grid.h
        ${root}/include/psl/ioTools.h
)

set(SOURCE_FILES
        ${root}/src/cameraMatrix.cpp
        ${root}/src/colorMapJet.cpp
        ${root}/src/configFile.cpp
        ${root}/src/depthMap.cpp
        ${root}/src/fishEyeCameraMatrix.cpp
        ${root}/src/fishEyeDepthMap.cpp
        ${root}/src/grid.cpp
        ${root}/src/cudaFishEyeImageProcessor.cpp
        ${root}/src/deviceBuffer.cpp
        ${root}/src/deviceImage.cpp
        ${root}/src/ioTools.cpp
        ${root}/src/cudaFishEyePlaneSweep.cpp
        ${root}/src/cudaPlaneSweep.cpp
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
set(psl_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)


################################################################################
# Build
################################################################################


## psl cuda
include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/psl
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

## psl cuda
add_library(${LIBRARY_NAME}_cuda
        ${CUDA_HEADER_FILES}
        ${CUDA_SOURCE_FILES}
)

target_include_directories(${LIBRARY_NAME}_cuda
        PUBLIC
            ${CUDA_INCLUDE_DIRS}
)

## psl
add_library(${LIBRARY_NAME} SHARED
        ${SOURCE_FILES}
        ${HEADER_FILES}
        ${FORM_HEADERS}
        ${HEADERS_MOC}
        ${RESOURCES_RCC}
)

target_include_directories(${LIBRARY_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # for headers when building
        $<INSTALL_INTERFACE:${OpenREALM_INC_INSTALL_DIR}> # for client in install mode
)

target_link_libraries(${LIBRARY_NAME}
        PUBLIC
            ${LIBRARY_NAME}_cuda
        PRIVATE
            ${OpenCV_LIBRARIES}
            ${Boost_LIBRARIES}
            cuda
            ${CUDA_LIBRARIES}
            ${CUDA_CUDART_LIBRARY}
)

add_definitions(
        -Wno-deprecated-declarations
)


################################################################################
# Install
################################################################################

set_target_properties(${LIBRARY_NAME} PROPERTIES OUTPUT_NAME "open_realm_${LIBRARY_NAME}-${OpenREALM_VERSION}")

install(TARGETS ${LIBRARY_NAME} ${LIBRARY_NAME}_cuda EXPORT OpenREALMTargets
        RUNTIME       DESTINATION ${OpenREALM_RUNTIME_INSTALL_DIR}
        LIBRARY       DESTINATION ${OpenREALM_LIBRARY_INSTALL_DIR}
        ARCHIVE       DESTINATION ${OpenREALM_ARCHIVE_INSTALL_DIR}
        FRAMEWORK     DESTINATION ${OpenREALM_FRAMEWORK_INSTALL_DIR})

# Headers
install(
        DIRECTORY include/${LIBRARY_NAME}
        DESTINATION ${OpenREALM_INC_INSTALL_DIR}/3rdparty
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp"
        PATTERN "*.cuh"
)


################################################################################
# Test
################################################################################

