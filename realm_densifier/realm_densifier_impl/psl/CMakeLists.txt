cmake_minimum_required(VERSION 2.8.3)
project(psl)



find_package(catkin REQUIRED)
find_package(OpenCV 3)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(cmake_modules REQUIRED)

find_package(CUDA)
if(NOT CUDA_FOUND)
    message(STATUS "CUDA not found. Skipping PSL package...\n")
    return()
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(EIGEN3 QUIET)
if(NOT EIGEN3_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
    set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/CUDA_Helper.cmake)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -O3 --generate-code ${CUDA_GENCODE} -use_fast_math -D_FORCE_INLINES)
set(CUDA_PROPAGATE_HOST_FLAGS OFF)
set(CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE OFF)

add_definitions(-DRMD_CORR_PATCH_SIDE=5)
add_definitions(-DRMD_MAX_EXTENT_EPIPOLAR_SEARCH=100)


catkin_package(
        INCLUDE_DIRS include include/psl
        LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_cuda
        CATKIN_DEPENDS cmake_modules
        DEPENDS CUDA OpenCV Boost EIGEN3
)

###################
## Build Library ##
###################

include_directories(
        include
        include/psl
        ${CUDA_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${cmake_modules_INCLUDE_DIRS}
)

## psl_cuda
list(APPEND CU_SOURCE_FILES
        src/psl_lib/psl_stereo/cudaPlaneSweep.cu
        src/psl_lib/psl_stereo/cudaFishEyePlaneSweep.cu
        src/psl_lib/psl_cudaBase/deviceBuffer.cu
        src/psl_lib/psl_cudaBase/deviceImage.cu
        src/psl_lib/psl_cudaBase/cudaFishEyeImageProcessor.cu
        OPTIONS
        -use_fast_math
        #-arch sm_10
        #-Xcompiler -fpic
)
cuda_add_library(${PROJECT_NAME}_cuda
        ${CU_SOURCE_FILES}
)

## psl
add_library(${PROJECT_NAME} SHARED
        src/psl_lib/psl_base/cameraMatrix.cpp
        src/psl_lib/psl_base/colorMapJet.cpp
        src/psl_lib/psl_base/configFile.cpp
        src/psl_lib/psl_base/depthMap.cpp
        src/psl_lib/psl_base/fishEyeCameraMatrix.cpp
        src/psl_lib/psl_base/fishEyeDepthMap.cpp
        src/psl_lib/psl_base/grid.cpp
        src/psl_lib/psl_cudaBase/cudaFishEyeImageProcessor.cpp
        src/psl_lib/psl_cudaBase/deviceBuffer.cpp
        src/psl_lib/psl_cudaBase/deviceImage.cpp
        src/psl_lib/psl_io/ioTools.cpp
        src/psl_lib/psl_stereo/cudaFishEyePlaneSweep.cpp
        src/psl_lib/psl_stereo/cudaPlaneSweep.cpp
)
target_link_libraries(${PROJECT_NAME}
        cuda
        ${CUDA_LIBRARIES}
        ${CUDA_CUDART_LIBRARY}
        ${PROJECT_NAME}_cuda
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${Boost_LIBRARIES}
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