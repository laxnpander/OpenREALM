cmake_minimum_required(VERSION 2.8.3)
project(psl)

#-arch=compute_62 -code=sm_62

find_package(catkin REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenCV 3.3.1 EXACT)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(cmake_modules REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/CUDA_Helper.cmake)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -O3 --generate-code ${CUDA_GENCODE} -use_fast_math -D_FORCE_INLINES)
set(CUDA_PROPAGATE_HOST_FLAGS OFF)
set(CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE OFF)

add_definitions(-DRMD_CORR_PATCH_SIDE=5)
add_definitions(-DRMD_MAX_EXTENT_EPIPOLAR_SEARCH=100)
add_definitions(-std=c++11)


catkin_package(
        INCLUDE_DIRS include include/psl
        LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_cuda
        DEPENDS OpenCV Eigen3 Boost
        LIBRARIES
)

###################
## Build Library ##
###################

include_directories(
        include
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
cuda_add_library(${PROJECT_NAME}_cuda SHARED
        ${CU_SOURCE_FILES}
)
target_include_directories(${PROJECT_NAME}_cuda PUBLIC
        ${CUDA_INCLUDE_DIRS}
)

## psl
add_library(${PROJECT_NAME}
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
target_include_directories(${PROJECT_NAME} PUBLIC
        include
        include/psl
        ${OpenCV_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${cmake_modules_INCLUDE_DIRS}
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
add_definitions(-std=c++11)
