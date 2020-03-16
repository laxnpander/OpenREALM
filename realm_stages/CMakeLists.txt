cmake_minimum_required(VERSION 2.8.3)
project(realm_stages)

find_package(OpenCV 3.2)
find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        realm_core
        realm_vslam_base
        realm_densifier_base
        realm_ortho
        )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        DEPENDS realm_core realm_vslam_base realm_densifier_base realm_ortho
)

###################
## Build Library ##
###################

add_library(${PROJECT_NAME} SHARED
        src/realm_stages_lib/stage_base.cpp
        src/realm_stages_lib/conversions.cpp
        src/realm_stages_lib/stage_settings_factory.cpp
        src/realm_stages_lib/pose_estimation.cpp
        src/realm_stages_lib/densification.cpp
        src/realm_stages_lib/surface_generation.cpp
        src/realm_stages_lib/ortho_rectification.cpp
        src/realm_stages_lib/mosaicing.cpp
        )
target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${cmake_modules_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${cmake_modules_LIBRARIES}
)
add_definitions(-std=c++11)
