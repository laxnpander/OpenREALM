cmake_minimum_required(VERSION 2.8.3)
project(realm_ros)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        pcl_ros
        geodesy
        cv_bridge
        tf
        tf2_geometry_msgs
        std_msgs
        nav_msgs
        sensor_msgs
        geographic_msgs
        realm_msgs
        realm_core
        realm_io
        realm_stages
        )

find_package(OpenCV 3.2)
find_package(cmake_modules REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        DEPENDS realm_msgs realm_core realm_io realm_stages
)

###################
## Build Library ##
###################

add_library(${PROJECT_NAME} SHARED
        src/realm_ros_lib/grabber_exiv2_node.cpp
        src/realm_ros_lib/stage_node.cpp
        src/realm_ros_lib/conversions.cpp
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
add_dependencies(${PROJECT_NAME}
        realm_msgs_generate_messages_cpp
        )
add_definitions(-std=c++11)

#################
## Executables ##
#################

add_executable(realm_exiv2_grabber src/grabber_exiv2_node_main.cpp)
add_dependencies(realm_exiv2_grabber ${catkin_EXPORTED_TARGETS})
target_link_libraries(realm_exiv2_grabber ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(realm_stage_node src/stage_node_main.cpp)
add_dependencies(realm_stage_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(realm_stage_node ${PROJECT_NAME} ${catkin_LIBRARIES})

## Compile Orb vocabulary
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools)
#add_executable(bin_vocabulary tools/bin_vocabulary.cc)
#target_link_libraries(bin_vocabulary ${PROJECT_NAME} ${catkin_LIBRARIES})
