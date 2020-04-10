cmake_minimum_required(VERSION 2.8.3)
project(realm_ros)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        pcl_ros
        geodesy
        cv_bridge
        tf
        std_msgs
        nav_msgs
        sensor_msgs
        geographic_msgs
        realm_msgs
        realm_core
        realm_io
        realm_stages
)

find_package(OpenCV 3)
find_package(cmake_modules REQUIRED)

####################
## Catkin Package ##
####################

catkin_package(
        INCLUDE_DIRS
            include
        LIBRARIES
            ${PROJECT_NAME}
        CATKIN_DEPENDS
            roscpp
            roslib
            pcl_ros geodesy
            cv_bridge
            tf
            std_msgs
            nav_msgs
            sensor_msgs
            geographic_msgs
            realm_msgs
            realm_core
            realm_io
            realm_stages
            cmake_modules
        DEPENDS
            OpenCV
)

###################
## Build Library ##
###################

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${cmake_modules_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME} SHARED
        src/realm_ros_lib/grabber_exiv2_node.cpp
        src/realm_ros_lib/stage_node.cpp
        src/realm_ros_lib/conversions.cpp
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
target_link_libraries(realm_exiv2_grabber ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${cmake_modules_LIBRARIES})

add_executable(realm_stage_node src/stage_node_main.cpp)
add_dependencies(realm_stage_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(realm_stage_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${cmake_modules_LIBRARIES} -lCGAL)

## Compile Orb vocabulary
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools)
#add_executable(bin_vocabulary tools/bin_vocabulary.cc)
#target_link_libraries(bin_vocabulary ${PROJECT_NAME} ${catkin_LIBRARIES})

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
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)