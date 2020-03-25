cmake_minimum_required(VERSION 3.5)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(PROJECT_NAME OpenREALM)

project(${PROJECT_NAME})

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS ON)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SSE_FLAGS} -O2 -pthread -march=native")
endif()

add_subdirectory(realm_core)
add_subdirectory(realm_densifier/realm_densifier_base)
add_subdirectory(realm_densifier/realm_densifier_impl/psl)
add_subdirectory(realm_io)
add_subdirectory(realm_msgs)
add_subdirectory(realm_ortho)
add_subdirectory(realm_ros)
add_subdirectory(realm_rviz_plugin)
add_subdirectory(realm_stages)
add_subdirectory(realm_vslam/realm_vslam_base)
add_subdirectory(realm_vslam/realm_vslam_impl/orb_slam_2)



