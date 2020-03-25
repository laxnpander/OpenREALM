cmake_minimum_required(VERSION 3.2)
project(OpenREALM)

# This is a toplevel CMake file which will be ignored by the catkin build system.
# However, it is useful if you want to use CMake based IDEs like CLion for complete
# indexing of all sources and header. 
# ! Do not set any flags here !

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

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



