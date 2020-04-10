cmake_minimum_required(VERSION 2.8.3)
project(realm_msgs)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        sensor_msgs
        message_generation
)

add_message_files(
  FILES
  Frame.msg
  Georeference.msg
  GroundImageCompressed.msg
  Pinhole.msg
  CvGridMap.msg
)

add_service_files(
  FILES
  ParameterChange.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

####################
## Catkin Package ##
####################

catkin_package(
        CATKIN_DEPENDS
            roscpp
            std_msgs
            sensor_msgs
            geometry_msgs
            message_runtime
)
