cmake_minimum_required(VERSION 2.8.3)
project(realm_rviz_plugin)
find_package(catkin REQUIRED COMPONENTS rviz cv_bridge geographic_msgs geodesy realm_msgs)

find_package(PCL 1.7 REQUIRED) 

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

set (CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake/Qt5")
find_package(Qt5Core)
find_package(Qt5Gui)
find_package(Qt5Widgets)
include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${Qt5Gui_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})


add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_cpp(MOC_FILES
  src/ground_image_display.h
)

set(SOURCE_FILES
  src/ground_image_display.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}
	realm_msgs_generate_messages_cpp
)

add_executable(ground_image_test_node
	src/ground_image_test_publisher.cpp
)

add_dependencies(ground_image_test_node
	realm_msgs_generate_messages_cpp
)

add_definitions(-std=c++11)

target_link_libraries(ground_image_test_node
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)


install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})




