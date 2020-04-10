cmake_minimum_required(VERSION 2.8.3)
project(realm_densifier_base)

find_package(CUDA QUIET)
if(CUDA_FOUND)
    message(STATUS "CUDA found. Compiling densification with CUDA support...\n")
    set(DENSIFIER_WITH_CUDA True)
else(NOT CUDA_FOUND)
    message(STATUS "CUDA not found. Skipping PSL package...\n")
    set(DENSIFIER_WITH_CUDA False)
endif()

IF (DENSIFIER_WITH_CUDA)
    add_definitions(-DUSE_CUDA)
    list(APPEND DENSIFIER_LIBS psl)
    list(APPEND DENSIFIER_SOURCES src/realm_densifier_base_lib/plane_sweep.cpp)
ENDIF()

find_package(catkin REQUIRED COMPONENTS
        cmake_modules
        realm_core
        realm_io
        ${DENSIFIER_LIBS}
        )

find_package(OpenCV 3)

####################
## Catkin Package ##
####################

catkin_package(
        INCLUDE_DIRS
            include
        LIBRARIES
            ${PROJECT_NAME}
        CATKIN_DEPENDS
            cmake_modules
            realm_core
            realm_io
            ${DENSIFIER_LIBS}
        DEPENDS
            OpenCV
)

###################
## Build Library ##
###################

list(APPEND DENSIFIER_SOURCES
        src/realm_densifier_base_lib/densifier_factory.cpp
        src/realm_densifier_base_lib/densifier_settings_factory.cpp
        src/realm_densifier_base_lib/densifier_dummy.cpp
        src/realm_densifier_base_lib/sparse_depth.cpp
        )

## realm densifier
include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME} SHARED
        ${DENSIFIER_SOURCES}
        )
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        )
add_definitions(-std=c++11)

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