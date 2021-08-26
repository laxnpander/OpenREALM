
cmake_minimum_required(VERSION 3.15)

set(LIBRARY_NAME realm_core)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

################################################################################
# Dependencies
################################################################################

# Fix to avoid OpenCV package confusion with ROS melodic
find_package(OpenCV 3.3.1 EXACT QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 3 QUIET)
endif()
if (NOT OpenCV_FOUND)
    find_package(OpenCV 4 REQUIRED)
    message(WARNING "OpenCV 4 Support is experimental, use at your own risk!")
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GDAL REQUIRED)


################################################################################
# Sources
################################################################################


set(root ${CMAKE_CURRENT_SOURCE_DIR})

set(HEADER_FILES
        ${root}/include/realm_core/analysis.h
        ${root}/include/realm_core/camera.h
        ${root}/include/realm_core/camera_settings.h
        ${root}/include/realm_core/camera_settings_factory.h
        ${root}/include/realm_core/conversions.h
        ${root}/include/realm_core/cv_grid_map.h
        ${root}/include/realm_core/depthmap.h
        ${root}/include/realm_core/enums.h
        ${root}/include/realm_core/frame.h
        ${root}/include/realm_core/imu_settings.h
        ${root}/include/realm_core/loguru.h
        ${root}/include/realm_core/plane_fitter.h
        ${root}/include/realm_core/settings_base.h
        ${root}/include/realm_core/stereo.h
        ${root}/include/realm_core/point_cloud.h
        ${root}/include/realm_core/structs.h
        ${root}/include/realm_core/timer.h
        ${root}/include/realm_core/tree_node.h
        ${root}/include/realm_core/utm32.h
        ${root}/include/realm_core/wgs84.h
        ${root}/include/realm_core/worker_thread_base.h
)

set(SOURCE_FILES
        ${root}/src/timer.cpp
        ${root}/src/analysis.cpp
        ${root}/src/stereo.cpp
        ${root}/src/point_cloud.cpp
        ${root}/src/conversions.cpp
        ${root}/src/camera.cpp
        ${root}/src/depthmap.cpp
        ${root}/src/frame.cpp
        ${root}/src/settings_base.cpp
        ${root}/src/camera_settings_factory.cpp
        ${root}/src/cv_grid_map.cpp
        ${root}/src/worker_thread_base.cpp
        ${root}/src/plane_fitter.cpp
        src/depthmap.cpp)

# Organize the source and header files into groups
source_group("Headers" FILES ${HEADER_FILES})
source_group("Source" FILES ${SOURCE_FILES})
source_group("Forms" FILES ${FORM_FILES})
source_group("Resources" FILES ${RESOURCE_FILES})

if(CMAKE_VERSION VERSION_GREATER 3.8)
    source_group(TREE ${root} FILES ${HEADER_FILES} ${SOURCE_FILES})
endif()

# Define the folder containing the header files for this library
set(realm_core_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)


################################################################################
# Build
################################################################################


include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${GDAL_INCLUDE_DIR}
)

add_library(${LIBRARY_NAME}
        ${SOURCE_FILES}
        ${HEADER_FILES}
        ${FORM_HEADERS}
        ${HEADERS_MOC}
        ${RESOURCES_RCC}
)

target_include_directories(${LIBRARY_NAME}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # for headers when building
            $<INSTALL_INTERFACE:${OpenREALM_INC_INSTALL_DIR}>
)

target_link_libraries(${LIBRARY_NAME}
        PUBLIC
            ${OpenCV_LIBRARIES}
            Eigen3::Eigen
        PRIVATE
            ${GDAL_LIBRARY}
            dl
)

add_definitions(
        -Wno-deprecated-declarations
)


################################################################################
# Install
################################################################################


set_target_properties(${LIBRARY_NAME} PROPERTIES OUTPUT_NAME "open_${LIBRARY_NAME}-${OpenREALM_VERSION}")

install(TARGETS ${LIBRARY_NAME} EXPORT OpenREALMTargets
        RUNTIME       DESTINATION ${OpenREALM_RUNTIME_INSTALL_DIR}
        LIBRARY       DESTINATION ${OpenREALM_LIBRARY_INSTALL_DIR}
        ARCHIVE       DESTINATION ${OpenREALM_ARCHIVE_INSTALL_DIR}
        FRAMEWORK     DESTINATION ${OpenREALM_FRAMEWORK_INSTALL_DIR}
)

# Headers
install(
        DIRECTORY include/${LIBRARY_NAME}
        DESTINATION ${OpenREALM_INC_INSTALL_DIR}
        FILES_MATCHING
            PATTERN "*.h"
            PATTERN "*.hpp"
)


################################################################################
# Test
################################################################################


if(TESTS_ENABLED)
    include(GoogleTest)

    add_executable(run_realm_core_tests
            test/test_realm_core.cpp
            test/test_helper.cpp
            test/conversion_test.cpp
            test/cvgridmap_test.cpp
            test/depthmap_test.cpp
            test/frame_test.cpp
            test/pinhole_test.cpp
            test/plane_fitter_test.cpp
            test/settings_test.cpp
            test/stereo_test.cpp
            test/worker_thread_test.cpp
    )

    # Standard linking to gtest stuff.
    target_link_libraries(run_realm_core_tests gtest_main)

    # Extra linking for the project.
    target_link_libraries(run_realm_core_tests ${LIBRARY_NAME})

    add_custom_command(TARGET run_realm_core_tests PRE_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/test/data $<TARGET_FILE_DIR:run_realm_core_tests>
            )

    # This is so you can do 'make test' to see all your tests run, instead of
    # manually running the executable run_unit_tests to see those specific tests.
    add_test(
            NAME
                realm_core_unit_tests
            COMMAND
                ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}/run_realm_core_tests
    )
endif()