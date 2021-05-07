
cmake_minimum_required(VERSION 3.15)

set(LIBRARY_NAME realm_io)

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

find_package(GDAL REQUIRED)

find_package(Boost REQUIRED COMPONENTS
        filesystem
)


################################################################################
# Sources
################################################################################


set(root ${CMAKE_CURRENT_SOURCE_DIR})

set(HEADER_FILES
        ${root}/include/realm_io/cv_export.h
        ${root}/include/realm_io/cv_import.h
        ${root}/include/realm_io/gis_export.h
        ${root}/include/realm_io/gdal_continuous_writer.h
        ${root}/include/realm_io/mvs_export.h
        ${root}/include/realm_io/realm_export.h
        ${root}/include/realm_io/realm_import.h
        ${root}/include/realm_io/utilities.h
)

set(SOURCE_FILES
        ${root}/src/cv_export.cpp
        ${root}/src/cv_import.cpp
        ${root}/src/gis_export.cpp
        ${root}/src/gdal_continuous_writer.cpp
        ${root}/src/mvs_export.cpp
        ${root}/src/realm_export.cpp
        ${root}/src/realm_import.cpp
        ${root}/src/utilities.cpp
        include/realm_io/gdal_continuous_writer.h src/gdal_continuous_writer.cpp)

# Conditionally add exif import/export to handle removing Exiv2 dependancy
if (WITH_EXIV2)
    list(APPEND HEADER_FILES
            ${root}/include/realm_io/exif_export.h
            ${root}/include/realm_io/exif_import.h)
    list(APPEND SOURCE_FILES
            ${root}/src/exif_export.cpp
            ${root}/src/exif_import.cpp)
endif()

# Conditionally add PCL import/export to handle removing PCL dependancy
if (WITH_PCL)
    list(APPEND HEADER_FILES
            ${root}/include/realm_io/pcl_export.h)
    list(APPEND SOURCE_FILES
            ${root}/src/pcl_export.cpp)
endif()

# Organize the source and header files into groups
source_group("Headers" FILES ${HEADER_FILES})
source_group("Source" FILES ${SOURCE_FILES})
source_group("Forms" FILES ${FORM_FILES})
source_group("Resources" FILES ${RESOURCE_FILES})

if(CMAKE_VERSION VERSION_GREATER 3.8)
    source_group(TREE ${root} FILES ${HEADER_FILES} ${SOURCE_FILES})
endif()

# Define the folder containing the header files for this library
set(realm_io_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)

################################################################################
# Build
################################################################################

include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${GDAL_INCLUDE_DIR}
        ${Boost_INCLUDE_DIR}
)

add_library(${LIBRARY_NAME}
        ${SOURCE_FILES}
        ${HEADER_FILES}
        ${FORM_HEADERS}
        ${HEADERS_MOC}
        ${RESOURCES_RCC}
)

target_include_directories(${LIBRARY_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # for headers when building
        $<INSTALL_INTERFACE:${OpenREALM_INC_INSTALL_DIR}> # for client in install mode
)

target_link_libraries(${LIBRARY_NAME}
        PUBLIC
            realm_core
        PRIVATE
            ${OpenCV_LIBRARIES}
            ${GDAL_LIBRARY}
            ${Boost_LIBRARIES}
)

################################################################################
# Options
################################################################################

# Optional Exiv2 file input support
# We can disable this to avoid GPL licensing issues
if(WITH_EXIV2)
    find_package(Exiv2 REQUIRED)
    add_compile_definitions(WITH_EXIV2)

    include_directories(${Exiv2_INCLUDE_DIRS})
    target_link_libraries(${LIBRARY_NAME}
            PRIVATE
            ${Exiv2_LIBRARIES})
else()
    message(STATUS "** WARNING ** Exiv2 Disabled.  Some I/O features will not be present.")
endif()

if (WITH_PCL)
    find_package(PCL 1.7 REQUIRED)
    add_compile_definitions(WITH_PCL)

    include_directories(
            ${PCL_INCLUDE_DIRS}
            ${PCL_LIBRARY_DIRS})
    target_link_libraries(${LIBRARY_NAME}
            PRIVATE
            ${PCL_LIBRARIES}
            )
else()
    message(STATUS "** WARNING ** PCL Disabled.  Some I/O features will not be present.")
endif()

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

    add_executable(run_realm_io_tests
            test/test_realm_io.cpp
            test/mvs_io_test.cpp
            test/cv_io_test.cpp
            test/realm_io_test.cpp
            )

    # Standard linking to gtest stuff.
    target_link_libraries(run_realm_io_tests gtest_main)

    # Extra linking for the project.
    target_link_libraries(run_realm_io_tests ${LIBRARY_NAME} ${Boost_LIBRARIES})

    add_custom_command(TARGET run_realm_io_tests PRE_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/test/data $<TARGET_FILE_DIR:run_realm_io_tests>
    )

    # This is so you can do 'make test' to see all your tests run, instead of
    # manually running the executable run_unit_tests to see those specific tests.
    add_test(
            NAME
                realm_io_unit_tests
            COMMAND
                ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}/run_realm_io_tests
    )

endif()