# This file is part of OpenREALM.
#
# Copyright (C) 2018 Alexander Kern <laxnpander at gmail dot com> (Braunschweig University of Technology)
# For more information see <https://github.com/laxnpander/OpenREALM>
#
# OpenREALM is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# OpenREALM is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with OpenREALM. If not, see <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.9)

project(OpenREALM)

set(OpenREALM_VERSION "1.0.0")
set(OpenREALM_VERSION_NUMBER "1000")


################################################################################
# Paths
################################################################################

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(UNIX)
    include(GNUInstallDirs)

    set(OpenREALM_INC_INSTALL_DIR "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}")
    set(OpenREALM_RUNTIME_INSTALL_DIR "${CMAKE_INSTALL_BINDIR}")
    set(OpenREALM_LIBRARY_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}")
    set(OpenREALM_ARCHIVE_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}")
    set(OpenREALM_FRAMEWORK_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}")

    set(OpenREALM_CMAKE_CONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/OpenREALM")
    set(OpenREALM_ADDITIONAL_FILES_INSTALL_DIR "${CMAKE_INSTALL_DATADIR}/OpenREALM")

#elseif(WIN32)
#    set(OpenREALM_INC_INSTALL_DIR "include/OpenREALM")
#    set(OpenREALM_RUNTIME_INSTALL_DIR   "bin")
#    set(OpenREALM_LIBRARY_INSTALL_DIR   "bin")
#    set(OpenREALM_ARCHIVE_INSTALL_DIR   "lib")
#    set(OpenREALM_FRAMEWORK_INSTALL_DIR "bin")
#
#    set(OpenREALM_CMAKE_CONFIG_INSTALL_DIR "share/OpenREALM/cmake")
#    set(OpenREALM_ADDITIONAL_FILES_INSTALL_DIR "share/OpenREALM")
#    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
else()
    message(FATAL_ERROR "Could not set install folders for this platform!")
endif()


################################################################################
# Include CMake dependencies
################################################################################


set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(CheckCXXCompilerFlag)
include(CheckLanguage)
include(CMakePackageConfigHelpers)


################################################################################
# Options
################################################################################

option(TESTS_ENABLED "Whether to build test binaries" OFF)

if(TESTS_ENABLED)
    enable_testing()

    include(cmake/googletest.cmake)
    fetch_googletest(
            ${PROJECT_SOURCE_DIR}/cmake
            ${PROJECT_BINARY_DIR}/googletest
    )
endif()


################################################################################
# Compiler specific configuration
################################################################################


if(CMAKE_BUILD_TYPE)
    message(STATUS "Build type specified as ${CMAKE_BUILD_TYPE}")
else()
    message(STATUS "Build type not specified, using Release")
    set(CMAKE_BUILD_TYPE Release)
    set(IS_DEBUG OFF)
endif()

check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    message(STATUS "CUDA Support enabled.")

    include(FindCUDA)
    set(CUDA_ARCH_LIST Auto CACHE STRING  "List of CUDA architectures (e.g. Pascal, Volta, etc) or \
                                           compute capability versions (6.1, 7.0, etc) to generate code for. \
                                           Set to Auto for automatic detection (default).")
    cuda_select_nvcc_arch_flags(CUDA_ARCH_FLAGS ${CUDA_ARCH_LIST})
    list(APPEND CUDA_NVCC_FLAGS ${CUDA_ARCH_FLAGS})
else()
    message(STATUS "CUDA Support disabled.")
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


################################################################################
# Add modules
################################################################################

add_subdirectory(modules)

################################################################################
# Install
################################################################################

# Create and install version file
set(version_file "${CMAKE_CURRENT_BINARY_DIR}/OpenREALMConfigVersion.cmake")
write_basic_package_version_file(${version_file}
        VERSION ${OpenREALM_VERSION}
        COMPATIBILITY ExactVersion
)
install(FILES ${version_file}
        DESTINATION ${OpenREALM_CMAKE_CONFIG_INSTALL_DIR})

# install readme and license
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE" "${CMAKE_CURRENT_SOURCE_DIR}/README.md"
        DESTINATION ${OpenREALM_ADDITIONAL_FILES_INSTALL_DIR}
)

install(
        EXPORT OpenREALMTargets
        DESTINATION ${OpenREALM_CMAKE_CONFIG_INSTALL_DIR}
        FILE OpenREALMConfig.cmake
)


################################################################################
# Done :)
################################################################################