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

set(OpenREALM_VERSION "1.0")
set(OpenREALM_VERSION_NUMBER "1000")


################################################################################
# Include CMake dependencies
################################################################################


set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(CheckCXXCompilerFlag)
include(CheckLanguage)


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

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


################################################################################
# Add modules
################################################################################


#add_subdirectory(apps)
#add_subdirectory(data)
#add_subdirectory(doc)
add_subdirectory(modules)
#add_subdirectory(samples)


################################################################################
# Done :)
################################################################################