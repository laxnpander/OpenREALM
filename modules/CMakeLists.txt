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
# along with OpenREALM. If not, see <http://www.gnu.org/licenses/>

cmake_minimum_required(VERSION 3.15)

################################################################################
# Options
################################################################################

# Optional Libraries
option(WITH_EXIV2      "Enable/Disable support for Exiv2 input libraries" ON)
option(WITH_PCL        "Enable/Disable support for PCL libraries"         ON)

# Optional Modules
option(WITH_core       "Enable/Disable building of core library"          ON)
option(WITH_io         "Enable/Disable building of io library"            ON)
option(WITH_densifier  "Enable/Disable building of densifier library"     ON)
option(WITH_ortho      "Enable/Disable building of ortho library"         ON)
option(WITH_stages     "Enable/Disable building of stages library"        ON)
option(WITH_vslam_base "Enable/Disable building of vslam_base library"    ON)


################################################################################
# Add modules
################################################################################

set(BUILD_MODULES "")

if(WITH_core)
    list(APPEND BUILD_MODULES "core")
    add_subdirectory(realm_core)
endif()

if(WITH_io)
    list(APPEND BUILD_MODULES "io")
    add_subdirectory(realm_io)
endif()

if (WITH_densifier)
    list(APPEND BUILD_MODULES "densifier")
    add_subdirectory(realm_densifier/realm_densifier_impl/psl)
    add_subdirectory(realm_densifier/realm_densifier_base)
endif()

if (WITH_ortho)
    list(APPEND BUILD_MODULES "ortho")
    add_subdirectory(realm_ortho)
endif()

if (WITH_stages)
    list(APPEND BUILD_MODULES "stages")
    add_subdirectory(realm_stages)
endif()

if (WITH_vslam_base)
    list(APPEND BUILD_MODULES "vslam_base")
    add_subdirectory(realm_vslam/realm_vslam_base)
endif()

message("-- With OpenREALM Modules: ${BUILD_MODULES}")