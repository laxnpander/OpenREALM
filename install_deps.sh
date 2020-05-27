#!/bin/sh
set -ex

# Check CMake version and update if necessary
OUTPUT=$(cmake --version)
read CMAKE_VERSION_MAJOR CMAKE_VERSION_MINOR CMAKE_VERSION_PATCH <<< ${OUTPUT//[^0-9]/ }

if [ "${CMAKE_VERSION_MINOR}" -le 9 ]; then

  echo 'CMake Version is too old! Trying to download newer version '

  CMAKE_FILE="cmake-3.10.3-Linux-x86_64"

  # Check if file already exists
  if [ ! -e "${CMAKE_FILE}.tar.gz" ]; then
    wget https://cmake.org/files/v3.10/${CMAKE_FILE}.tar.gz
  fi

  # Remove existing unpacked cmake folder
  if [ -d "${CMAKE_FILE}" ]; then
    rm -r ${CMAKE_FILE}
  fi

  tar xvzf ${CMAKE_FILE}.tar.gz

  export PATH="`pwd`/${CMAKE_FILE}/bin:$PATH"
fi

# Update the Apt Cache
sudo apt-get update

# General packages
sudo apt-get install -y -q apt-utils ca-certificates lsb-release gnupg2 curl

sudo apt-get install -y -q libopencv-dev
# Eigen3 for several linear algebra problems
sudo apt-get install -y -q libeigen3-dev

# Gdal library for conversions between UTM and WGS84
sudo apt-get install -y -q gdal-bin

# Cgal library for delauney 2.5D triangulation and mesh creation
sudo apt-get install -y -q libcgal-dev
sudo apt-get install -y -q libcgal-qt5-dev

# PCL for writing point clouds and mesh data
sudo apt-get install -y -q libpcl-dev

# Exiv2 for Exif tagging.
sudo apt-get install -y -q exiv2 libexiv2-dev apt-utils

# Used by Pangolin/OpenGL
sudo apt-get install -y -q libglew-dev libxkbcommon-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev
sudo apt-get install -y -q libxi-dev libxmu-dev libxmu-headers x11proto-input-dev

# Pangolin
cd ~ && mkdir Pangolin && cd Pangolin
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && mkdir build && cd build && cmake ..
make -j $(nproc --all) && sudo make install
