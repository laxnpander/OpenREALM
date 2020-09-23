#!/bin/bash
set -ex

# Basic dependencies
sudo apt-get install -y build-essential pkg-config git wget curl unzip

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
sudo apt-get install -y -q apt-utils ca-certificates lsb-release gnupg2 curl libproj-dev

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

# g2o dependencies
sudo apt-get install -y libatlas-base-dev libsuitesparse-dev

# OpenCV dependencies
sudo apt-get install -y libgtk-3-dev
sudo apt-get install -y ffmpeg
sudo apt-get install -y libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libavresample-dev

# eigen dependencies
sudo apt-get install -y gfortran

# other dependencies
sudo apt-get install -y libyaml-cpp-dev libgoogle-glog-dev libgflags-dev

if [[ $(lsb_release -rs) == "16.04" ]]; then

       echo "Its Ubuntu 16.04. Repairing the Links for libproj"
       sudo ln -s /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so /usr/lib/libvtkproj4.so
else
       echo "No problems to repair."
fi

# DBoW2
cd ~ && mkdir DBoW2 && cd DBoW2
git clone https://github.com/shinsumicco/DBoW2.git
cd DBoW2
mkdir build && cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    ..
make -j4
sudo make install

# G2O
cd ~ && mkdir g2o && cd g2o
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout 9b41a4ea5ade8e1250b9c1b279f3a9c098811b5a
mkdir build && cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DCMAKE_CXX_FLAGS=-std=c++11 \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_UNITTESTS=OFF \
    -DBUILD_WITH_MARCH_NATIVE=ON \
    -DG2O_USE_CHOLMOD=OFF \
    -DG2O_USE_CSPARSE=ON \
    -DG2O_USE_OPENGL=OFF \
    -DG2O_USE_OPENMP=ON \
    ..
make -j4
sudo make install

# OpenVSLAM
cd ~ && mkdir openvslam && cd openvslam
mkdir build && cd build
cmake \
    -DBUILD_WITH_MARCH_NATIVE=ON \
    -DUSE_PANGOLIN_VIEWER=OFF \
    -DUSE_SOCKET_PUBLISHER=OFF \
    -DUSE_STACK_TRACE_LOGGER=ON \
    -DBOW_FRAMEWORK=DBoW2 \
    -DBUILD_TESTS=ON \
    ..
make -j4
sudo make install