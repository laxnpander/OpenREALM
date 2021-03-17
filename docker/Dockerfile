FROM nvidia/cuda:10.0-devel-ubuntu18.04

ARG DEBIAN_FRONTEND=noninteractive

# Basic deps
RUN apt-get update && apt-get install -y \
    build-essential \
    pkg-config \
    git \
    wget \
    curl \
    unzip \
    cmake \
    apt-utils

# Install cmake - UNCOMMENT + clone
# RUN cd /root/OpenREALM/tools && chmod u+x install_cmake.sh && ./install_cmake.sh

# Install other deps
RUN apt-get install -y -q \
    # General packages
    apt-utils ca-certificates lsb-release gnupg2 curl libproj-dev \
    # Eigen3 for several linear algebra problems - REMOVE
    libeigen3-dev \
    # Gdal library for conversions between UTM and WGS84
    gdal-bin \
    # Cgal library for delauney 2.5D triangulation and mesh creation
    libcgal-dev libcgal-qt5-dev \
    # PCL for writing point clouds and mesh data
    libpcl-dev \
    # Exiv2 for Exif tagging. REMOVE apt-utils
    exiv2 libexiv2-dev apt-utils \
    # Used by Pangolin/OpenGL
    libglew-dev libxkbcommon-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev \
    libxi-dev libxmu-dev libxmu-headers x11proto-input-dev \
    # g2o dependencies
    libatlas-base-dev libsuitesparse-dev \
    # OpenCV dependencies
    libgtk-3-dev ffmpeg \
    libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libavresample-dev \
    # eigen dependencies
    gfortran \
    # other dependencies
    libyaml-cpp-dev libgoogle-glog-dev libgflags-dev

# Install Eigen
RUN cd ~ && mkdir Eigen3 && cd Eigen3 \
    && wget -q https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.bz2 \
    && tar xf eigen-3.3.9.tar.bz2 \
    && rm -rf eigen-3.3.9.tar.bz2 \
    && cd eigen-3.3.9 \
    && mkdir -p build && cd build \
    && cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        .. \
    && make -j4 \
    && make install

# Install opencv
RUN set -ex \
    && cd ~ && git clone --depth 1 -b 3.3.1 https://github.com/opencv/opencv.git 

RUN set -ex \
    && cd ~/opencv && mkdir build && cd build \
    && cmake -D ENABLE_CXX11=ON -D WITH_CUDA=OFF -D CMAKE_BUILD_TYPE=RELEASE -D BUILD_EXAMPLES=OFF -D BUILD_opencv_apps=OFF -D BUILD_DOCS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF -D CMAKE_INSTALL_PREFIX=/usr/local .. \
    && make -j8 && make install

# Install DBoW2
RUN set -ex \
    && cd ~ && mkdir DBoW2 && cd DBoW2 \
    && git clone https://github.com/shinsumicco/DBoW2.git \
    && cd DBoW2 \
    && mkdir build && cd build \
    && cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        .. \
    && make -j4 \
    && make install

# Install G2O
RUN set -ex \
    && cd ~ && mkdir g2o && cd g2o \
    && git clone https://github.com/RainerKuemmerle/g2o.git \
    && cd g2o \
    && git checkout 9b41a4ea5ade8e1250b9c1b279f3a9c098811b5a \
    && mkdir build && cd build \
    && cmake \
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
        .. \
    && make -j4 \
    && make install

# Install OpenVSLAM
RUN set -ex \
    && cd ~ && mkdir openvslam && cd openvslam \
    && git clone https://github.com/laxnpander/openvslam.git \
    && cd openvslam && mkdir build && cd build \
    && cmake \
        -DBUILD_WITH_MARCH_NATIVE=ON \
        -DUSE_PANGOLIN_VIEWER=OFF \
        -DUSE_SOCKET_PUBLISHER=OFF \
        -DUSE_STACK_TRACE_LOGGER=ON \
        -DBOW_FRAMEWORK=DBoW2 \
        -DBUILD_TESTS=ON \
        .. \
    && make -j4 \
    && make install

# Finally install OpenREALM Librararies
RUN set -ex \
    && cd ~ && mkdir OpenREALM && cd OpenREALM \
    && git clone https://github.com/laxnpander/OpenREALM.git \
    && cd OpenREALM && OPEN_REALM_DIR=$(pwd) \
    && cd $OPEN_REALM_DIR && mkdir build && cd build && cmake -DTESTS_ENABLED=ON .. \
    && make -j $(nproc --all) && make install

# Install ROS Noetic
RUN set -ex \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update \
    && apt-get install -y -q ros-melodic-desktop
RUN set -ex \
    && apt-get install -y -q ros-melodic-geographic-msgs ros-melodic-geodesy \
        ros-melodic-cv-bridge ros-melodic-rviz ros-melodic-pcl-ros

# Create catkin workspace and clone the repo
RUN set -ex \
    && cd / && mkdir -p catkin_ws/src \
    && cd catkin_ws/src \
    && git clone https://github.com/laxnpander/OpenREALM_ROS1_Bridge.git

# Set workdir
WORKDIR /catkin_ws

# Clone rviz_satellite for rviz plugins
RUN set -ex && cd ./src && git clone https://github.com/gareth-cross/rviz_satellite.git

# Build catkin workspace
RUN set -ex && . /opt/ros/melodic/setup.sh && catkin_make -DCMAKE_BUILD_TYPE=Release

# Install rosbrindge suite
RUN apt-get install -yq --no-install-recommends ros-melodic-rosbridge-suite

# Setup .bashrc and /ros_entrypoint.sh
RUN set -ex \
    && echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc \
    && echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc \
    && echo 'export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH' >> /root/.bashrc 
    # && sed --in-place --expression \
    # '$isource "/catkin_ws/devel/setup.bash"' \
    # /ros_entrypoint.sh

CMD ["/bin/bash"]
