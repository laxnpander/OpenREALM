FROM nvidia/cuda:10.0-devel-ubuntu18.04

ARG DEBIAN_FRONTEND=noninteractive

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
    sudo \
    apt-utils

RUN cd / && wget https://raw.githubusercontent.com/laxnpander/OpenREALM/dev/tools/install_opencv.sh

RUN cd / && sed -i 's/sudo //g' install_opencv.sh && bash install_opencv.sh && cd ~ && rm -rf *

RUN wget https://raw.githubusercontent.com/laxnpander/OpenREALM/dev/tools/install_deps.sh

RUN cd / && sed -i 's/sudo //g' install_deps.sh && apt-get update && export DEBIAN_FRONTEND=noninteractive && \
	bash install_deps.sh && rm -rf /var/lib/apt/lists/*

# Finally install OpenREALM Librararies
RUN set -ex \
    && cd ~ && mkdir OpenREALM && cd OpenREALM \
    && git clone https://github.com/laxnpander/OpenREALM.git \
    && cd OpenREALM && OPEN_REALM_DIR=$(pwd) \
    && git submodule init && git submodule update \
    && cd $OPEN_REALM_DIR && mkdir build && cd build && cmake -DTESTS_ENABLED=ON -DWITH_PCL=ON .. \
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
