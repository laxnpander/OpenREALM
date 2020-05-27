FROM ubuntu:18.04

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive && \
            apt-get install -yq --no-install-recommends libeigen3-dev gdal-bin \
            libcgal-qt5-dev libpcl-dev exiv2 libexiv2-dev apt-utils libglew-dev \
            libxkbcommon-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev libxi-dev git \
            libxmu-dev libxmu-headers x11proto-input-dev nano && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -yq --no-install-recommends ca-certificates cmake g++ gcc && rm -rf /var/lib/apt/lists/*

RUN cd / && mkdir Pangolin && cd Pangolin && git clone https://github.com/stevenlovegrove/Pangolin.git && \
            cd Pangolin && mkdir build && cd build && cmake .. && make -j $(nproc --all) && make install

RUN mkdir -p /cmake_ws/src && cd /cmake_ws/src && git clone -b cmake_only https://github.com/laxnpander/OpenREALM.git

RUN apt-get update && apt-get install -yq --no-install-recommends lsb-release gnupg2

RUN apt-get update && apt-get install curl

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add - \
    && apt-get update && apt-get install -yq --no-install-recommends ros-melodic-desktop-full && echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

RUN cd cmake_ws/src/OpenREALM && mkdir build && cd build && cmake .. && make && make install

CMD ["/bin/bash"]