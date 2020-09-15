FROM ubuntu:18.04

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive && \
            apt-get install -yq --no-install-recommends libeigen3-dev gdal-bin \
            libcgal-qt5-dev libpcl-dev exiv2 libexiv2-dev apt-utils libglew-dev \
            libxkbcommon-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev libxi-dev git \
            libxmu-dev libxmu-headers x11proto-input-dev nano ca-certificates cmake g++ gcc \
            lsb-release gnupg2 curl libopencv-dev && rm -rf /var/lib/apt/lists/*

RUN cd / && mkdir Pangolin && cd Pangolin && git clone https://github.com/stevenlovegrove/Pangolin.git && \
            cd Pangolin && mkdir build && cd build && cmake .. && make -j && make install

RUN mkdir -p /cmake_ws/src && cd /cmake_ws/src && git clone https://github.com/laxnpander/OpenREALM.git

RUN cd cmake_ws/src/OpenREALM && mkdir build && cd build && cmake .. && make

CMD ["/bin/bash"]
