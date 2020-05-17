FROM ros:melodic-perception-bionic

RUN apt-get update && apt-get install -y --no-install-recommends python-catkin-tools \
			ros-melodic-geographic-msgs ros-melodic-geodesy ros-melodic-cv-bridge \
            ros-melodic-rviz ros-melodic-pcl-ros && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends libeigen3-dev gdal-bin \
            libcgal-qt5-dev libpcl-dev exiv2 libexiv2-dev apt-utils libglew-dev \
            libxkbcommon-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev libxi-dev \
            libxmu-dev libxmu-headers x11proto-input-dev && rm -rf /var/lib/apt/lists/*

RUN cd / && mkdir Pangolin && cd Pangolin && git clone https://github.com/stevenlovegrove/Pangolin.git && \
            cd Pangolin && mkdir build && cd build && cmake .. && make -j $(nproc --all) && sudo make install

RUN mkdir -p /catkin_ws/src && cd /catkin_ws/src && git clone https://github.com/laxnpander/OpenREALM.git \
            && catkin init --workspace .

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash && cd /catkin_ws; \
                  catkin config --blacklist psl --cmake-args -DCMAKE_BUILD_TYPE=Release; catkin build'

RUN apt-get update && apt-get install -y --no-install-recommends wget && rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]