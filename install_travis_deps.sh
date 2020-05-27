#!/bin/sh
set -ex

cd ~ && mkdir Pangolin && cd Pangolin && git clone https://github.com/stevenlovegrove/Pangolin.git && \
            cd Pangolin && mkdir build && cd build && cmake .. && make -j && make install

mkdir -p ~/cmake_ws/src && cd ~/cmake_ws/src && git clone -b cmake_only https://github.com/laxnpander/OpenREALM.git && \
cd OpenREALM && mkdir build && cd build && cmake .. && make