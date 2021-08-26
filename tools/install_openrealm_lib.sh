#!/bin/bash
set -ex

mkdir -p ~/cmake_ws/src && cd ~/cmake_ws/src && git clone -b dev https://github.com/laxnpander/OpenREALM.git && \
cd OpenREALM && git submodule init && git submodule update && mkdir build && cd build && cmake -DTESTS_ENABLED=ON .. && make -j 1
