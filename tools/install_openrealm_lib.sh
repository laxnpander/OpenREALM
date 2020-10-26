#!/bin/bash
set -ex

mkdir -p ~/cmake_ws/src && cd ~/cmake_ws/src && git clone -b dev https://github.com/laxnpander/OpenREALM.git && \
cd OpenREALM && mkdir build && cd build && cmake .. && make -j 1
