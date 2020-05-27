#!/bin/bash
set -ex

mkdir -p ~/cmake_ws/src && cd ~/cmake_ws/src && git clone -b cmake_only https://github.com/laxnpander/OpenREALM.git && \
cd OpenREALM && mkdir build && cd build && cmake .. && make -j 1