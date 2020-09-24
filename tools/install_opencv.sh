#!/bin/bash
set -ex

cd ~ && git clone -b 3.3.1 https://github.com/opencv/opencv.git \
&& cd ~/opencv && mkdir build && cd build \
&& cmake -D CMAKE_BUILD_TYPE=RELEASE -D BUILD_EXAMPLES=OFF -D BUILD_opencv_apps=OFF -D BUILD_DOCS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF -D CMAKE_INSTALL_PREFIX=/usr/local .. \
&& make && sudo make install
