#!/bin/bash
set -ex

cd ~ && git clone -b 3.3.1 https://github.com/opencv/opencv.git \
&& cd ~/opencv && mkdir build && cd build \
&& cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local .. \
&& make && sudo make install