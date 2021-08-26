#!/bin/bash
set -ex

cd ~/cmake_ws/src/OpenREALM/build/modules/realm_core
./run_realm_core_tests

cd ../realm_io
./run_realm_io_tests
