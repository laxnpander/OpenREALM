#!/bin/bash
set -ex

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd $SCRIPT_DIR/../build/modules/realm_core
./run_realm_core_tests

cd ../realm_io
./run_realm_io_tests
