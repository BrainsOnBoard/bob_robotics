#!/bin/bash

# exit if command fails
set -e

cd "$(dirname $0)/.."

# Make sure we have necessary python packages for our python modules
rm -rf virtualenv
python3 -m venv virtualenv
. virtualenv/bin/activate
pip3 install numpy scikit-build

# Configure and build source, continuing if there are errors
rm -rf build
mkdir build
cd build
cmake -DPYTHON_EXECUTABLE="$(which python3)" -DCMAKE_EXPORT_COMPILE_COMMANDS=1 "$@" ..
make -k -j $(nproc)
