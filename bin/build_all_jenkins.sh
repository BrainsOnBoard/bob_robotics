#!/bin/bash

# exit if command fails
set -e

cd "$(dirname $0)/.."

# Make sure we have necessary python packages for our python modules
rm -rf virtualenv
python -m venv virtualenv
. virtualenv/bin/activate
pip install numpy scikit-build

# Configure and build source, continuing if there are errors
rm -rf build
cmake -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=1 "$@"
make -C build -k -j $(nproc)
