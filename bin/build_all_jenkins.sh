#!/bin/bash

# exit if command fails
set -e

cd "$(dirname $0)/.."

# Make sure we have necessary python packages for our python modules
rm -rf virtualenv
python3 -m venv virtualenv
. virtualenv/bin/activate
pip3 install numpy scikit-build

rm -rf build
mkdir build
cd build

# We build everything with -fPIC because some code (e.g. python modules) needs
# it and it's easier just to enable it everywhere. If we don't do this then we
# get errors when trying to link shared library files against the BoB static
# libraries.
#
# The --allow-multiple-definition is in there because otherwise I get errors
# caused by symbols being defined multiple times though, weirdly, only when
# building everything. It's harmless so just suppress it.
cmake -DCMAKE_CXX_FLAGS="-fPIC -Wl,--allow-multiple-definition" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DBUILD_TEST=ON "$@" ..
make -k -j $(nproc)
