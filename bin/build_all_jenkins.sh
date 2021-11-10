#!/bin/bash

# exit if command fails
set -e

builddir=$(dirname "$0")/../build
if [ ! -d $builddir ]; then
    mkdir $builddir
fi
cd $builddir

cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 "$@"
make -k -j $(nproc)
