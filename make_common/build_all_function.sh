#!/bin/bash

builddir=$(dirname "$0")/build
if [ -d $builddir ]; then
    echo Removing old build directory...
    rm -rf $builddir
fi
OLDPWD=$PWD
mkdir $builddir && cd $builddir

cmake .. && make -k -j `nproc`
ret=$?
cd $OLDPWD

exit $ret
