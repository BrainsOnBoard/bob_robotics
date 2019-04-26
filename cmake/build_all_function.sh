#!/bin/bash

builddir=$(dirname "$0")/build
OLDPWD=$PWD
if [ ! -d $builddir ]; then
    mkdir $builddir
fi
cd $builddir

cmake .. && make -k -j `nproc`
ret=$?
cd $OLDPWD

exit $ret
