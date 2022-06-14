#!/bin/sh

set -e

rm -rf genn
git clone --depth=1 https://github.com/genn-team/genn

# Our current examples only need CPU, so don't bother building the CUDA bits
export CUDA_PATH=
export OPENCL_PATH=

cd genn
make -j`nproc`
