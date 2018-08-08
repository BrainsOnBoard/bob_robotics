#!/bin/bash

source ../common/build_all_function.sh

basename=$(dirname "$0")

echo Cleaning libs...
for lib in "$basename"/../lib*; do
    make -C "$lib" clean
done

build_all examples "$basename"/*
