#!/bin/bash

basename=$(dirname "$0")

echo Cleaning libs...
for lib in "$basename"/../lib*; do
    make -C "$lib" clean
done

goodcount=0
projectcount=0
for project in "$basename"/*_test; do
    echo -e "\e[34m========== BUILDING $project ==========\e[39m"
    cd "$project"
    make clean
    if make all -j `nproc`; then
        echo -e "\e[32m========== $project built successfully ==========\e[39m"
        (( goodcount++ ))
    else
        echo -e "\e[31m========== Building $project failed! ==========\e[39m"
    fi
    (( projectcount++ ))
    cd ..
done

echo -e "\e[34m
BUILD COMPLETED
[$goodcount/$projectcount of the projects built successfully]\e[39m"
