#!/bin/sh

basename=$(dirname "$0")
goodcount=0
projectcount=0
for project in "$basename"/*_test; do
    echo -e "\e[34m========== BUILDING $project ==========\e[39m"
    if make -C $project clean all -j `nproc`; then
        echo -e "\e[32m========== $project built successfully ==========\e[39m"
        (( goodcount++ ))
    else
        echo -e "\e[31m========== Building $project failed! ==========\e[39m"
    fi
    (( projectcount++ ))
done

echo -e "\e[34m
BUILD COMPLETED
[$goodcount/$projectcount of the projects built successfully]\e[39m"