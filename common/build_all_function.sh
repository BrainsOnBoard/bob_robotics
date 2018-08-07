#!/bin/bash

build_all() {
    types=$1; shift
    goodcount=0
    projectcount=0
    for project in $@; do
        if [ ! -d $project ]; then continue; fi
        
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
    [$goodcount/$projectcount of the $types built successfully]\e[39m"

    if [ $goodcount -lt $projectcount ]; then
        exit 1
    fi
}