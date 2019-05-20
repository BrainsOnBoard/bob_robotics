#!/bin/bash

build_all() {
    # Make sure plog headers are available before we start building things
    git submodule update --init $(dirname "$0")/../third_party/plog

    types=$1; shift
    goodcount=0
    projectcount=0
    skipcount=0
    for project in $@; do
        # Skip if this isn't a directory
        if [ ! -d $project ]; then continue; fi

        # Skip if the file .test_skip exists in folder
        if [ -f $project/.test_skip ]; then
            (( skipcount++ ))
            continue
        fi

        # If MakefileTest exists, use that instead of Makefile
        if [ -f $project/MakefileTest ]; then
            makefile="-f MakefileTest"
        else
            if [ ! -f $project/Makefile ]; then
                # There's no makefile, so skip it
                continue
            fi

            makefile=
        fi

        echo -e "\e[34m========== BUILDING $project ==========\e[39m"
        cd "$project"
        make clean $makefile
        if make all $makefile -j `nproc`; then
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

    if [ $skipcount -gt 0 ]; then
        echo $skipcount projects skipped.
    fi

    if [ $goodcount -lt $projectcount ]; then
        exit 1
    fi
}