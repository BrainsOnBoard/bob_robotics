#!/bin/bash

source ../make_common/build_all_function.sh

basename=$(dirname "$0")

build_all tools "$basename"/*