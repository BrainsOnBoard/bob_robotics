#!/bin/sh
# export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$(dirname "$0")
gzserver $(dirname "$0")/gazebo_plugin.xml --verbose
