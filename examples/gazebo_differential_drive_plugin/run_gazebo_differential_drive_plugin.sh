#!/bin/sh
# export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$(dirname "$0") #run once per session
gazebo $(dirname "$0")/differential_drive.world --verbose
