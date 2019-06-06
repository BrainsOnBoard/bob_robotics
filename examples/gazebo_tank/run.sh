#!/bin/sh
trap 'kill %1' SIGINT
GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$(dirname "$0") gazebo --verbose differential_drive.world & ./gazebo_tank "$@"
