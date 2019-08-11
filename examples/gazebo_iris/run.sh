#!/bin/sh
trap 'kill %1' SIGINT
world_file=iris_demo.world
# display_flag=''

print_usage() {
  printf "Usage: ./run.sh <-d>\n"
}

while getopts 'd' flag; do
  case "${flag}" in
    d)camera_url='/gazebo/default/iris_demo/iris_demo/panoramic_camera/link/camera/image'
      display_flag='-p' ;;
    *) print_usage
       exit 1 ;;
  esac
done

GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$(dirname "$0") gazebo --verbose $world_file & ./bob_iris $display_flag $camera_url

