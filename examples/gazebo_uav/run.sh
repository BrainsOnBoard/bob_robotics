#!/bin/sh
trap 'kill %1' SIGINT
world_file=iris_demo.world
# display_flag=''

print_usage() {
  printf "Usage: ./run.sh <-d -r>\n"
}

while getopts 'dr' flag; do
  case "${flag}" in
    d)camera_url='/gazebo/default/iris_demo/iris_demo/panoramic_camera/link/camera/image'
      display_flag='-p' ;;
    r)world_file=iris_demo_rothamsted.world
      export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH$ROTHAMSTED_3D_MODEL_PATH/gazebo_models: ;;
    *) print_usage
       exit 1 ;;
  esac
done

GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$(dirname "$0") gazebo --verbose $world_file & ./gazebo_uav $display_flag $camera_url

