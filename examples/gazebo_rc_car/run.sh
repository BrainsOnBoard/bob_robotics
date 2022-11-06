#!/bin/sh
trap 'kill %1' INT
world_file=simple_world.world
display_flag=''
camera_url='/gazebo/default/cart_front_steer_pancam/simple_camera/link/camera/image'

print_usage() {
  printf "Usage: ./run.sh <-d> <-p>\n"
}

while getopts 'dp' flag; do
  case "${flag}" in
    d) display_flag='-s' ;;
    p)  world_file=outdoor_world.world
        camera_url='/gazebo/default/cart_front_steer_pancam/panoramic_camera/link/camera/image'
        display_flag='-p' ;;
    *) print_usage
       exit 1 ;;
  esac
done

GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$(dirname "$0") gazebo --verbose $world_file &
./gazebo_rc_car $display_flag $camera_url
