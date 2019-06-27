#!/bin/sh
trap 'kill %1' SIGINT
world_file=differential_drive.world
display_flag=''
camera_url='/gazebo/default/differential_drive_robot/simple_camera/link/camera/image'

print_usage() {
  printf "Usage: ./run.sh <-d> <-p>\n"
}

while getopts 'dp' flag; do
  case "${flag}" in
    d) display_flag='-s' ;;
    p)  world_file=differential_drive_panoramic.world 
        camera_url='/gazebo/default/differential_drive_robot/panoramic_camera/link/camera/image'
        display_flag='-p' ;;
    *) print_usage
       exit 1 ;;
  esac
done

GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$(dirname "$0") gazebo --verbose $world_file & ./gazebo_tank $display_flag $camera_url
