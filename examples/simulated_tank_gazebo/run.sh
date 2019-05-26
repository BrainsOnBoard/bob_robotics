#!/bin/sh
./simulated_tank_gazebo & GAZEBO_PLUGIN_PATH=GAZEBO_PLUGIN_PATH:${PWD} gazebo --verbose differential_drive.world