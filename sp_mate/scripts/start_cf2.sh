#!/bin/bash

# call like following:
# $ start_cf2
# $ start_cf2 19951 INADDR_ANY
# echo "sourcing ROS paths..."
# source ~/dev/sp/source_ros.bash

cf_gazebo_path=$(rospack find crazyflie_gazebo)
to_cf2="../crazyflie-firmware/sitl_make/build/cf2"
# mav_id=$1
port_num=$1
cf2_args="$port_num INADDR_ANY"

total_path="${cf_gazebo_path}/${to_cf2}"
echo "cf2 path : $total_path"
echo "calling with : $cf2_args"
$total_path $cf2_args 
