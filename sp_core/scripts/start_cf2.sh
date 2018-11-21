#!/bin/bash

# call like following:
# $ start_cf2
# $ start_cf2 1951 INADDR_ANY

cf_gazebo_path=$(rospack find crazyflie_gazebo)
to_cf2="../crazyflie-firmware/sitl_make/build/cf2"

total_path="${cf_gazebo_path}/${to_cf2}"
echo "cf2 path : $total_path"
echo "calling with : $@"
$total_path $@