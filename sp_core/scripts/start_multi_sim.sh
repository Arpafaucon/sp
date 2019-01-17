#!/bin/bash

echo "sourcing ROS paths..."
source ~/dev/sp/source_ros.bash


# CF2  & spawns
num_drones=${1:-0}
echo "spawning $num_drones drones..."
biggest_id=$(echo "$num_drones - 1" | bc)

for id in `seq 0 $biggest_id`; do
cf2_cmd="$(rospack find sp_core)/scripts/start_cf2.sh $id"
spawn_cmd="roslaunch sp_core _spawn_mav_id.launch mav_id:=$id"
echo -e "\e[1m$cf2_cmd\e[0m"
$cf2_cmd &
echo -e "\e[1m$spawn_cmd\e[0m"
$spawn_cmd &
done

wait
# 