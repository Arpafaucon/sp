#!/bin/bash

newpane="tmux split-window -h"
# start server
$newpane "roslaunch crazyflie_driver crazyflie_server.launch"
# start 4 simulation engines
$newpane "rosrun crazyflie_gazebo run_cfs.sh 4 19950 INADDR_ANY"
# start 4 simulation ROS links (topics + gazebo)
$newpane "roslaunch sp_mate sim_4_cf.launch"