#!/bin/bash

bold=$(tput bold)
normal=$(tput sgr0)

function wait4go()
{
    # $1=${1:-DEFAULTVALUE}
    echo
    echo "${bold}$1${normal}  [Press <Enter> to run command below]"
    read -p "> $2"
    eval $2
}

echo "${bold} SP demonstration routine ${normal}";
echo "------------------------------"
echo 


wait4go "Request 3 active drones" "rosservice call /sp/active_target 3"
wait4go "Register drone #1 as faulty" "rosservice call /sp/set_drone_state '{connected_id: 1, state: 2}'"
wait4go "Register drone #1 as repaired" "rosservice call /sp/set_drone_state '{connected_id: 1, state: 1}'"
wait4go "Start planification" "rosservice call /sp/start_admiral True"
wait4go "Register drone #0 as faulty" "rosservice call /sp/set_drone_state '{connected_id: 0, state: 2}'"
wait4go "Limit to 1 active drone" "rosservice call /sp/active_target 1"
wait4go "Land last active drone" "rosservice call /sp/active_target 0"
# wait4go "Stop planification" "rosservice call /sp/start_admiral False"

# rosservice call /gazebo/unpause_physics;

# wait4go "Takeoff active drones";
# # -1 is ACTIVE
# rosservice call /sp/takeoff "id: -1";

# wait4go "Start order chain";
# rosservice call /sp/start_admiral "run: true";