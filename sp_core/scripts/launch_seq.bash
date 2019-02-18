#!/bin/bash

bold=$(tput bold)
normal=$(tput sgr0)

function wait4go()
{
    # $1=${1:-DEFAULTVALUE}
    read -p "${bold} $1 ${normal}  [Press Enter]"
}

echo "${bold} SP starting routine ${normal}";
wait4go "Unpause physics";
rosservice call /gazebo/unpause_physics;

wait4go "Takeoff active drones";
# -1 is ACTIVE
rosservice call /sp/takeoff "id: -1";

wait4go "Start order chain";
rosservice call /sp/start_admiral "run: true";