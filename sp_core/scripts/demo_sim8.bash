#!/bin/bash

bold=$(tput bold)
normal=$(tput sgr0)
un_s=$(tput smul)
un_e=$(tput rmul)

h0=$(tput setaf 1)
h1=$(tput setaf 4)
h2=$(tput setaf 2)



function wait4go()
{
    # $1=${1:-DEFAULTVALUE}
    echo
    echo "${bold}${h1}$1${normal}  [Press <Enter> to run command below]"
    read -p "> $2"
    eval $2
}

function title(){
    echo
    echo "${bold}${h2} ${un_s}$1${un_e} ${normal}";
    echo
}

echo        "${h0}============================${normal}"
echo "${bold}${h0}| SP demonstration routine |${normal}";
echo "${bold}${h0}|       SIM - 8 drones     |${normal}";
echo        "${h0}============================${normal}"

# title "setup"


title "static"

wait4go "Request 6 active drones" "rosservice call /sp/active_target 6"
wait4go "Register drone #1 as faulty" "rosservice call /sp/set_drone_state '{connected_id: 0, state: 2}'"
wait4go "Register drone #2 as faulty" "rosservice call /sp/set_drone_state '{connected_id: 1, state: 2}'"

title "repairing faults"
wait4go "Register drone #2 as repaired" "rosservice call /sp/set_drone_state '{connected_id: 1, state: 1}'"

title "dynamic"

wait4go "Start planification" "rosservice call /sp/start_admiral True"
wait4go "Register drone #5 as faulty" "rosservice call /sp/set_drone_state '{connected_id: 4, state: 2}'"

title "shutdown"

wait4go "Limit to 2 active drones" "rosservice call /sp/active_target 2"
wait4go "Land last active drones" "rosservice call /sp/active_target 0"
