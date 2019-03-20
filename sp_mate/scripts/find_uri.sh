#!/bin/bash

# call like following:
# $ find_uri.sh 1

if [ $# -eq 0 ]
  then
    >&2 echo "No arguments supplied."
    >&2 echo "Call like:"
    >&2 echo "$0 <drone_id>"
    exit 1
fi

mav_id=$1

mav_address=$(printf "0xE7E7E7E7%02d" $mav_id)
# mav_address="0xE7E7E7E70${mav_id}"
>&2 echo "searching for : ${mav_address}"
cmd="rosrun crazyflie_tools scan --address ${mav_address}"
>&2 echo $cmd
$cmd
