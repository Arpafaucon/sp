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


uri=$(rosrun sp_mate find_uri.sh $mav_id)
if [ -z "$uri" ]
then 
  >&2 echo "No drone was found. Exiting"
  exit 1
fi
echo "found uri= '$uri' "
echo "starting mix14"
read -p "press enter"
roslaunch sp_mate mix14_spawn.launch real_uri:=${uri}
