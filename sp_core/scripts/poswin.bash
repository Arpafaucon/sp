#!/bin/bash

# script to position windows nicely for a demo

GZ=$(xdotool search --name gazebo)
echo $GZ
echo $(xdotool getwindowgeometry $GZ)

xdotool windowsize --sync $GZ 1027 661 
xdotool windowmove --sync $GZ 37 443 
xdotool windowfocus --sync $GZ
# xdotool key --window $GZ ctrl+h