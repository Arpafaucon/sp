#!/bin/bash
while true; do
rosservice call /cf1/go_to "groupMask: 0
relative: false
goal: {x: 0.0, y: 0.0, z: 1}                      
yaw: 0.0
duration: {secs: 4, nsecs: 0}" 
sleep 1
rosservice call /cf1/go_to "groupMask: 0
relative: false
goal: {x: 1.0, y: 0.0, z: 1}                      
yaw: 0.0
duration: {secs: 4, nsecs: 0}" 
sleep 1
echo "looping"
done