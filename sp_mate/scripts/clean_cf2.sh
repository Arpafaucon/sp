#!/bin/bash
rosservice call --wait /gazebo/delete_model "model_name: 'cf1'" &&
rosservice call --wait /gazebo/delete_model "model_name: 'cf2'" &&
rosservice call --wait /gazebo/delete_model "model_name: 'handler1'" &&
echo "killed all gazebo models. Killing ./cf2 instances"
killall cf2