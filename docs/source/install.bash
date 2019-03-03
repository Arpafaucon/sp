#!/bin/bash
##
# SP setup procedure
##

# SYSTEM DEPS
sudo apt install git python-pip python-tk

# ROS INSTALLATION
# on ubuntu 18
# follow the repo installation procedure from ROS website
sudo apt install ros-melodic-ros-base
sudo apt install ros-melodic-rviz ros-melodic-gazebo-ros ros-melodic-xacro ros-melodic-map-server
# equivalent to catkin_make, but cleaner (and faster)
sudo apt install python-catkin-tools
# additional deps for wuwushrek/sim_cf
sudo apt install libopencv-dev ros-melodic-cv-bridge
# Don't forget to init rosdep if doing a fresh install

# PYTHON DEPS
pip install --user matplotlib numpy typing tkinter

# Workspace
source /opt/ros/melodic/setup.bash
mkdir -p rosws/src
cd rosws
catkin init
cd src

# crazyflie_ros
git clone https://github.com/whoenig/crazyflie_ros.git
cd crazyflie_ros
git submodule update --init --recursive
cd ..

# ===========
# sim_cf
git clone https://github.com/wuwushrek/sim_cf.git
#f ollow installation instructions from README
# currently:
# 
# Basic dependencies
sudo apt-get install git zip qtcreator cmake build-essential genromfs ninja-build -y
# Protobuf , eigen3 and google-glog dependencies
sudo apt-get install protobuf-compiler libgoogle-glog-dev libeigen3-dev libxml2-utils
cd sim_cf/
git submodule update --init --recursive
# build sitl
cd crazyflie-firmware/sitl-make
mkdir build
cd build
cmake ..
make
cd ../../../../ # back to src/

# sp
git clone https://github.com/Arpafaucon/sp.git

catkin build
# re-source ROS setup after changes
source /opt/ros/melodic/setup.bash
source ~/rosws/devel/setup.bash
