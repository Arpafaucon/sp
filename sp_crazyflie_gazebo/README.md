## Note

test model 
```sh
rosrun xacro xacro models/rotors_description/urdf/crazyflie_base.xacro \
rotors_description_dir:=$(rospack find sp_crazyflie_gazebo)/models/rotors_description \
enable_ground_truth:=true enable_wind:=true namespace:=spcf color_prop_front:=Red color_prop_back:=Green
```

```sh
rosservice call /gazebo/delete_model "model_name: 'spcf'" && roslaunch sp_crazyflie_gazebo spawn_sp_crazyflie.launch && ./src/cv.py && gz sdf -p out.urdf > out.sdf && echo "-----------look-------------" && cat out.sdf | grep spot
```