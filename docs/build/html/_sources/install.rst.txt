Install
=======

Technical requirements and supported versions
---------------------------------------------

The project has been developped and tested for both:

* Ubuntu 16.04 and ROS Kinetic [Lab PC: 40 CPUs, 64Go RAM, 1To SSD]
* Ubuntu 18.04 and ROS Melodic [Personal PC: 8 CPUs, 16Go RAM, 128To SSD]

.. note:: ROS Kinetic being shipped with Gazebo 7, attaching lights to models is not supported in that setup. Gazebo 7 will gracefully ignore the light source. To allow consistency, an ``enable_light`` parameter is present to explicitely disable the light sources in Melodic.

The project isn't particularly ressource heavy, most of the processing power was taken by the Gazebo physics engine.
All operating rates for ``sp`` nodes are defined as parameters, and can be adjusted if needed.


Overview
--------
The following tools/packages are required

* a working ROS base installation (see http://wiki.ros.org/melodic/Installation/Ubuntu)
* RVIZ
* git



Additionnal python modules:

* typing (for static code checks)
* numpy

Other ROS packages:

* wuwushrek/sim_cf
* whoenig/crazyflie_ros


Install script
---------------

The following install script was tested on a fresh Ubuntu install.

.. literalinclude:: install.bash





