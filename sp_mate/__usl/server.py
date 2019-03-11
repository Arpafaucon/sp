#!/usr/bin/python
import rospy
import roslaunch
import os
import rospkg
import time
import sys
import subprocess
import select

from typing import Union

x = 1  # type: Union[int, str]

def start_cf_server(launch):
    package = 'crazyflie_gazebo'
    executable = 'crazyflie_server'

    node = roslaunch.core.Node(package=package, node_type=executable,
                               name=executable, namespace='/sp', output='screen')

    process = launch.launch(node)
    return process


def start_cf2_engine(launch, port):
    package = 'sp_mate'
    executable = 'start_cf2.sh'
    args = "{} INADDR_ANY".format(19950+port)

    node = roslaunch.core.Node(package=package, node_type=executable,
                               name=executable, namespace='/sp', output='screen', args=args)

    process = launch.launch(node)
    return process
    # # path from crazyflie_gazebo
    # cf2_relative_path = "../crazyflie-firmware/sitl_make/build/cf2"

    # port_num = (19950 + port)

    # rospack = rospkg.RosPack()
    # craz_gazebo_path = rospack.get_path('crazyflie_gazebo')
    # abs_path = "{}/{}".format(craz_gazebo_path, cf2_relative_path)
    # command = [abs_path, str(port_num), "INADDR_ANY"]
    # print abs_path
    # print os.path.exists(abs_path)
    # print "launching command ", " ".join(command)
    # process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    # process = subprocess.Popen(['ping', '127.0.0.1', '-c', '2'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    # print "started, pid=", process.pid
    # while(True):
    #     retcode = process.poll()
    #     line = process.stdout.readline()
    #     if line:
    #         print line

    #     if retcode is not None:
    #         break
    # # while not process.poll():
        
    # # time.sleep(5)
    # process.wait()
    # print "killed"
    # for line in process.stdout:
    #     print line
    #     # sys.stdout.write(line)
    # for line in process.stderr:
    #     # sys.stdout.write(line)
    #     print line


def init_launch():
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    return launch


def rosmain():
    launch = init_launch()
    # cf_server_ps = start_cf_server(launch)
    # print cf_server_ps.is_alive()
    start_cf2_engine(launch, 0)

    # cf_server_ps.stop()
    time.sleep(10)
    launch.stop()


if __name__ == "__main__":
    rosmain()
    # print os.getcwd()
