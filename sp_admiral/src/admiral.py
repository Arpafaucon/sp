#!/usr/bin/python
# coding: utf-8

"""


Image.size: width, height

array.shape: height, width
    or height, width, mode

access element in array:
array[line, column, mode]
"""


import numpy as np
import math
import itertools
import random
import rospy

# from obs_map.src.annealing
from obs_map.src.support import create_observation_map, score_generation_step, reap_state_score
from obs_map.src.ros_io import AdmiralRosInterface
from obs_map.src.annealing import SimulAnnealingOptimisation
from obs_map.src.constants import *


# image = Image.open('map2.pgm')
# print(image.format, image.size, image.mode)
# SCALE = 20
# new_format = tuple([SCALE*dim for dim in image.size])
# par = np.array(image)
# print(par.shape)
# disp_image(par, mode='L')

# print(image.getpixel((0, 0)))
# image = image.resize(new_format)
# image.show()
# print(image.getdata())

# pgmf = open('map1.pgm', encoding='ascii')
# print(read_pgm(pgmf))


def test_score(obs_map):
    h = obs_map.shape[0]
    step = 256./h
    for i in range(h):
        obs_map[i, 10,  M_SCORE] = int(i*step)
    print(obs_map[:, 10])


def doctest_fun():
    import doctest
    doctest.testmod()


def rosmain():
    # setup_node()
    # params = load_params_ROS('')
    ros_if = AdmiralRosInterface()
    params = ros_if.get_params()

    valid, obs_map = ros_if.load_map_ROS('', wall_radius=params.WALL_RADIUS)
    state = None
    if not valid:
        rospy.logerr("Couldn't get the observation map. Exiting")
        return
    rate = rospy.Rate(params.RATE)
    done = False

    optim = SimulAnnealingOptimisation(
        obs_map, params)
    while not done:
        state_f, score_f, temp = optim._simul_annealing_simple(
            state, n_iterations=params.N_ITERATIONS, temp0=params.INITIAL_TEMP)
        rospy.loginfo('converged with score {}'.format(score_f))
        state = state_f

        # so this is the 
        reap_state_score(obs_map, state, params.DRONE_SIGHT_RADIUS)
        ros_if.publish_msg(obs_map, state, params.NUM_DRONES,
                           params.N_ITERATIONS, score_f, score_f)
        # ros_if._publish_status(obs_map, params.NUM_DRONES, state_f, params.N_ITERATIONS, score_f, score_f)
        score_generation_step(obs_map)

        rate.sleep()
        if rospy.is_shutdown():
            done = True
    rospy.signal_shutdown('normal shutdown')


def main():
    obs_map = create_observation_map('map2.pgm', wall_radius=6)
    optim = SimulAnnealingOptimisation(
        obs_map, 8)
    optim._graphic_simul_annealing(None, 10, 1)


if __name__ == '__main__':
    # main()
    rosmain()
