#!/usr/bin/python
# coding: utf-8

"""
Admiral Node
------------

Top-level planification node

Optimisation intelligence is in `annealing.py`
ROS interfacing in `ros_io.py`

------------
Conventions on data access

Image.size: width, height

array.shape: height, width
    or height, width, mode

access element in array:
array[line, column, mode]
"""


import rospy

from obs_map.support import score_generation_step, reap_state_score
from obs_map.ros_io import AdmiralRosInterface
from obs_map.annealing import SimulAnnealingOptimisation

def rosmain():
    """
    Main ROS procedure
    Load data, then periodically performs state optimisation and dispatches orders
    """
    ros_if = AdmiralRosInterface()

    # blocking wait 
    # return false if node must shutdown
    if not ros_if.wait_for_go():
        rospy.loginfo("admiral was shut down while waiting to start")
        return

    # get the map and params
    obs_map, params = ros_if.init_map_params()

    done = False
    first = True

    rate = rospy.Rate(params.RATE)
    optim = SimulAnnealingOptimisation(
        obs_map, params)

    state_measured, num_drones = ros_if.get_drone_positions()
    optim.set_num_drones(num_drones)

    state_target = optim.init_pos_drones(state_measured)
    state_curr = state_target

    while not done and not rospy.is_shutdown():
        # get current drone state
        # that would be the point where we update the number of drones as well
        # here : simplified : current state is last target state
        state_curr, num_drones = ros_if.get_drone_positions()

        if num_drones > 0:
            # there is something to optimize...

            # we update score map taking into account current state (score at t0)
            # if not first:
            reap_state_score(obs_map, state_curr, params.DRONE_SIGHT_RADIUS)
                # first = False
            # map is updated to next target time (scores of t0+T)
            score_generation_step(obs_map, params.SCORE_STEP_INCREMENT)

            # run optimisation to find target for t0+T
            state_target, score_target, temp = optim.simul_annealing_simple(
                state_curr, n_iterations=params.N_ITERATIONS, temp0=params.INITIAL_TEMP)
            rospy.loginfo('converged with score {}'.format(score_target))

            # publish target
            ros_if.publish_msgs(obs_map, state_curr, state_target, num_drones,
                                params.N_ITERATIONS, score_target, score_target)

        # ... and sleep until t0+T
        rate.sleep()

    rospy.loginfo('admiral was shut down gracefully')


if __name__ == '__main__':
    # main()
    rosmain()
