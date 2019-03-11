#!/usr/bin/python
# coding: utf8

from ros_rrt_wrapper import RosRrtWrapper

class Captain(object):
    """
    For a given `orders` message: 
    An 'association' is an array (type: int[num_drones]) that gives for each drone_id the index of the corresponding target in 'target_xs' and 'target_ys'
    therefore the drone i must go to 
        (target_xs[association[i]], target_ys[association[i]])
    """

    def __init__(self):
        self.rrt_wrapper = RosRrtWrapper()
        rospy.loginfo('Captain init done')

    def spin(self):
        while not rospy.is_shutdown():
            self.rrt_wrapper.spin_once_rate()


def rosmain():
    cap = Captain()
    cap.spin()


if __name__ == "__main__":
    rosmain()
