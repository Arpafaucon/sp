#!/usr/bin/python
# coding: utf8
"""
High level starter for the Captain node.

The hard work is not done here, but in ros_rrt_wrapper and arrts_multi
This script merely starts the wrapper.
"""
import rospy

from ros_wrapper import RosRrtWrapper


class Captain(object):
    """
    Captain handler

    Used to be more complex, kept as a class for modularity
    """

    def __init__(self):
        self.captain_wrapper = RosRrtWrapper()
        rospy.loginfo('Captain init done')

    def spin(self):
        while not rospy.is_shutdown():
            self.captain_wrapper.spin_once_rate()


def rosmain():
    cap = Captain()
    cap.spin()


if __name__ == "__main__":
    rosmain()
