#!/usr/bin/python
# coding: utf8

import rospy
import threading
import math
import cProfile
import pstats
import StringIO

from sp_core.msg import AdmiralOrders, CaptainOrders
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

from ros_rrt_wrapper import RosRrtWrapper




class Captain(object):
    """
    [summary]

    For a given `orders` message: 
    An 'association' is an array (type: int[num_drones]) that gives for each drone_id the index of the corresponding target in 'target_xs' and 'target_ys'
    therefore the drone i must go to 
        (target_xs[association[i]], target_ys[association[i]])
    """

    def __init__(self):
        

        self.rrt_wrapper = RosRrtWrapper()

        rospy.loginfo('Captain init done')

    def _compute_paths(self):
        res = dict()
        pr = cProfile.Profile()

        # cProfile.runctx("res['success'], res['results'] = wrap.planif_assign()", globals(), {'wrap':self.rrt_wrapper, 'res' : res}, sort='tottime')

        # success = res['success']
        # results = res['results']
        # if res['success']:
        #     exit()
        pr.enable()
        success, results = self.rrt_wrapper._planif_assign()
        pr.disable()

        if success:
            s = StringIO.StringIO()
            ps= pstats.Stats(pr, stream=s).sort_stats('tottime')
            ps.print_stats(10)
            # print(s.getvalue())
            # exit()

    def spin(self):
        while not rospy.is_shutdown():
            self.rrt_wrapper.spin_once_rate()


def rosmain():
    cap = Captain()
    cap.spin()


if __name__ == "__main__":
    # cProfile.run('rosmain()')
    rosmain()
