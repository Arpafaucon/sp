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

QUEUE_SIZE = 5
ORDERS_TOPIC = '/sp/admiral_orders'
STATUS_TOPIC = '/sp/captain_orders'
VIZ_TOPIC = '/sp/captain_viz'
MAP_TOPIC = '/map'
RATE = 1


class Captain(object):
    """
    [summary]

    For a given `orders` message: 
    An 'association' is an array (type: int[num_drones]) that gives for each drone_id the index of the corresponding target in 'target_xs' and 'target_ys'
    therefore the drone i must go to 
        (target_xs[association[i]], target_ys[association[i]])
    """

    def __init__(self):
        rospy.init_node('sp_captain_node')

        self.orders_pub = rospy.Publisher(
            STATUS_TOPIC, CaptainOrders, queue_size=QUEUE_SIZE)

        self.viz_pub = rospy.Publisher(VIZ_TOPIC, Marker, queue_size=QUEUE_SIZE)

        self.rrt_wrapper = RosRrtWrapper(self.orders_pub, self.viz_pub)

        self.orders_sub = rospy.Subscriber(
            ORDERS_TOPIC, data_class=AdmiralOrders, callback=self.rrt_wrapper.set_orders)
        self.map_sub = rospy.Subscriber(
            MAP_TOPIC, data_class=OccupancyGrid, callback=self.rrt_wrapper.set_map)

        self.rate = rospy.Rate(RATE)
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


            # self._publish_status(results)

    # def _publish_status(self, results):
    #     stat = CaptainStatus()
    #     stat.num_drones = results.num_drones
    #     stat.drone_association = results.perm

    #     stat.total_distance = results.total_cost
    #     stat.current_xs = self.orders.current_xs
    #     stat.current_ys = self.orders.current_ys
    #     stat.target_xs = self.orders.target_xs
    #     stat.target_ys = self.orders.target_ys
    #     self.orders_pub.publish(stat)
    #     rospy.loginfo('Captain published status')

    def spin(self):
        while not rospy.is_shutdown():
            self.rrt_wrapper.spin_once_rate()


def rosmain():
    cap = Captain()
    cap.spin()


if __name__ == "__main__":
    # cProfile.run('rosmain()')
    rosmain()
