#!/usr/bin/python
# coding: utf8

import rospy
import threading
import math

from sp_core.msg import AdmiralOrders, CaptainStatus
from nav_msgs.msg import OccupancyGrid
QUEUE_SIZE = 5
ORDERS_TOPIC = '/sp/admiral_orders'
STATUS_TOPIC = '/sp/captain_status'
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
        self.map = None
        self.map_lock = threading.RLock()
        self.orders = None
        self.orders_lock = threading.RLock()

        rospy.init_node('sp_captain_node')
        self.orders_sub = rospy.Subscriber(
            ORDERS_TOPIC, data_class=AdmiralOrders, callback=self._orders_callback)
        self.map_sub = rospy.Subscriber(
            MAP_TOPIC, data_class=OccupancyGrid, callback=self._map_callback)
        self.status_pub = rospy.Publisher(
            STATUS_TOPIC, CaptainStatus, queue_size=QUEUE_SIZE)

        self.rate = rospy.Rate(RATE)
        rospy.loginfo('Captain init done')

    def _orders_callback(self, orders_msg):
        with self.orders_lock:
            self.orders = orders_msg
        # pass

    def _map_callback(self, map_msg):
        with self.map_lock:
            self.map = map_msg

    def _compute_distance(self, orders, association):
        num_drones = orders.num_drones
        #first compute the target for each drone
        ass_target_xs = [orders.target_xs[association[i]] for i in range(num_drones)]
        ass_target_ys = [orders.target_ys[association[i]] for i in range(num_drones)]

        dist_x2 = [(ass_target_xs[i] - orders.current_xs[i])**2 for i in range(num_drones)]
        dist_y2 = [(ass_target_ys[i] - orders.current_ys[i])**2 for i in range(num_drones)]
        
        dist = [math.sqrt(dx2+dy2) for dx2, dy2 in zip(dist_x2, dist_y2)]
        total = sum(dist)
        return dist, total

    

    def _compute_paths(self):
        with self.orders_lock:
            with self.map_lock:
                num_drones = self.orders.num_drones
                association = range(num_drones)
                dist, total = self._compute_distance(self.orders, association)
                return (True, association, dist, total)


    def _publish_status(self):
        if self.orders and self.map:
            stat = CaptainStatus()
            stat.num_drones = self.orders.num_drones
            ok, association, costs, total = self._compute_paths()
            stat.drone_association = association
            stat.distance = costs
            stat.total_distance = total
            stat.current_xs = self.orders.current_xs
            stat.current_ys = self.orders.current_ys
            stat.target_xs = self.orders.target_xs
            stat.target_ys = self.orders.target_ys
            self.status_pub.publish(stat)
            rospy.loginfo('Captain published status')


    def spin(self):
        while not rospy.is_shutdown():
            self._publish_status()
            self.rate.sleep()


def rosmain():
    cap = Captain()
    cap.spin()

if __name__ == "__main__":
    rosmain()