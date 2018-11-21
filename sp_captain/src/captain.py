#!/usr/bin/python
# coding: utf8

import rospy
import threading
from sp_msgs.msg import AdmiralOrders, CaptainStatus
from nav_msgs.msg import OccupancyGrid
QUEUE_SIZE = 5
ORDERS_TOPIC = '/sp/admiral_orders'
STATUS_TOPIC = '/sp/captain_status'
MAP_TOPIC = '/map'
RATE = 1./5


class Captain(object):
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

    def _orders_callback(self, orders_msg):
        with self.orders_lock:
            self.orders = orders_msg
        # pass

    def _map_callback(self, map_msg):
        with self.map_lock:
            self.map = map_msg

    def _compute_paths(self):
        with self.orders_lock:
            with self.map_lock:
                num_drones = self.orders.num_drones
                return (True, range(num_drones), [1 for i in range(num_drones)])


    def _publish_status(self):
        if self.orders and self.map:
            stat = CaptainStatus()
            stat.num_drones = self.orders.num_drones
            ok, association, costs = self._compute_paths()
            stat.drone_association = association
            stat.distance = costs
            stat.total_distance = sum(costs)
            stat.current_xs = self.orders.current_xs
            stat.current_ys = self.orders.current_ys
            stat.target_xs = self.orders.target_xs
            stat.target_ys = self.orders.target_ys
            self.status_pub.publish(stat)


    def spin(self):
        while not rospy.is_shutdown():
            self._publish_status()
            self.rate.sleep()


def rosmain():
    cap = Captain()
    cap.spin()

if __name__ == "__main__":
    rosmain()