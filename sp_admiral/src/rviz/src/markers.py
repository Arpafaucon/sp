#!/usr/bin/python
# coding : utf8
import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3
from sp_msgs.msg import AdmiralOrders
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class AdmiralOrdersRviz(object):
    def __init__(self):
        rospy.init_node('admiral_rviz')
        self.orders_viz_pub = rospy.Publisher('/sp/admiral_viz', Marker)
        self.orders_sub = rospy.Subscriber(
            '/sp/admiral_orders', AdmiralOrders, self._orders_callback)
        self.sight_radius = rospy.get_param('/sp/admiral/obs_map/drone_sight_radius')
        self.seq = 0

        self.scale = Vector3()
        self.scale.x = self.scale.y = self.scale.z = 0.05
        self.last_orders = None

    def _pub_order(self, orders, isnew=True):
        color = ColorRGBA()
        color.a = 1
        if isnew:
            color.r = 1
            marker_id = 0
        else:
            color.g = 1
            marker_id = 1

        viz = Marker()
        viz.header.stamp = rospy.Time()
        viz.header.seq = self.seq
        viz.header.frame_id = '/map'

        viz.action = viz.MODIFY
        viz.ns = 'orders'
        viz.id = marker_id
        viz.type = viz.POINTS
        viz.color = color
        viz.scale = self.scale

        for i_drone in range(orders.num_drones):
            loc = Point()
            loc.x = orders.x_coords[i_drone]
            loc.y = orders.y_coords[i_drone]
            loc.z = 0
            viz.points.append(loc)

        self.orders_viz_pub.publish(viz)

    def _pub_sight_areas(self, orders, isnew=True):
        color = ColorRGBA()
        color.a = .5
        if isnew:
            color.r = 1
            marker_id = 0
        else:
            color.g = 1
            marker_id = 1

        viz = Marker()
        viz.header.stamp = rospy.Time()
        viz.header.seq = self.seq
        viz.header.frame_id = '/map'

        viz.action = viz.MODIFY
        viz.ns = 'orders_sight'
        viz.id = marker_id
        viz.type = viz.CUBE_LIST
        viz.color = color
        viz.scale.x = 2*orders.sight_radius
        viz.scale.y = 2*orders.sight_radius
        viz.scale.z = 0.05

        for i_drone in range(orders.num_drones):
            loc = Point()
            loc.x = orders.x_coords[i_drone]
            loc.y = orders.y_coords[i_drone]
            loc.z = 0
            viz.points.append(loc)

        self.orders_viz_pub.publish(viz)

    def _orders_callback(self, orders):
        # define common components

        # print orders
        if self.last_orders:
            self._pub_order(self.last_orders, isnew=False)
            self._pub_sight_areas(self.last_orders, isnew=False)
        self._pub_order(orders, isnew=True)
        self._pub_sight_areas(orders, isnew=True)
        self.seq += 1
        self.last_orders = orders
        rospy.loginfo('published markers')

    def spin(self):
        rospy.spin()


def rosmain():
    rviz_if = AdmiralOrdersRviz()
    rviz_if.spin()


if __name__ == "__main__":
    rosmain()
