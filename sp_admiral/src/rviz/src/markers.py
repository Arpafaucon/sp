#!/usr/bin/python
# coding : utf8
import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3
from sp_core.msg import AdmiralOrders
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def extract_coords(orders, target=True):
    if target:
        return orders.target_xs, orders.target_ys
    return orders.current_xs, orders.current_ys


class AdmiralOrdersRviz(object):
    def __init__(self):
        rospy.init_node('admiral_rviz')
        self.orders_viz_pub = rospy.Publisher('/sp/admiral_viz', Marker, queue_size=5)
        self.orders_sub = rospy.Subscriber(
            '/sp/admiral_orders', AdmiralOrders, self._orders_callback, queue_size=5)
        self.sight_radius = rospy.get_param('/sp/admiral/drone_sight_radius')
        self.seq = 0

        self.scale = Vector3()
        self.scale.x = self.scale.y = self.scale.z = 0.05
        # self.last_orders = None

    def _pub_order(self, orders, target=True):
        color = ColorRGBA()
        color.a = 1
        if target:
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

        x_coords, y_coords = extract_coords(orders, target=target)

        for i_drone in range(orders.num_drones):
            loc = Point()
            loc.x = x_coords[i_drone]
            loc.y = y_coords[i_drone]
            loc.z = 0
            viz.points.append(loc)

        self.orders_viz_pub.publish(viz)

    def _pub_drone_ids(self, orders):
        color = ColorRGBA()
        color.a = color.r = color.g = color.b = 1

        viz = Marker()
        viz.header.stamp = rospy.Time()
        viz.header.seq = self.seq
        viz.header.frame_id = '/map'

        viz.action = viz.MODIFY
        viz.ns = 'drone_ids'
        viz.type = viz.TEXT_VIEW_FACING
        viz.color = color
        viz.scale.z = .1

        x_coords, y_coords = extract_coords(orders, target=False)
        for i_drone in range(orders.num_drones):
            viz.id = i_drone
            viz.text = str(i_drone)
            viz.pose.position.x = x_coords[i_drone]
            viz.pose.position.y = y_coords[i_drone]
            # viz.pose.position.z = 0
            self.orders_viz_pub.publish(viz)

    def _pub_sight_areas(self, orders, target=True):
        color = ColorRGBA()
        color.a = .5
        if target:
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

        x_coords, y_coords = extract_coords(orders, target=target)

        for i_drone in range(orders.num_drones):
            loc = Point()
            loc.x = x_coords[i_drone]
            loc.y = y_coords[i_drone]
            loc.z = 0
            viz.points.append(loc)

        self.orders_viz_pub.publish(viz)

    def _orders_callback(self, orders):
        # define common components

        # print orders
        # if self.last_orders:
        #     self._pub_order(self.last_orders, target=False)
        #     self._pub_sight_areas(self.last_orders, target=False)
        self._pub_order(orders, target=True)
        self._pub_order(orders, target=False)
        self._pub_sight_areas(orders, target=True)
        self._pub_sight_areas(orders, target=False)
        self._pub_drone_ids(orders)
        self.seq += 1
        # self.last_orders = orders
        rospy.loginfo('published markers')

    def spin(self):
        rospy.spin()


def rosmain():
    rviz_if = AdmiralOrdersRviz()
    rviz_if.spin()


if __name__ == "__main__":
    rosmain()
