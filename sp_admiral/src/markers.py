#!/usr/bin/python
# coding : utf8
import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3
from sp_admiral.msg import AdmiralOrders
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

ADM_ORDERS_SUB = '/sp/admiral_orders'
ADM_VIZ_PUB = '/sp/admiral_viz'


def extract_coords(orders, target=True):
    if target:
        return orders.target_xs, orders.target_ys
    return orders.current_xs, orders.current_ys

MARKERS_LIFETIME = rospy.Duration(secs=15)
COL_ALPHA = .5

COL_CURRENT = ColorRGBA()
COL_CURRENT.a = COL_ALPHA
COL_CURRENT.r = 1

COL_TARGET = ColorRGBA()
COL_TARGET.a = COL_ALPHA
COL_TARGET.g = 1

CONE_ALTITUDE = 1
CYL_HEIGHT = .05



class AdmiralOrdersRviz(object):
    def __init__(self):
        rospy.init_node('admiral_rviz')
        self.orders_viz_pub = rospy.Publisher(
            ADM_VIZ_PUB, Marker, queue_size=5)
        self.orders_sub = rospy.Subscriber(
            ADM_ORDERS_SUB, AdmiralOrders, self._orders_callback, queue_size=5)
        self.sight_radius = rospy.get_param('/sp/admiral/drone_sight_radius')
        self.pm_altitude = rospy.get_param('/sp/mate/altitude', CONE_ALTITUDE)
        self.seq = 0

        self.scale = Vector3()
        self.scale.x = self.scale.y = self.scale.z = 0.05

        rospy.loginfo("Admiral markers set up")
        # self.last_orders = None

    def _pub_order(self, orders, target=True):
        color = ColorRGBA()
        if target:
            color = COL_TARGET
            ns = "orders.target"
        else:
            color = COL_CURRENT
            ns = "orders.current"
            

        viz = Marker()
        viz.header.stamp = rospy.Time()
        viz.header.seq = self.seq
        viz.header.frame_id = '/map'

        viz.action = viz.MODIFY
        viz.ns = ns
        viz.id = 0
        viz.type = viz.POINTS
        viz.color = color
        viz.scale = self.scale
        viz.lifetime = MARKERS_LIFETIME

        x_coords, y_coords = extract_coords(orders, target=target)

        for i_drone in range(orders.num_drones):
            loc = Point()
            loc.x = x_coords[i_drone]
            loc.y = y_coords[i_drone]
            loc.z = 0
            viz.points.append(loc)

        rospy.loginfo("pub {} points".format(len(viz.points)))
        self.orders_viz_pub.publish(viz)

    @staticmethod
    def active_id_to_letter(aid):
        assert 0 <= aid < 26
        return chr(ord('A') + aid)

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
        viz.scale.z = .2
        viz.lifetime = MARKERS_LIFETIME

        x_coords, y_coords = extract_coords(orders, target=False)
        for i_drone in range(orders.num_drones):
            viz.id = i_drone
            viz.text = AdmiralOrdersRviz.active_id_to_letter(i_drone)
            viz.pose.position.x = x_coords[i_drone]
            viz.pose.position.y = y_coords[i_drone]
            # viz.pose.position.z = 0
            self.orders_viz_pub.publish(viz)

    

    def _pub_sight_areas_arrow(self, orders, target=True):
        """
        Publish sight areas as cones

        FIXME: for an unknown reason, arrow do not care about alpha parameter
        """
        color = ColorRGBA()
        if target:
            ns = 'sight.target.cone'
            color = COL_TARGET
        else:
            ns = 'sight.current.cone'
            color = COL_CURRENT

        x_coords, y_coords = extract_coords(orders, target=target)
        
        viz = Marker()
        viz.header.stamp = rospy.Time()
        viz.header.frame_id = '/map'

        viz.action = viz.MODIFY
        viz.ns = ns
        viz.type = viz.ARROW
        viz.color = color
        viz.scale.x = 0
        viz.scale.y = 2*orders.sight_radius
        viz.scale.z = self.pm_altitude
        viz.lifetime = MARKERS_LIFETIME

        for i_drone in range(orders.num_drones):
            viz.id = i_drone
            viz.points = []
            base = Point()
            base.x = x_coords[i_drone]
            base.y = y_coords[i_drone]
            base.z = 0
            viz.points.append(base)
            tip = Point()
            tip.x = x_coords[i_drone]
            tip.y = y_coords[i_drone]
            tip.z = self.pm_altitude
            viz.points.append(tip)

            self.orders_viz_pub.publish(viz)

    def _pub_sight_areas_cylinder(self, orders, target=True):
        """
        Publish sight areas as cylinders
        """
        color = ColorRGBA()
        if target:
            ns = 'sight.target.cyl'
            color = COL_TARGET
        else:
            ns = 'sight.current.cyl'
            color = COL_CURRENT

        x_coords, y_coords = extract_coords(orders, target=target)
        
        viz = Marker()
        viz.header.stamp = rospy.Time()
        viz.header.frame_id = '/map'

        viz.action = viz.MODIFY
        viz.ns = ns
        viz.type = viz.CYLINDER
        viz.color = color
        viz.scale.x = 2*orders.sight_radius
        viz.scale.y = 2*orders.sight_radius
        viz.scale.z = CYL_HEIGHT
        viz.lifetime = MARKERS_LIFETIME


        for i_drone in range(orders.num_drones):
            viz.id = i_drone
            viz.pose.position.x = x_coords[i_drone]
            viz.pose.position.y = y_coords[i_drone]
            viz.pose.position.z = CYL_HEIGHT/2.

            self.orders_viz_pub.publish(viz)


    def _pub_sight_areas(self, orders, target=True):
        """
        Publish sight areas as squares
        """
        color = ColorRGBA()
        if target:
            color = COL_TARGET
            marker_id = 0
        else:
            color = COL_CURRENT
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
        viz.lifetime = MARKERS_LIFETIME

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
        rospy.logwarn("viz got admiral orders")
        # print orders
        # if self.last_orders:
        #     self._pub_order(self.last_orders, target=False)
        #     self._pub_sight_areas(self.last_orders, target=False)
        self._pub_order(orders, target=True)
        self._pub_order(orders, target=False)
        self._pub_sight_areas_arrow(orders, target=True)
        self._pub_sight_areas_arrow(orders, target=False)
        self._pub_sight_areas_cylinder(orders, target=True)
        self._pub_sight_areas_cylinder(orders, target=False)
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
