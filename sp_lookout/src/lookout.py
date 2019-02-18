#!/usr/bin/python
"""
Lookout node
"""
import math
import rospy
import numpy as np

from threading import RLock
from sp_core.tools.rospools import PublisherPool

from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from crazyflie_driver.msg import GenericLogData
from sp_core.msg import SwarmAllocation, SwarmPosition
from sp_lookout.srv import DronePosition, DronePositionRequest, DronePositionResponse, SwarmPositionSrv, SwarmPositionSrvResponse


class Lookout:
    def __init__(self):
        rospy.init_node("sp_lookout")
        # Node rate
        self.rate_hz = float(rospy.get_param('sp/lookout/rate'))
        self.rate_ros = rospy.Rate(self.rate_hz)
        # Allocation subscriber
        self.swarm_alloc_sub = rospy.Subscriber(
            '/sp/swarm_allocation', SwarmAllocation, self._cb_swarm_alloc, None)
        self.swarm_alloc_msg = None
        self.swarm_alloc_lock = RLock()
        self.swarm_pos_array = None
        self.swarm_pos_lock = RLock()

        # Position publisher
        self.swarm_pos_pub = rospy.Publisher(
            '/sp/swarm_position', SwarmPosition, queue_size=10)
        self.swarm_pos_seq = 0

        # drone position service
        self.drone_pos_srv = rospy.Service(
            "/sp/drone_position", DronePosition, self._srv_drone_position)
        # swarm position service
        self.swarm_pos_srv = rospy.Service(
            "/sp/swarm_position", SwarmPositionSrv, self._srv_swarm_position)

        # position publishers
        self.pose_pubpool = PublisherPool(PoseStamped, "pose_stamped")

    def _srv_drone_position(self, req):
        active_id = req.active_id
        with self.swarm_pos_lock:
            if self.swarm_pos_array is None:
                return None
            drone_pos = self.swarm_pos_array[active_id]
            return drone_pos

    def _srv_swarm_position(self, req):
        with self.swarm_pos_lock:
            if self.swarm_pos_array is None:
                return None
            num_drones_active = self.swarm_pos_array.shape[0]

            res = SwarmPositionSrvResponse()
            res.num_active_drones = num_drones_active
            res.x = self.swarm_pos_array[:, 0]
            res.y = self.swarm_pos_array[:, 1]
            res.z = self.swarm_pos_array[:, 2]
            res.roll = self.swarm_pos_array[:, 3]
            res.pitch = self.swarm_pos_array[:, 4]
            res.yaw = self.swarm_pos_array[:, 5]
            return res

    def _cb_swarm_alloc(self, msg):
        with self.swarm_alloc_lock:
            self.swarm_alloc_msg = msg

    def _get_drone_log(self, d_active_id):
        """
        Get active drone position

        Args:
            drone_active_id (int): id of active drone

        Returns:
            list[6*float]: array of coordinates x, y, z (meters), roll, pitch, yaw (radians)
        """
        with self.swarm_alloc_lock:
            # assert d_active_id < self.swarm_alloc_msg.num_drones_active
            d_connected_id = self.swarm_alloc_msg.active_drones_connected_ids[d_active_id]

            namespace = self.swarm_alloc_msg.connected_drones_namespaces[d_connected_id]
            position_topic = "/{}/local_position".format(namespace)
            local_position_msg = rospy.wait_for_message(
                position_topic, GenericLogData)
            coord_array = [0] * 6
            # copy x, y, z as is
            for i in range(0, 3):
                coord_array[i] = local_position_msg.values[i]
            # converts angles to radian
            for i in range(3, 6):
                coord_array[i] = local_position_msg.values[i] * math.pi / 180.
            return coord_array, namespace

    def _publish_swarm_pose_stamped(self, swarm_log):
        for drone_log in swarm_log:
            pos = drone_log[0]
            namespace = drone_log[1]
            self._pub_pose_stamped(namespace, *pos)

    def _pub_pose_stamped(self, namespace, x, y, z, roll, pitch, yaw):
        pub = self.pose_pubpool.get_pub(namespace)
        pos = PoseStamped()
        pos.header.frame_id = "/map"
        pos.header.stamp = rospy.Time.now()

        pos.pose.position.x = x
        pos.pose.position.y = y
        pos.pose.position.z = z
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pos.pose.orientation.x = qx
        pos.pose.orientation.y = qy
        pos.pose.orientation.z = qz
        pos.pose.orientation.w = qw

        pub.publish(pos)

    def _get_swarm_log(self):
        if self.swarm_alloc_msg is None:
            rospy.logwarn_throttle(
                1, "Lookout hasn't received swarm allocation. Cannot publish positions")
            return None
        swarm_log = []
        for ad_id in range(self.swarm_alloc_msg.num_drones_active):
            position, ns = self._get_drone_log(ad_id)
            swarm_log.append((position, ns))
        return swarm_log

    def _build_position_array(self, swarm_log):
        num_drones = len(swarm_log)
        positions = np.zeros((num_drones, 6))
        # x_l = []
        # y_l = []
        # z_l = []
        # roll_l = []
        # pitch_l = []
        # yaw_l = []

        for drone_id in range(num_drones):
            # index 0 is position, 1 is namespace
            drone_pos, drone_ns = swarm_log[drone_id]
            positions[drone_id] = drone_pos
        return positions

    def _publish_swarm_pos(self, swarm_pos_array):

        swarm_position_msg = SwarmPosition()
        swarm_position_msg.header.stamp = rospy.Time.now()
        swarm_position_msg.header.seq = self.swarm_pos_seq
        swarm_position_msg.header.frame_id = "/map"
        self.swarm_pos_seq += 1

        swarm_position_msg.num_drones_active = self.swarm_alloc_msg.num_drones_active
        swarm_position_msg.x = swarm_pos_array[:, 0]
        swarm_position_msg.y = swarm_pos_array[:, 1]
        swarm_position_msg.z = swarm_pos_array[:, 2]
        swarm_position_msg.roll = swarm_pos_array[:, 3]
        swarm_position_msg.pitch = swarm_pos_array[:, 4]
        swarm_position_msg.yaw = swarm_pos_array[:, 5]

        self.swarm_pos_pub.publish(swarm_position_msg)
        rospy.loginfo_throttle(
            20, "publishing swarm positions at {} hz".format(self.rate_hz))

    def spin(self):
        while (not rospy.is_shutdown()):
            swarm_log = self._get_swarm_log()
            if swarm_log is not None:
                with self.swarm_pos_lock:
                    self.swarm_pos_array = self._build_position_array(
                        swarm_log)

                self._publish_swarm_pos(self.swarm_pos_array)
                self._publish_swarm_pose_stamped(swarm_log)

            self.rate_ros.sleep()


def rosmain():
    look = Lookout()
    look.spin()


if __name__ == "__main__":
    rosmain()
