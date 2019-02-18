#!/usr/bin/python
"""
Lookout node
"""
import math
import rospy

from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from sp_core.msg import SwarmAllocation, SwarmPosition
from sp_mate.srv import DronePosition, DronePositionRequest, DronePositionResponse
from crazyflie_driver.msg import GenericLogData

from rospools import PublisherPool


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
        # Position publisher
        self.swarm_pos_pub = rospy.Publisher(
            '/sp/swarm_position', SwarmPosition, queue_size=10)
        self.swarm_pos_seq = 0

        # position service
        self.drone_pos_srv = rospy.Service(
            "/sp/drone_position", DronePosition, self._srv_drone_location)

        # position publishers
        self.pose_pubpool = PublisherPool(PoseStamped, "pose_stamped")

    def _srv_drone_location(self, req):
        pos, namespace= self._get_drone_pos(req.active_id)
        res = DronePositionResponse()
        res.position = pos[0:4]
        return res

    def _cb_swarm_alloc(self, msg):
        self.swarm_alloc_msg = msg

    def _get_drone_pos(self, drone_active_id):
        """
        Get active drone position
        
        Args:
            drone_active_id (int): id of active drone
        
        Returns:
            list[6*float]: array of coordinates x, y, z (meters), roll, pitch, yaw (radians)
        """
        # self.swarm_alloc_msg = SwarmAllocation()
        assert drone_active_id < self.swarm_alloc_msg.num_drones_active
        ad_namespace = self.swarm_alloc_msg.active_drones_namespaces[drone_active_id]
        ad_position_topic = "/{}/local_position".format(ad_namespace)
        local_position_msg = rospy.wait_for_message(
            ad_position_topic, GenericLogData)
        coord_array = [0] * 6
        # copy x, y, z as is
        for i in range(0, 3):
            coord_array[i] = local_position_msg.values[i]
        # converts angles to radian
        for i in range(3, 6):
            coord_array[i] =  local_position_msg.values[i] * math.pi / 180.
        # x = local_position_msg.values[0]
        # y = local_position_msg.values[1]
        # z = local_position_msg.values[2]
        # r = local_position_msg.values[3]
        # p = local_position_msg.values[4]
        # yaw = local_position_msg.values[5]
        print coord_array
        return coord_array, ad_namespace

    def _publish_swarm_pose_stamped(self, swarm_pos):
        for drone_pos in swarm_pos:
            pos = drone_pos[0]
            namespace = drone_pos[1]
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

    def _get_swarm_pos(self):
        if self.swarm_alloc_msg is None:
            rospy.logwarn_throttle(
                1, "Lookout hasn't received swarm allocation. Cannot publish positions")
            return None
        swarm_pos = []
        for ad_id in range(self.swarm_alloc_msg.num_drones_active):
            position, ns = self._get_drone_pos(ad_id)
            swarm_pos.append((position, ns))
        return swarm_pos

    def _publish_swarm_pos(self, swarm_pos):
        
        xl = []
        yl = []
        zl = []
        yawl = []
        for drone_id in range(len(swarm_pos)):
            pos = swarm_pos[drone_id][0]
            xl.append(pos[0])
            yl.append(pos[1])
            zl.append(pos[2])
            yawl.append(pos[3])

        swarm_position_msg = SwarmPosition()
        swarm_position_msg.header.stamp = rospy.Time.now()
        swarm_position_msg.header.seq = self.swarm_pos_seq
        swarm_position_msg.header.frame_id = "/map"
        self.swarm_pos_seq += 1

        swarm_position_msg.num_drones_active = self.swarm_alloc_msg.num_drones_active
        swarm_position_msg.x = xl
        swarm_position_msg.y = yl
        swarm_position_msg.z = zl
        swarm_position_msg.yaw = yl

        self.swarm_pos_pub.publish(swarm_position_msg)
        rospy.loginfo_throttle(1, "published swarm positions")

    def spin(self):
        while (not rospy.is_shutdown()):
            swarm_pos = self._get_swarm_pos()
            if swarm_pos is not None:
                self._publish_swarm_pos(swarm_pos)
                self._publish_swarm_pose_stamped(swarm_pos)

            self.rate_ros.sleep()


def rosmain():
    look = Lookout()
    look.spin()


if __name__ == "__main__":
    rosmain()
