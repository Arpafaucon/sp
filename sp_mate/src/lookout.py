#!/usr/bin/python
"""
Lookout node
"""

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
        self.swarm_alloc_sub = rospy.Subscriber('sp/swarm_allocation', SwarmAllocation, self._cb_swarm_alloc, None)
        self.swarm_alloc_msg  = None
        # Position publisher
        self.swarm_pos_pub = rospy.Publisher('/sp/swarm_position', SwarmPosition, queue_size=10)
        self.swarm_pos_seq = 0

        # position service
        self.drone_pos_srv = rospy.Service("/sp/drone_position", DronePosition, self._srv_drone_location)

        # position publishers
        self.pose_pubpool = PublisherPool(PoseStamped, "/pose_stamped")

    
    def _srv_drone_location(self, req):
        x, y, z = self._get_drone_pos(req.active_id)
        res = DronePositionResponse()
        res.position = [x, y, z]
        return res


    def _cb_swarm_alloc(self, msg):
        self.swarm_alloc_msg = msg

    def _get_drone_pos(self, drone_active_id):
        # self.swarm_alloc_msg = SwarmAllocation()
        assert drone_active_id < self.swarm_alloc_msg.num_drones_active
        ad_namespace = self.swarm_alloc_msg.active_drones_namespaces[drone_active_id]
        ad_position_topic = "/{}/local_position".format(ad_namespace)
        local_position_msg = rospy.wait_for_message(ad_position_topic, GenericLogData)
        # x = local_position_msg.values[0]
        # y = local_position_msg.values[1]
        # z = local_position_msg.values[2]
        # r = local_position_msg.values[3]
        # p = local_position_msg.values[4]
        # yaw = local_position_msg.values[5]
        return local_position_msg.values, ad_namespace

    def _pub_pose_stamped(self, namespace, x, y, z, roll, pitch, yaw):
        pub = self.pose_pubpool.get_pub(namespace)
        pos = PoseStamped()
        pos.position.x = x
        pos.position.y = y
        pos.position.z = z

        pos.header.frame_id = "/map"
        pos.header.stamp = rospy.Time.now()
        pos.orientation = quaternion_from_euler(roll, pitch, yaw)

        pub.publish(pos)
    
    def  _get_swarm_pos(self):
        xl = []
        yl = []
        zl = []
        yawl = []
        swarm_pos = []
        for ad_id in range(self.swarm_alloc_msg.num_drones_active):
            position, ns = self._get_drone_pos(ad_id)
            swarm_pos.append( (position, ns))
        return swarm_pos

    def _publish_swarm_pos(self, swarm_pos):
        if self.swarm_alloc_msg is None:
            rospy.logwarn_throttle(1, "Lookout hasn't received swarm allocation. Cannot publish positions")
            return

        for drone_id in range(len(swarm_pos)):
            

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
        rospy.loginfo("published swarm positions")

    def spin(self):
        while (not rospy.is_shutdown()):
            self._publish_swarm_pos()
            self.rate_ros.sleep()

def rosmain():
    look = Lookout()
    look.spin()

if __name__ == "__main__":
    rosmain()