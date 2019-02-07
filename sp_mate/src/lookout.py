#!/usr/bin/python
"""
Lookout node
"""

import rospy
from sp_core.msg import SwarmAllocation, SwarmPosition
from crazyflie_driver.msg import GenericLogData


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


    def _cb_swarm_alloc(self, msg):
        self.swarm_alloc_msg = msg

    def _get_drone_pos(self, drone_active_id):
        # self.swarm_alloc_msg = SwarmAllocation()
        assert drone_active_id < self.swarm_alloc_msg.num_drones_active
        ad_namespace = self.swarm_alloc_msg.active_drones_namespaces[drone_active_id]
        ad_position_topic = "/{}/local_position".format(ad_namespace)
        local_position_msg = rospy.wait_for_message(ad_position_topic, GenericLogData)
        x = local_position_msg.values[0]
        y = local_position_msg.values[1]
        z = local_position_msg.values[2]
        # r = local_position_msg.values[3]
        # p = local_position_msg.values[4]
        # y = local_position_msg.values[5]
        return x, y, z
    
    def  _get_swarm_pos(self):
        xl = []
        yl = []
        zl = []
        for ad_id in range(self.swarm_alloc_msg.num_drones_active):
            x, y, z = self._get_drone_pos(ad_id)
            xl.append(x)
            yl.append(y)
            zl.append(z)
        return xl, yl, zl

    def _publish_swarm_pos(self):
        if self.swarm_alloc_msg is None:
            rospy.logwarn_throttle(1, "Lookout hasn't received swarm allocation. Cannot publish positions")
            return

        xl, yl, zl = self._get_swarm_pos()
        swarm_position_msg = SwarmPosition()
        swarm_position_msg.header.stamp = rospy.Time.now()
        swarm_position_msg.header.seq = self.swarm_pos_seq
        swarm_position_msg.header.frame_id = "/map"
        self.swarm_pos_seq += 1

        swarm_position_msg.num_drones_active = self.swarm_alloc_msg.num_drones_active
        swarm_position_msg.x = xl
        swarm_position_msg.y = yl
        swarm_position_msg.z = zl

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