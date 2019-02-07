#!/usr/bin/python
"""
static allocation server
"""

import rospy
from sp_core.msg import SwarmAllocation
# from crazyflie_driver import 

NUM_D_CONNECTED = 2
NUM_D_ACTIVE = 2

CD_ISREAL = [False, False]
CD_STATUS = [sa_msg.STATUS_ACTIVE, sa_msg.STATUS_ACTIVE]
CD_ACTIVE_ID = [0, 1]
AD_NAMESPACES = ['cf1', 'cf2']


class StaticSwarmAllocation:
    def __init__(self):
        rospy.init_node("swarm_allocation")

        self.swarm_allocation_pub = rospy.Publisher("/sp/swarm_allocation", SwarmAllocation, latch=True, queue_size=10)
        self.swarm_allocation_seq = 0

        self.rate_hz = 0.5;
        self.rate_ros = rospy.Rate(self.rate_hz)

    def _publish_swarm_allocation(self):
        sa_msg = SwarmAllocation()

        sa_msg.header.stamp = rospy.Time.now()
        sa_msg.header.seq = self.swarm_allocation_seq
        sa_msg.header.frame_id = ''
        self.swarm_allocation_seq += 1

        sa_msg.num_drones_connected = NUM_D_CONNECTED
        sa_msg.num_drones_active = NUM_D_ACTIVE
        
        sa_msg.connected_drones_isreal = CD_ISREAL
        sa_msg.connected_drones_status = CD_STATUS
        sa_msg.connected_drones_active_ids = CD_ACTIVE_ID
        # assert len(sa_msg.connected_drones_active_ids) == sa_msg.num_drones_connected

        sa_msg.active_drones_namespaces = AD_NAMESPACES

        self.swarm_allocation_pub.publish(sa_msg)
        rospy.loginfo("published allocation")

    def takeoff_drones(self):
        swarm = []
        for cf_ns in AD_NAMESPACES:



    def spin(self):
        while not rospy.is_shutdown():
            self._publish_swarm_allocation()
            self.rate_ros.sleep()


def rosmain():
    ssa = StaticSwarmAllocation()
    ssa.spin()

if __name__ == "__main__":
    rosmain()