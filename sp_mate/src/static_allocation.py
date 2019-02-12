#!/usr/bin/python
"""
static allocation server
"""

import rospy
from sp_core.msg import SwarmAllocation
from crazyflie_gazebo.tools import crazyflie
from sp_mate.srv import Land, Takeoff, LandRequest, LandResponse, TakeoffRequest, TakeoffResponse, ActiveDroneInfo, ActiveDroneInfoRequest, ActiveDroneInfoResponse
import time
# from crazyflie_driver import 

NUM_D_CONNECTED = 2
NUM_D_ACTIVE = 2

CD_ISREAL = [False, False]
CD_STATUS = [SwarmAllocation.STATUS_ACTIVE, SwarmAllocation.STATUS_ACTIVE]
CD_ACTIVE_ID = [0, 1]

AD_NAMESPACES = ['cf1', 'cf2']


class StaticSwarmAllocation:
    def __init__(self):
        rospy.init_node("swarm_allocation")

        self.swarm_allocation_pub = rospy.Publisher("/sp/swarm_allocation", SwarmAllocation, latch=True, queue_size=10)
        self.swarm_allocation_seq = 0

        self.rate_hz = 0.5;
        self.rate_ros = rospy.Rate(self.rate_hz)

        self.takeoff_srv = rospy.Service("/sp_mate/takeoff", Takeoff, self.takeoff_drones)
        self.land_srv = rospy.Service("/sp_mate/land", Land, self.land_drones)
        self.activeDroneInfo_srv = rospy.Service("/sp/active_drone_info", ActiveDroneInfo, self._srv_active_drone_info)

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

    def _srv_active_drone_info(self, req):
        activeId = req.active_id
        try:
            cid = CD_ACTIVE_ID.index(activeId)
        except ValueError as _:
            return None

        res = ActiveDroneInfoResponse()
        res.connected_id =  cid
        res.is_real = CD_ISREAL[cid]
        res.namespace = AD_NAMESPACES[activeId]
        res.status = CD_STATUS[cid]
        return res

    def takeoff_drones(self, req):
        """
        TODO: Must be Thread-Safe
        """
        swarm = []
        # setup drones
        for i, cf_ns in enumerate(AD_NAMESPACES):
            # FIXME i here is not valid . This will fail if getPosition() is called
            drone = crazyflie.Crazyflie(cf_ns, i)
            drone.enableHighLevel()
            swarm.append(drone)
        # takeoff
        for drone in swarm:
            drone.takeoff(targetHeight=1.5, duration=5.)
        return TakeoffResponse(True)

    def land_drones(self, req):
        swarm = []
        # setup drones
        for i, cf_ns in enumerate(AD_NAMESPACES):
            # FIXME i here is not valid . This will fail if getPosition() is called
            drone = crazyflie.Crazyflie(cf_ns, i)
            swarm.append(drone)
        # takeoff
        for drone in swarm:
            drone.land(targetHeight=0., duration=5.)
        return LandResponse(True)

    def spin(self):
        # self.takeoff_drones()
        
        while not rospy.is_shutdown():
            self._publish_swarm_allocation()
            self.rate_ros.sleep()
        # self.land_drones()

def rosmain():
    ssa = StaticSwarmAllocation()
    ssa.spin()

if __name__ == "__main__":
    rosmain()