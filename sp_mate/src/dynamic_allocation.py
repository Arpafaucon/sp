#!/usr/bin/python3
"""
static allocation server
"""

import rospy
from sp_core.msg import SwarmAllocation, SwarmPosition
from crazyflie_gazebo.tools import crazyflie
from sp_mate.srv import Land, TakeOff, LandRequest, LandResponse, TakeOffRequest, TakeOffResponse, ActiveDroneInfo, ActiveDroneInfoRequest, ActiveDroneInfoResponse
import time
from drone_state_machine import MODE, STATE, DroneStateMachine, DroneAllocation


PUB_ALLOCATION = "/sp/swarm_allocation"
SUB_POSITION = "/sp/swarm_position"


class DynamicAllocation(object):
    """
    assumptions:
    - drone namespaces are 'cf<i>' , i starting from 1. So drone 0 has namespace 'cf1'
    """
    @staticmethod
    def namespace(drone_id):
        return "/cf{}".format(drone_id+1)

    def __init__(self):

        # ROS INIT
        rospy.init_node("dynamic_allocation")

        # publishers:
        self.swarm_allocation_pub = rospy.Publisher(
            PUB_ALLOCATION, SwarmAllocation, latch=True, queue_size=10)

        # subscribers
        self.swarm_position_sub = rospy.Subscriber(
            SUB_POSITION, SwarmPosition, self.cb_swarm_position)

        # Params
        self.pm_num_connected_drones = int(
            rospy.get_param("/sp/num_drones_total"))
        self.pm_num_active_drones = int(
            rospy.get_param("/sp/num_drones_active"))
        self.pm_altitude = float(rospy.get_param("/sp/altitude"))
        self.pm_rate = float(rospy.get_param("/sp/allocation_rate"))
        self.rate_ros = rospy.Rate(self.pm_rate)

        # Class
        self.ms_machine = DroneStateMachine.new(self.pm_num_connected_drones)
        self.swarm = None

        # results
        self.last_da = None

    def spin(self):
        while not rospy.is_shutdown():
            allocation = self.ms_machine.spin()
            self.pub_allocation(allocation)
            self.rate_ros.sleep()

    def _init_high_level(self):
        # setup drones
        self.swarm = []
        for cid in range(NUM_D_CONNECTED):
            cd_namespace = self.namespace(cid)
            # FIXME: second argument is weird
            drone = crazyflie.Crazyflie(cd_namespace, cid)
            drone.enableHighLevel()
            self.swarm.append(drone)

    def cb_swarm_position(self, msg: SwarmPosition):
        pass

    def cb_mode_changed(self, drone_index: int, old_mode: MODE, new_mode: MODE):
        pass

    def cb_state_changed(self, drone_index: int, old_state: STATE, new_state: STATE):
        pass

    def pub_allocation(self, allocation: DroneAllocation):
        sa_msg = SwarmAllocation()

        sa_msg.header.stamp = rospy.Time.now()
        sa_msg.header.frame_id = ''

        sa_msg.num_drones_connected = self.pm_num_connected_drones
        sa_msg.num_drones_active = allocation.num_active

        sa_msg.connected_drones_isreal = [False] * self.pm_num_connected_drones
        sa_msg.connected_drones_status = [state.value + mode.value for state, mode in zip(allocation.state, allocation.mode)]
        sa_msg.connected_drones_active_ids = allocation.cd_active_drones
        sa_msg.connected_drones_namespaces = [namespace()]

        sa_msg.active_drones_connected_ids = AD_CONNECTED_ID



    def _publish_swarm_allocation(self):

        # assert len(sa_msg.connected_drones_active_ids) == sa_msg.num_drones_connected

        self.swarm_allocation_pub.publish(sa_msg)
        rospy.loginfo_throttle(5, "published allocation")


NUM_D_CONNECTED = 4
NUM_D_ACTIVE = 3

CD_ISREAL = [False, False, False, False]
CD_STATUS = [SwarmAllocation.STATUS_ACTIVE, SwarmAllocation.STATUS_ACTIVE,
             SwarmAllocation.STATUS_ACTIVE, SwarmAllocation.STATUS_READY]
CD_NAMESPACES = ['cf1', 'cf2', 'cf3', 'cf4']

# AD_CID[0] is the connected drone id that realize the active drone 0
AD_CONNECTED_ID = [0, 1, 2]


def cid2aid(ad_cid, num_connected):
    """
    reverse the AD_CID list to get a map from a connected drone id to the potential active drone id

    Args:
        ad_cid (list[int]): list of connected id for each active drone
        num_connected (int): number of total connected drones

    Returns:
        list[num_connected*int]: cell at index cid is aid if drone cid is active under active name aid, and -1 otherwise
    """
    cd_to_aid = [-1] * num_connected
    for aid, cid_for_that_aid in enumerate(ad_cid):
        cd_to_aid[cid_for_that_aid] = aid
    return cd_to_aid


CD_ACTIVE_ID = cid2aid(AD_CONNECTED_ID, NUM_D_CONNECTED)


# class StaticSwarmAllocation:
#     def __init__(self):
#         rospy.init_node("swarm_allocation")

#         self.swarm_allocation_pub = rospy.Publisher(
#             "/sp/swarm_allocation", SwarmAllocation, latch=True, queue_size=10)

#         self.rate_hz = 0.5
#         self.rate_ros = rospy.Rate(self.rate_hz)

#         self.takeoff_srv = rospy.Service(
#             "/sp/takeoff", TakeOff, self.srv_takeoff)
#         self.land_srv = rospy.Service("/sp/land", Land, self.srv_land)
#         self.activeDroneInfo_srv = rospy.Service(
#             "/sp/active_drone_info", ActiveDroneInfo, self.srv_active_drone_info)

#         self.swarm = []



#     def srv_active_drone_info(self, req):
#         activeId = req.active_id
#         try:
#             cid = CD_ACTIVE_ID.index(activeId)
#         except ValueError as _:
#             return None

#         res = ActiveDroneInfoResponse()
#         res.connected_id = cid
#         res.is_real = CD_ISREAL[cid]
#         res.namespace = CD_NAMESPACES[cid]
#         res.status = CD_STATUS[cid]
#         return res

#     @staticmethod
#     def _interpret_set(request_id):
#         drone_set = []

#         if request_id >= 0:
#             drone_set = [request_id]
#         elif request_id == TakeOffRequest.ALL:
#             drone_set = list(range(NUM_D_CONNECTED))
#         elif request_id == TakeOffRequest.ACTIVE:
#             drone_set = AD_CONNECTED_ID
#         else:
#             # parameter makes no sense
#             return None
#         rospy.logdebug("got swarm service request with selector {}. Selecting {}".format(
#             request_id, drone_set))
#         return drone_set

#     def srv_takeoff(self, req):
#         """
#         TODO: Must be Thread-Safe
#         """
#         drone_set = self._interpret_set(req.id)
#         for cid in drone_set:
#             drone = self.swarm[cid]
#             drone.takeoff(targetHeight=1.5, duration=5.)
#         return TakeOffResponse(True)

#     def srv_land(self, req):
#         drone_set = self._interpret_set(req.id)
#         for cid in drone_set:
#             drone = self.swarm[cid]
#             drone.land(targetHeight=0., duration=5.)
#         return LandResponse(True)

#     def spin(self):
#         # self.takeoff_drones()
#         self._init_high_level()

#         while not rospy.is_shutdown():
#             self._publish_swarm_allocation()
#             self.rate_ros.sleep()
#         # self.land_drones()


def rosmain():
    ssa = StaticSwarmAllocation()
    ssa.spin()


if __name__ == "__main__":
    rosmain()
