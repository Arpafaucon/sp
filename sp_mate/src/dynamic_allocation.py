#!/usr/bin/python2
"""
static allocation server
"""
from threading import RLock
import rospy
from sp_core.msg import SwarmAllocation
from crazyflie_gazebo.tools import crazyflie
from sp_mate.srv import Land, TakeOff, LandRequest, LandResponse, TakeOffRequest, TakeOffResponse, ActiveDroneInfo, ActiveDroneInfoRequest, ActiveDroneInfoResponse, DroneState, DroneStateRequest, DroneStateResponse, ActiveTarget, ActiveTargetResponse
from sp_lookout.msg import SwarmPosition
import time
from drone_state_machine import MODE, STATE, DroneStateMachine, DroneAllocation


PUB_ALLOCATION = "/sp/swarm_allocation"
SUB_POSITION = "/sp/all_swarm_position"


class DynamicAllocation(object):
    """
    assumptions:
    - drone namespaces are 'cf<i>' , i starting from 1. So drone 0 has namespace 'cf1'
    """
    @staticmethod
    def namespace(drone_id):
        return "cf{}".format(drone_id+1)

    def __init__(self):

        # ROS INIT
        rospy.init_node("dynamic_allocation")

        # publishers:
        self.swarm_allocation_pub = rospy.Publisher(
            PUB_ALLOCATION, SwarmAllocation, latch=True, queue_size=10)

        # subscribers
        self.swarm_position_sub = rospy.Subscriber(
            SUB_POSITION, SwarmPosition, self.cb_swarm_position)

        # services
        # self.svros_takeoff = rospy.Service(
        #     "/sp/takeoff", TakeOff, self.srv_takeoff)
        # self.svros_land = rospy.Service("/sp/land", Land, self.srv_land)
        # self.svros_active_drone_info = rospy.Service(
        #     "/sp/active_drone_info", ActiveDroneInfo, self.srv_active_drone_info)
        self.svros_active_target = rospy.Service(
            "/sp/active_target", ActiveTarget, self.srv_active_target)
        self.svros_drone_state = rospy.Service("/sp/set_drone_state", DroneState, self.srv_set_drone_state)
        # Params
        self.pm_num_connected_drones = int(
            rospy.get_param("/sp/mate/num_drones_total"))
        self.pm_num_active_drones = int(
            rospy.get_param("/sp/mate/num_drones_active"))
        self.pm_altitude = float(rospy.get_param("/sp/mate/altitude"))
        self.pm_transition_duration = float(
            rospy.get_param("/sp/mate/transition_duration", 2.))
        self.pm_transition_threshold = float(
            rospy.get_param("/sp/mate/transition_threshold", .1))
        self.pmd_landed_thres = self.pm_altitude * self.pm_transition_threshold
        self.pmd_flying_thres = self.pm_altitude * \
            (1-self.pm_transition_threshold)
        self.pm_rate = float(rospy.get_param("/sp/mate/allocation_rate"))
        self.rate_ros = rospy.Rate(self.pm_rate)

        # Class
        self.ms_machine = DroneStateMachine.new(self.pm_num_connected_drones)
        self.ms_machine.on_mode_change(self.cb_mode_changed)
        self.ms_machine.on_state_change(self.cb_state_changed)
        self.swarm = None  # type: List[Crazyflie]

        # results
        self.last_da_lock = RLock()
        self.last_da = None  # type: DroneAllocation

    def srv_set_drone_state(self, req):
        # type: (DroneStateRequest) -> DroneStateResponse
        cid = req.connected_id
        if req.state == req.STATE_FAULTY:
            new_state = STATE.FAULTY
        elif req.state == req.STATE_OK:
            new_state = STATE.OK
        else:
            rospy.logerr("Unrecognized state change. Ignoring.")
            return None
        self.ms_machine.register_drone_update(cid, state=new_state)
        return DroneStateResponse()

    def srv_active_target(self, req):
        # type: (ActiveTargetRequest) -> ActiveTargetResponse
        target = req.active_target
        if target > self.pm_num_connected_drones:
            target = self.pm_num_connected_drones
        rospy.loginfo("New active target: {} [requested {}]".format(
            target, req.active_target))

        self.ms_machine.register_active_target_update(target)
        return ActiveTargetResponse(active_target_ok=target)

    def srv_takeoff(self, req):
        # type: (TakeOffRequest)->TakeOffResponse
        return None

    def srv_land(self, req):
        # type: (LandRequest)->LandResponse
        return None

    def spin(self):
        self._init_high_level()
        while not rospy.is_shutdown():
            with self.last_da_lock:
                allocation = self.ms_machine.spin()
                self.last_da = allocation
            self.pub_allocation(allocation)
            self.rate_ros.sleep()

    def _init_high_level(self):
        # setup drones
        self.swarm = []
        for cid in range(self.pm_num_connected_drones):
            cd_namespace = self.namespace(cid)
            # FIXME: second argument is weird
            drone = crazyflie.Crazyflie(cd_namespace, cid)
            drone.enableHighLevel()
            self.swarm.append(drone)

    def cb_swarm_position(self, msg):
        # type: (SwarmPosition) -> None
        """
        Checks for transitions : 
        LANDING -> GROUND once altitude is below (threshold * altitude)
        TAKEOFF -> ACTIVE once altitude is above (1 - threshold)*altitude
        """
        # for
        assert msg.mask == SwarmPosition.MASK_ALL, "wrong subscription"
        assert msg.num_drones == self.pm_num_connected_drones, "consistent connected drones number"
        with self.last_da_lock:
            if self.last_da is None:
                return
            for cid in range(self.pm_num_connected_drones):
                d_mode = self.last_da.mode[cid]
                if d_mode == MODE.LANDING:
                    # check if altitude is below threshold
                    if msg.z[cid] < self.pmd_landed_thres:
                        rospy.loginfo("Landing finished for {}".format(cid))
                        self.ms_machine.register_drone_update(
                            cid, mode=MODE.GROUND)
                elif d_mode == MODE.TAKEOFF:
                    # check if altitude is above threshold
                    if msg.z[cid] > self.pmd_flying_thres:
                        rospy.loginfo("TakeOff finished for {}".format(cid))
                        self.ms_machine.register_drone_update(
                            cid, mode=MODE.ACTIVE)

    def cb_mode_changed(self, drone_index, old_mode, new_mode):
        # type: (int, MODE, MODE) -> None
        if old_mode in [MODE.GROUND, MODE.LANDING] and new_mode == MODE.TAKEOFF:
            self.swarm[drone_index].takeoff(
                targetHeight=self.pm_altitude,
                duration=self.pm_transition_duration)
            rospy.loginfo("Taking off {}".format(drone_index))
        if old_mode in [MODE.ACTIVE, MODE.TAKEOFF] and new_mode == MODE.LANDING:
            self.swarm[drone_index].land(
                targetHeight=0.,
                duration=self.pm_transition_duration)
            rospy.loginfo("Landing {}".format(drone_index))

    def cb_state_changed(self, drone_index, old_state, new_state):
        rospy.logwarn("cid# {} : state changed : {} to {}".format(
            drone_index, old_state, new_state))
        # type: (int, STATE, STATE) -> None
        pass

    def pub_allocation(self, allocation):
         # type: (DroneAllocation) -> None
        sa_msg = SwarmAllocation()

        sa_msg.header.stamp = rospy.Time.now()
        sa_msg.header.frame_id = ''

        sa_msg.num_drones_connected = self.pm_num_connected_drones
        sa_msg.num_drones_active = allocation.num_active

        sa_msg.connected_drones_isreal = [False] * self.pm_num_connected_drones
        sa_msg.connected_drones_status = [
            state.value + mode.value for state, mode in zip(allocation.state, allocation.mode)]
        sa_msg.connected_drones_active_ids = allocation.cd_active_drones
        sa_msg.connected_drones_namespaces = [DynamicAllocation.namespace(
            cd_id) for cd_id in range(self.pm_num_connected_drones)]

        sa_msg.active_drones_connected_ids = allocation.ad_connected_drones

        self.swarm_allocation_pub.publish(sa_msg)
        rospy.loginfo_throttle(
            20, "publishing allocation at {} Hz".format(self.pm_rate))


# NUM_D_CONNECTED = 4
# NUM_D_ACTIVE = 3

# CD_ISREAL = [False, False, False, False]
# CD_STATUS = [SwarmAllocation.STATUS_ACTIVE, SwarmAllocation.STATUS_ACTIVE,
#              SwarmAllocation.STATUS_ACTIVE, SwarmAllocation.STATUS_READY]
# CD_NAMESPACES = ['cf1', 'cf2', 'cf3', 'cf4']

# # AD_CID[0] is the connected drone id that realize the active drone 0
# AD_CONNECTED_ID = [0, 1, 2]


# def cid2aid(ad_cid, num_connected):
#     """
#     reverse the AD_CID list to get a map from a connected drone id to the potential active drone id

#     Args:
#         ad_cid (list[int]): list of connected id for each active drone
#         num_connected (int): number of total connected drones

#     Returns:
#         list[num_connected*int]: cell at index cid is aid if drone cid is active under active name aid, and -1 otherwise
#     """
#     cd_to_aid = [-1] * num_connected
#     for aid, cid_for_that_aid in enumerate(ad_cid):
#         cd_to_aid[cid_for_that_aid] = aid
#     return cd_to_aid


# CD_ACTIVE_ID = cid2aid(AD_CONNECTED_ID, NUM_D_CONNECTED)


# class StaticSwarmAllocation:
#     def __init__(self):
#         rospy.init_node("swarm_allocation")

#         self.swarm_allocation_pub = rospy.Publisher(
#             "/sp/swarm_allocation", SwarmAllocation, latch=True, queue_size=10)

#         self.rate_hz = 0.5
#         self.rate_ros = rospy.Rate(self.rate_hz)


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
    dynamic_mate = DynamicAllocation()
    dynamic_mate.spin()


if __name__ == "__main__":
    rosmain()
