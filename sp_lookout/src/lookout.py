#!/usr/bin/python
"""
Lookout node
"""
import math
import rospy
import numpy as np
import sys

from threading import RLock
from sp_core.tools.rospools import PublisherPool

from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from crazyflie_driver.msg import GenericLogData
from sp_mate.msg import SwarmAllocation
from sp_lookout.srv import DronePosition, DronePositionRequest, DronePositionResponse, SwarmPositionSrv, SwarmPositionSrvResponse
from sp_lookout.msg import SwarmPosition

# number of null messages to support before giving up
# and throwing an error
MAX_ZERO_MSG = 10

SUB_SWARM_ALLOCATION = "/sp/swarm_allocation"

class DroneLog(object):
    def __init__(self):
        self.position = None # type: List[float]
        self.active = None # type: bool
        self.namespace = None # type: str
        self.cid = None # type: int
        self.aid = None # type: int

    @staticmethod
    def extract_swarm_coordinate(drone_log_list, coord_index, active_only):
        # type: (List[DroneLog], int, bool) -> List[float]
        res = []
        for dronelog in drone_log_list:
            if not (active_only and not dronelog.active):
                res.append(dronelog.position[coord_index])
        return res


class Lookout:
    def __init__(self):
        rospy.init_node("sp_lookout")
        # Node rate
        self.pm_rate = float(rospy.get_param('/sp/lookout/rate'))
        self.rate_ros = rospy.Rate(self.pm_rate)
        # Allocation subscriber
        self.swarm_alloc_sub = rospy.Subscriber(
            '/sp/swarm_allocation', SwarmAllocation, self._cb_swarm_alloc, None)
        self.swarm_alloc_msg = None
        self.swarm_alloc_lock = RLock()
        self.swarm_log = None
        self.swarm_log_lock = RLock()

        # Position publisher
        self.pub_active_swarm_pos = rospy.Publisher(
            '/sp/active_swarm_position', SwarmPosition, queue_size=10)
        self.pub_all_swarm_pos = rospy.Publisher(
            '/sp/all_swarm_position', SwarmPosition, queue_size=10)
        
        # drone position service
        self.drone_pos_srv = rospy.Service(
            "/sp/drone_position", DronePosition, self._srv_drone_position)
        # swarm position service
        self.active_swarm_pos_srv = rospy.Service(
            "/sp/swarm_position", SwarmPositionSrv, self._srv_swarm_position)

        # position publishers
        self.pose_pubpool = PublisherPool(PoseStamped, "pose_stamped")

    def _srv_drone_position(self, req):
        active_id = req.active_id
        with self.swarm_log_lock:
            if self.swarm_log is None:
                return None
            drone_pos = self.swarm_log[active_id].position
            res = DronePositionResponse()
            res.position = drone_pos
            return res

    def _srv_swarm_position(self, req):
        active_only = req.active_only
        with self.swarm_log_lock:
            if self.swarm_log is None:
                return None
            (l_x, l_y, l_z, l_roll, l_pitch, l_yaw) = self._extract_coord_swarm(self.swarm_log, active_only=active_only)
        
            res = SwarmPositionSrvResponse()
            res.num_drones = len(l_x)
            res.x = l_x
            res.y = l_y
            res.z = l_z
            res.roll = l_roll
            res.pitch = l_pitch
            res.yaw = l_yaw
            
            return res

    def _cb_swarm_alloc(self, msg):
        with self.swarm_alloc_lock:
            self.swarm_alloc_msg = msg

    def _get_log(self, d_connected_id):
        with self.swarm_alloc_lock:
            namespace = self.swarm_alloc_msg.connected_drones_namespaces[d_connected_id]
            position_topic = "/{}/local_position".format(namespace)
            messages_seq = []
            for _ in range(MAX_ZERO_MSG):
                try:
                    local_position_msg = rospy.wait_for_message(
                        position_topic, GenericLogData, timeout=2) # type: GenericLogData
                except rospy.ROSException as re:
                    rospy.logerr("getting local_position for {} took too long. Check this topic is actually published".format(namespace))
                    rospy.logerr(re.message)
                    sys.exit(1)
                is_null_msg = all(abs(coord) < 1e-6 for coord in local_position_msg.values)
                messages_seq.append(local_position_msg.header.seq)
                if not is_null_msg:
                    break

            if is_null_msg:
                # happens if all loop iteration gave null messages
                rospy.logerr("%d consecutive null msg for #%d position. Will proceed with a null msg. Got seq %s", MAX_ZERO_MSG, d_connected_id, messages_seq)

            coord_array = [0] * 6
            # copy x, y, z as is
            for i in range(0, 3):
                coord_array[i] = local_position_msg.values[i]
            # converts angles to radian
            for i in range(3, 6):
                coord_array[i] = local_position_msg.values[i] * math.pi / 180.

            dl = DroneLog()
            dl.position = coord_array
            dl.cid = d_connected_id
            dl.aid = self.swarm_alloc_msg.connected_drones_active_ids[d_connected_id]
            dl.active = (dl.aid != -1)
            dl.namespace = namespace
            return dl

    def _get_log_active(self, d_active_id):
        """
        Get active drone position

        Args:
            drone_active_id (int): id of active drone

        Returns:
            DroneLog: log for the drone with position
        """
        with self.swarm_alloc_lock:
            # assert d_active_id < self.swarm_alloc_msg.num_drones_active
            d_connected_id = self.swarm_alloc_msg.active_drones_connected_ids[d_active_id]

            return self._get_log(d_connected_id)

    def _publish_pose_stamped_all_drones(self, swarm_log):
        # type: (List[DroneLog]) -> None
        for drone_log in swarm_log:
            pos = drone_log.position
            namespace = drone_log.namespace
            self._pub_pose_stamped(namespace, *pos)
        rospy.loginfo_throttle(5, "published PoseStamped for drones")

    def _pub_pose_stamped(self, namespace, x, y, z, roll, pitch, yaw):
        if namespace[0] != "/":
            # make namespace absolute
            namespace = "/" + namespace
        pub = self.pose_pubpool.get_pub(namespace)
        pos = PoseStamped()
        pos.header.frame_id = "/world"
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
        for cd_id in range(self.swarm_alloc_msg.num_drones_connected):
            drone_log = self._get_log(cd_id)
            swarm_log.append(drone_log)
        return swarm_log

    # @DeprecationWarning
    # def _build_position_array(self, swarm_log):
    #     num_drones = len(swarm_log)
    #     positions = np.zeros((num_drones, 6))
    #     # x_l = []
    #     # y_l = []
    #     # z_l = []
    #     # roll_l = []
    #     # pitch_l = []
    #     # yaw_l = []

    #     for drone_id in range(num_drones):
    #         # index 0 is position, 1 is namespace
    #         drone_pos, drone_ns = swarm_log[drone_id]
    #         positions[drone_id] = drone_pos
    #     return positions

    def _extract_coord_swarm(self, swarm_log, active_only):
        # type: (List[DroneLog], bool) -> (List[float], List[float], List[float], List[float], List[float], List[float])
        l_x = DroneLog.extract_swarm_coordinate(swarm_log, 0, active_only=active_only)
        l_y = DroneLog.extract_swarm_coordinate(swarm_log, 1, active_only=active_only)
        l_z = DroneLog.extract_swarm_coordinate(swarm_log, 2, active_only=active_only)
        l_roll = DroneLog.extract_swarm_coordinate(swarm_log, 3, active_only=active_only)
        l_pitch = DroneLog.extract_swarm_coordinate(swarm_log, 4, active_only=active_only)
        l_yaw = DroneLog.extract_swarm_coordinate(swarm_log, 5, active_only=active_only)
        return (l_x, l_y, l_z, l_roll, l_pitch, l_yaw)

    def _build_swarm_pos_msg(self, sw_log, active_only):
        # type: (List[DroneLog], bool) -> SwarmPosition
        swarm_position_msg = SwarmPosition()
        swarm_position_msg.header.stamp = rospy.Time.now()
        swarm_position_msg.header.frame_id = "/world"

        if active_only :
            swarm_position_msg.num_drones = self.swarm_alloc_msg.num_drones_active
            swarm_position_msg.mask = SwarmPosition.MASK_ACTIVE
        else:
            swarm_position_msg.num_drones = self.swarm_alloc_msg.num_drones_connected
            swarm_position_msg.mask = SwarmPosition.MASK_ALL

        (l_x, l_y, l_z, l_roll, l_pitch, l_yaw) = self._extract_coord_swarm(sw_log, active_only=active_only)
        
        swarm_position_msg.x = l_x
        swarm_position_msg.y = l_y
        swarm_position_msg.z = l_z
        swarm_position_msg.roll = l_roll
        swarm_position_msg.pitch = l_pitch
        swarm_position_msg.yaw = l_yaw

        return swarm_position_msg

    def _publish_swarm_pos(self, swarm_log):
        active_swarm_pos = self._build_swarm_pos_msg(swarm_log, active_only=True)
        self.pub_active_swarm_pos.publish(active_swarm_pos)

        all_swarm_pos = self._build_swarm_pos_msg(swarm_log, active_only=False)
        self.pub_all_swarm_pos.publish(all_swarm_pos)

        rospy.loginfo_throttle(
            20, "publishing swarm positions at {} hz".format(self.pm_rate))

    def spin(self):
        while (not rospy.is_shutdown()):
            swarm_log = self._get_swarm_log()
            if swarm_log is not None:
                with self.swarm_log_lock:
                    self.swarm_log = swarm_log

                self._publish_swarm_pos(swarm_log)
                self._publish_pose_stamped_all_drones(swarm_log)

            self.rate_ros.sleep()


def rosmain():
    look = Lookout()
    look.spin()


if __name__ == "__main__":
    rosmain()
