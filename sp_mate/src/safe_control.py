#!/usr/bin/python

import math
from threading import RLock

import rospy
from crazyflie_driver.msg import GenericLogData, Position
from crazyflie_driver.srv import GoTo, GoToRequest
from sp_captain.msg import CaptainOrders
from sp_mate.srv import ActiveDroneInfo
from sp_lookout.srv import DronePosition
from sp_mate.msg import SwarmAllocation

from sp_core.tools.rospools import PublisherPool, ServiceProxyPool

from geometry_msgs.msg import PointStamped

from waypoint_geometry import distance_to_collision, target_distance_cap

DRONE_INFO_SRV = '/sp/active_drone_info'
DRONE_LOCATION_SRV = '/sp/drone_position_connected'

RATE_PM = '/sp/mate/control_rate'
GOAL_RADIUS_PM = "/sp/mate/goal_radius"
ALTITUDE_PM = "/sp/mate/altitude"
PM_COLLISION_DISTANCE = "/sp/mate/collision_distance"
PM_DISTANCE_CAP = "/sp/mate/goto_distance_cap"
PM_GOTO_DURATION = "/sp/mate/goto_duration"

CAPTAIN_ORDERS_SUB = "/sp/captain_orders"
SUB_ALLOCATION = "/sp/swarm_allocation"

PUB_NET_ORDERS_SUFFIX = "sp_cmd_net"
PUB_RAW_ORDERS_SUFFIX = "sp_cmd_raw"
PUB_POS_SUFFIX = "sp_pos"


class DroneMission:
    next_free_id = 0

    @staticmethod
    def new():
        m = DroneMission(DroneMission.next_free_id)
        DroneMission.next_free_id += 1
        print("created mission {}".format(m.mission_id))
        return m

    def __init__(self, mid=0):
        self.mission_id = mid
        self.active_id = None  # type: int
        self.connected_id = None  # type: int
        self.waypoints = None  # type: List[Tuple[float, float, float]]
        self.current_wp = None  # type: int # index in `waypoints`
        self.arrived = None  # type: bool
        self.namespace = None  # type: string # ex '/cf1'
        self.distance = None  # type: float


class SafeControl:
    def __init__(self):
        rospy.init_node("sp_control", log_level=rospy.INFO)
        rospy.loginfo("control: init")
        self.missions = []
        self.mission_lock = RLock()

        # ROS Services
        rospy.loginfo("init services...")
        rospy.wait_for_service(DRONE_INFO_SRV)
        self.active_drone_info_svp = rospy.ServiceProxy(
            DRONE_INFO_SRV, ActiveDroneInfo)
        rospy.wait_for_service(DRONE_LOCATION_SRV)
        self.drone_location_svp = rospy.ServiceProxy(
            DRONE_LOCATION_SRV, DronePosition)
        rospy.loginfo("control: Required services are available")

        # ROS Params
        self.pm_rate_hz = float(rospy.get_param(RATE_PM))
        self.rate_ros = rospy.Rate(self.pm_rate_hz)

        self.pm_hover_altitude = float(rospy.get_param(ALTITUDE_PM))
        self.pm_goal_radius = float(rospy.get_param(GOAL_RADIUS_PM))
        self.pm_collision_distance = float(
            rospy.get_param(PM_COLLISION_DISTANCE))
        self.pm_goto_distance_cap = float(rospy.get_param(PM_DISTANCE_CAP))
        self.pm_goto_duration = float(rospy.get_param(PM_GOTO_DURATION))

        # self.pmd_collision_distance_squared = self.pm_collision_distance**2

        # ROS Pools 
        # viz pool to display current goto point
        self.net_target_vizpool = PublisherPool(PointStamped, PUB_NET_ORDERS_SUFFIX)
        self.raw_target_vizpool = PublisherPool(PointStamped, PUB_RAW_ORDERS_SUFFIX)
        self.position_vizpool = PublisherPool(PointStamped, PUB_POS_SUFFIX)
        # service pool to set current goto point
        self.target_srv_pool = ServiceProxyPool(GoTo, "go_to")

        # ROS Subscribers
        # When all is set up, advertise topic
        self.sub_allocation = rospy.Subscriber(SUB_ALLOCATION, SwarmAllocation, self.cb_allocation)
        self.last_allocation_msg = None

        self.captain_orders_sub = rospy.Subscriber(
            CAPTAIN_ORDERS_SUB, CaptainOrders, self.cb_captain_orders, queue_size=5)
        self.last_orders_msg = None
        rospy.loginfo("control: init done")

    # def _swarm_allocation_cb(self, msg):
    #     self.swarm_allocation_msg = msg

    def _get_active_info(self, active_id):
        adi_res = self.active_drone_info_svp(active_id=active_id)
        return adi_res

    def update_mission_allocation(self, mission):
        # type: (DroneMission) -> bool
        """
        fill mission with current allocation details
        
        Returns:
            bool: mission was allocated successfully
        """
        aid = mission.active_id
        if aid >= self.last_allocation_msg.num_drones_active:
            # active id is not valid
            return False
        
        cid = self.last_allocation_msg.active_drones_connected_ids[aid]
        mission.connected_id = cid
        mission.namespace = "/" + self.last_allocation_msg.connected_drones_namespaces[cid]
        return True

    def cb_captain_orders(self, msg):
        self.last_orders_msg = msg
        rospy.logdebug("control: got new missions")
        self._build_missions()

    def cb_allocation(self, msg):
        if self.last_allocation_msg is None or msg.changed_since_last:
            # Gotta do work!
            # Missions we had are outdated now
            rospy.loginfo("control: Swarm allocation changed")
            self._build_missions()
        self.last_allocation_msg = msg


    def _build_missions(self):
        with self.mission_lock:
            self.missions = []
        # get connected drone ID corresponding to active drone id
        # assign for each connected drone list of waypoints
        if self.last_orders_msg is None:
            rospy.logdebug("No orders. Skipping mission build")
            return
        if self.last_allocation_msg is None:
            rospy.logdebug("No allocation. Skipping mission build")
            return
        rospy.loginfo("Rebuilding mission set")

        msg = self.last_orders_msg
        new_missions = []
        for drone_id in range(msg.num_drones):
            wp_start_index = msg.waypoints_start_indices[drone_id]
            wp_stop_index = msg.waypoints_stop_indices[drone_id]
            wp_x = msg.waypoints_x[wp_start_index:wp_stop_index]
            wp_y = msg.waypoints_y[wp_start_index:wp_stop_index]
            wp_z = [self.pm_hover_altitude for wp_index in range(
                wp_start_index, wp_stop_index)]
            wp_tuples = list(zip(wp_x, wp_y, wp_z))

            # active_info = self._get_active_info(drone_id)

            m = DroneMission.new()
            m.active_id = drone_id
            m.waypoints = wp_tuples
            m.distance = msg.distance[drone_id]
            # IMPORTANT: changing the below value to 1 means that the first target drones will try to reach is the first waypoint (and not the starting point of the trip)
            m.current_wp = 1  # < len(wp_tuples)
            m.arrived = False
            new_missions.append(m)
        
        # allocate first 'num_drones' missions that are valid
        num_active = self.last_allocation_msg.num_drones_active
        allocated_missions = []
        for mission_ix, mission in enumerate(new_missions):
            if mission_ix >= num_active:
                break
            valid = self.update_mission_allocation(mission)
            if valid:
                allocated_missions.append(mission)

        def key_distance(mission):
            # type: (DroneMission) -> float
            return mission.distance

        # missions are sorted by decreasing distance
        allocated_missions.sort(key=key_distance, reverse=True)

        with self.mission_lock:
            self.missions = allocated_missions
        
        rospy.logdebug("Rebuild mission set. %d created, %d allocated, %d total", len(new_missions), len(allocated_missions), len(self.missions))

    @staticmethod
    def _norm_nd(x, y, dim=3):
        """
        Cartesian 2D/3D norm

        Args:
            x (tuple|list): 1st point
            y (tuple|list): 2nd point
            dim (int) : dimension : 2 | 3

        Returns:
            float: distance in the 2D horizontal plane if dim=2
            float: distance in 3D if dim=3
        """
        dist2 = sum((x[i] - y[i])**2 for i in range(2))
        return math.sqrt(dist2)

    def detect_collision(self, current_pos, target_pos, collision_points):
        # type: (Tuple[float, float, float],Tuple[float, float, float],Tuple[float, float, float])->bool
        """
        Return true if mission is currently menacing to collide with a higher priority drone

        Higher priority drones were processed before this one, and their current position is in 'collision_points'
        """
        for point in collision_points:
            dist_to_wayline = distance_to_collision(current_pos, target_pos, point)
            if dist_to_wayline < self.pm_collision_distance:
                rospy.logwarn("collision detected between point %s and line [%s %s]. Distance was %s [< %s]", point, current_pos, target_pos, dist_to_wayline, self.pm_collision_distance)
                return True, point
        return False, 0

    def cap_distance(self, current_pos, target_pos):
        # type: (Tuple[float, float, float],Tuple[float, float, float])->bool
        """
        return a target in the right direction of target_pos, but at a maximal distance of pm_distance_cap
        """
        net_target = target_distance_cap(current_pos, target_pos, self.pm_goto_distance_cap)
        return net_target


    def control_drone(self, mission, collision_points):
        try:
            current_6d = self.drone_location_svp(
                id=mission.connected_id).position
        except rospy.ServiceException as _:
            # control was invalid
            rospy.logdebug("Got invalid response for position. Skipping control for this drone.")
            return False
        current_pos = tuple(current_6d[0:3])

        if mission.arrived:
            # slacking off : job already done
            return

        # check if current target was reached
        while True:
            target = mission.waypoints[mission.current_wp]
            dist_to_target = self._norm_nd(target, current_pos, 2)

            if dist_to_target < self.pm_goal_radius:
                if mission.current_wp+1 < len(mission.waypoints):
                    rospy.loginfo("[#{}] Waypoint {} reached for mission {} ".format(
                        mission.active_id, mission.current_wp, mission.mission_id))
                    mission.current_wp += 1
                else:
                    # we did it!!
                    rospy.loginfo("[#{}] Mission {} arrived".format(
                        mission.active_id, mission.mission_id))
                    mission.arrived = True
                    # no orders given
                    return
            else:
                # current goal exist and is not reached
                break
            pass
        # at this point, `target` is the next goal not yet reached
        # We now setup a point in the right direction, but to a capped distance 
        # to avoid divergent orders
        net_target = self.cap_distance(current_pos, target)


        # let's now check that the path is clear to reach it
        order_6d = [0. for _ in range(6)]
        is_collision, pbematic_point = self.detect_collision(current_pos, net_target, collision_points)
        if is_collision:
            # then we move no further and wait for collision to clear
            order_6d = list(current_6d)
            # fix for weird altitude 0 order sometimes
            order_6d[2] = self.pm_hover_altitude
            rospy.logwarn("active drone %d stayed in place because of a collision risk with drone in %s [current pos = %s]", mission.active_id, pbematic_point, current_6d)
        else:
            # free to go further
            order_6d[0:3] = net_target
        
        # in all cases, current position is protected againts waylines for lesser priority missions
        collision_points.append(current_pos)

        # and... send order
        self._send_order_position(mission, order_6d)
        self._pub_net_target(mission, net_target)
        self._pub_raw_target(mission, target)
        self._pub_position(mission, current_pos)
        return True

    def _pub_net_target(self, mission, net_target):
        viz_pub = self.net_target_vizpool.get_pub(mission.namespace)
        ps = self._build_target_pointstamped(net_target)
        viz_pub.publish(ps)
    
    def _pub_raw_target(self, mission, raw_target):
        viz_pub = self.raw_target_vizpool.get_pub(mission.namespace)
        ps = self._build_target_pointstamped(raw_target)
        viz_pub.publish(ps)

    def _pub_position(self, mission, position):
        viz_pub = self.position_vizpool.get_pub(mission.namespace)
        ps = self._build_target_pointstamped(position)
        viz_pub.publish(ps)

    def _build_target_pointstamped(self, target):
        ps = PointStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "/world"
        ps.point.x = target[0]
        ps.point.y = target[1]
        ps.point.z = target[2]
        return ps

    def _send_order_position(self, mission, order_6d):
        goto_srv_fun = self.target_srv_pool.get_svp(mission.namespace)
        rospy.loginfo("sending target to #{}  :{} [ns={}]".format(
            mission.active_id, order_6d, mission.namespace))
        tm = GoToRequest()
        tm.goal.x = order_6d[0]
        tm.goal.y = order_6d[1]
        tm.goal.z = order_6d[2]
        tm.yaw = order_6d[3]
        tm.duration = rospy.Duration.from_sec(self.pm_goto_duration)
        tm.relative = False
        goto_srv_fun(tm)

    def control_swarm(self):
        collision_points = []
        num_orders = 0
        with self.mission_lock:
            for mission in self.missions:
                num_orders +=1
                self.control_drone(mission, collision_points)
        rospy.logdebug("orders dispatched. There was %d missions in last control round", num_orders)

    def spin(self):
        while not rospy.is_shutdown():
            self.control_swarm()
            self.rate_ros.sleep()

    


def rosmain():
    dc = SafeControl()
    dc.spin()


if __name__ == "__main__":
    rosmain()
