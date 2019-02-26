#!/usr/bin/python

import math

import rospy
from crazyflie_driver.msg import GenericLogData, Position
from crazyflie_driver.srv import GoTo, GoToRequest
from sp_core.msg import CaptainOrders
from sp_mate.srv import ActiveDroneInfo
from sp_lookout.srv import DronePosition

from sp_core.tools.rospools import PublisherPool, ServiceProxyPool

from geometry_msgs.msg import PointStamped

from waypoint_geometry import distance_to_collision

DRONE_INFO_SRV = '/sp/active_drone_info'
DRONE_LOCATION_SRV = '/sp/drone_position'

RATE_PM = '/sp/mate/control_rate'
GOAL_RADIUS_PM = "/sp/mate/goal_radius"
ALTITUDE_PM = "/sp/mate/altitude"
PM_COLLISION_DISTANCE = "/sp/mate/collision_distance"

CAPTAIN_ORDERS_SUB = "/sp/captain_orders"
DRONE_ORDERS_SUFFIX = "sp_cmd_position"



# param sp_mate/rate


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
        self.namespace = None  # type: string
        self.distance = None  # type: float


class DumbControl:
    def __init__(self):
        rospy.init_node("sp_control")
        rospy.loginfo("control: init")

        rospy.loginfo("init services...")
        rospy.wait_for_service(DRONE_INFO_SRV)
        self.active_drone_info_svp = rospy.ServiceProxy(
            DRONE_INFO_SRV, ActiveDroneInfo)
        rospy.wait_for_service(DRONE_LOCATION_SRV)
        self.drone_location_svp = rospy.ServiceProxy(
            DRONE_LOCATION_SRV, DronePosition)
        rospy.loginfo("control: Required services are available")

        self.rate_hz = float(rospy.get_param(RATE_PM))
        self.rate_ros = rospy.Rate(self.rate_hz)

        self.hover_altitude = float(rospy.get_param(ALTITUDE_PM))
        self.goal_radius = float(rospy.get_param(GOAL_RADIUS_PM))
        self.pm_collision_distance = float(
            rospy.get_param(PM_COLLISION_DISTANCE))
        # self.pmd_collision_distance_squared = self.pm_collision_distance**2

        self.missions = []

        self.target_viz_pubpool = PublisherPool(PointStamped, DRONE_ORDERS_SUFFIX)
        self.target_srv_pool = ServiceProxyPool(GoTo, "go_to")

        # When all is set up, advertise topic
        self.captain_orders_sub = rospy.Subscriber(
            CAPTAIN_ORDERS_SUB, CaptainOrders, self._captain_orders_cb, queue_size=5)
        self.captain_orders_msg = None
        rospy.loginfo("control: init done")

    # def _swarm_allocation_cb(self, msg):
    #     self.swarm_allocation_msg = msg

    def _get_active_info(self, active_id):
        adi_res = self.active_drone_info_svp(active_id=active_id)
        return adi_res

    def update_mission_allocation(self, mission):
        active_info = self._get_active_info(mission.active_id)
        mission.connected_id = active_info.connected_id
        mission.namespace = active_info.namespace

    def _captain_orders_cb(self, msg):
        # get connected drone ID corresponding to active drone id
        # assign for each connected drone list of waypoints
        self.captain_orders_msg = msg
        rospy.logdebug("control: got new missions")

        new_missions = []
        for drone_id in range(msg.num_drones):
            wp_start_index = msg.waypoints_start_indices[drone_id]
            wp_stop_index = msg.waypoints_stop_indices[drone_id]
            wp_x = msg.waypoints_x[wp_start_index:wp_stop_index]
            wp_y = msg.waypoints_y[wp_start_index:wp_stop_index]
            wp_z = [self.hover_altitude for wp_index in range(
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
            self.update_mission_allocation(m)
            new_missions.append(m)

        def key_distance(mission):
            # type: (DroneMission) -> float
            return mission.distance

        # missions are sorted by decreasing distance
        new_missions.sort(key=key_distance, reverse=True)

        self.missions = new_missions

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
        # type: (DroneMission, List[Tuple[float, float, float]])->bool
        """
        Return true if mission is currently menacing to collide with a higher priority drone

        Higher priority drones were processes before this one, and their current position is in 'collision_points'
        """
        for point in collision_points:
            dist_to_wayline = distance_to_collision(current_pos, target_pos, point)
            if dist_to_wayline < self.pm_collision_distance:
                return True, point
        return False, 0

    def control_drone(self, mission, collision_points):
        current_6d = self.drone_location_svp(
            active_id=mission.active_id).position
        current_pos = tuple(current_6d[0:3])

        if mission.arrived:
            # slacking off : job already done
            return

        # check if current target was reached
        while True:
            target = mission.waypoints[mission.current_wp]
            dist_to_target = self._norm_nd(target, current_pos, 2)

            if dist_to_target < self.goal_radius:
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
        # let's now check that the path is clear to reach it
        order_6d = [0. for _ in range(6)]
        is_collision, pbematic_point = self.detect_collision(current_pos, target, collision_points)
        if is_collision:
            # then we move no further and wait for collision to clear
            order_6d = current_6d
            rospy.logdebug("active drone %d stayed in place because of a collision risk with drone in %s", mission.active_id, pbematic_point)
        else:
            # free to go further
            order_6d[0:3] = target
        
        # in all cases, current position is protected againts waylines for lesser priority missions
        collision_points.append(current_pos)

        # and... send order
        self._send_order_position(mission, order_6d)

    def _send_order_position(self, mission, order_6d):
        goto_srv_fun = self.target_srv_pool.get_pub(mission.namespace)
        rospy.loginfo("sending target to #{}  :{} [ns={}]".format(
            mission.active_id, order_6d, mission.namespace))
        tm = GoToRequest()
        tm.goal.x = order_6d[0]
        tm.goal.y = order_6d[1]
        tm.goal.z = order_6d[2]
        tm.yaw = order_6d[3]
        tm.duration.secs = 2
        tm.relative = False
        goto_srv_fun(tm)

        viz_pub = self.target_viz_pubpool.get_pub(mission.namespace)
        ps = PointStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "/world"
        ps.point.x = order_6d[0]
        ps.point.y = order_6d[1]
        ps.point.z = order_6d[2]
        viz_pub.publish(ps)

    def control_swarm(self):
        collision_points = []
        for mission in self.missions:
            self.control_drone(mission, collision_points)

    def spin(self):
        while not rospy.is_shutdown():
            self.control_swarm()
            self.rate_ros.sleep()

    


def rosmain():
    dc = DumbControl()
    dc.spin()


if __name__ == "__main__":
    rosmain()
