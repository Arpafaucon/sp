import math

import rospy
from crazyflie_driver.msg import GenericLogData, Position
from crazyflie_driver.srv import GoTo, GoToRequest
from sp_core.msg import CaptainOrders
from sp_mate.srv import ActiveDroneInfo, DroneLocation

DRONE_INFO_SRV = '/sp/active_drone_info'
DRONE_LOCATION_SRV = '/sp/drone_position'

RATE_PM = '/sp/mate/rate'
GOAL_RADIUS_PM = "/sp/mate/goal_radius"
ALTITUDE_PM = "/sp/mate/altitude"

CAPTAIN_ORDERS_SUB = "/sp/captain_orders"
DRONE_ORDERS_SUFFIX = "cmd_position"

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
        self.active_id = None # int
        self.conn_id = None # int
        self.waypoints = None # list[(double, double, double)]
        self.current_wp = None # index in `waypoints`
        self.arrived = None # bool
        self.namespace = None # string


class PublisherPool:
    def __init__(self, Message_Class, topic_suffix):
        self.pool = dict()
        self.MsgClass = Message_Class
        self.topic_suffix = topic_suffix

    def get_pub(self, namespace):
        if namespace in self.pool:
            return self.pool[namespace]
        else:
            topic = '{}/{}'.format(namespace, self.topic_suffix)
            pub = rospy.Publisher(topic, self.MsgClass, queue_size=10)
            self.pool[namespace] = pub
            return pub

class ServiceProxyPool:
    def __init__(self, Srv_class, srv_suffix):
        self.pool = dict()
        self.MsgClass = Srv_class
        self.srv_suffix = srv_suffix

    def get_pub(self, namespace):
        if namespace in self.pool:
            return self.pool[namespace]
        else:
            topic = '{}/{}'.format(namespace, self.srv_suffix)
            rospy.wait_for_service(topic)
            srv = rospy.ServiceProxy(topic, self.MsgClass)
            self.pool[namespace] = srv
            return srv


class DumbControl:
    def __init__(self):
        rospy.init_node("sp_control")
        rospy.loginfo("init")

        rospy.loginfo("init services...")
        rospy.wait_for_service(DRONE_INFO_SRV)
        self.active_drone_info_svp = rospy.ServiceProxy(DRONE_INFO_SRV,ActiveDroneInfo)
        rospy.wait_for_service(DRONE_LOCATION_SRV)
        self.drone_location_svp = rospy.ServiceProxy(DRONE_LOCATION_SRV, DroneLocation)
        rospy.loginfo("Required services are available")


        self.rate_hz = float(rospy.get_param(RATE_PM))
        self.rate_ros = rospy.Rate(self.rate_hz)

        self.hover_altitude = float(rospy.get_param(ALTITUDE_PM))
        self.goal_radius = float(rospy.get_param(GOAL_RADIUS_PM))

        self.missions = []

        self.target_pub_pool = PublisherPool(Position, DRONE_ORDERS_SUFFIX)
        self.target_srv_pool = ServiceProxyPool(GoTo, "go_to")

        # When all is set up, advertise topic
        self.captain_orders_sub = rospy.Subscriber(CAPTAIN_ORDERS_SUB, CaptainOrders, self._captain_orders_cb, queue_size=5)
        self.captain_orders_msg = None
        rospy.loginfo("init done")

    def _swarm_allocation_cb(self, msg):
        self.swarm_allocation_msg = msg

    def _get_active_info(self, active_id):
        adi_res = self.active_drone_info_svp(active_id=active_id)
        return adi_res

    def _captain_orders_cb(self, msg):
        # get connected drone ID corresponding to active drone id
        # assign for each connected drone list of waypoints
        self.captain_orders_msg = msg
        rospy.logdebug("got new missions")

        new_missions = []
        for drone_id in range(msg.num_drones):
            wp_start_index = msg.waypoints_start_indices[drone_id]
            wp_stop_index = msg.waypoints_stop_indices[drone_id]
            wp_x = msg.waypoints_x[wp_start_index:wp_stop_index]
            wp_y = msg.waypoints_y[wp_start_index:wp_stop_index]
            wp_z = [self.hover_altitude for wp_index in range(wp_start_index, wp_stop_index)]
            wp_tuples = list(zip(wp_x, wp_y, wp_z))

            active_info = self._get_active_info(drone_id)

            m = DroneMission.new()
            m.active_id = drone_id
            m.connected_id = active_info.connected_id
            m.waypoints = wp_tuples
            m.current_wp = 0 # < len(wp_tuples)
            m.namespace = active_info.namespace
            m.arrived = False

            new_missions.append(m)

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
        dist2 = sum( (x[i] - y[i])**2 for i in range(2))
        return math.sqrt(dist2)

    def control_drone(self, mission):
        current_pos = tuple(self.drone_location_svp(active_id = mission.active_id).position)

        if mission.arrived:
            # slacking off : job already done
            return

        # check if current target was reached
        while True:
            target = mission.waypoints[mission.current_wp]
            dist_to_target = self._norm_nd(target, current_pos, 2)
            
            if dist_to_target < self.goal_radius:
                if mission.current_wp+1 < len(mission.waypoints):
                    rospy.loginfo("[#{}] Waypoint {} reached for mission {} ".format(mission.active_id, mission.current_wp, mission.mission_id ))
                    mission.current_wp += 1
                else:
                    # we did it!!
                    rospy.loginfo("[#{}] Mission {} arrived".format(mission.active_id, mission.mission_id))
                    mission.arrived = True
                    # no orders given
                    return
            else :
                # current goal exist and is not reached
                break
            pass

        # at this point, `target` is the next goal not yet reached
        a = 2
        target_6d = [0. for _ in range(6)]

        #
        target_6d[0:3] = target
        self._send_order_position(mission, target_6d)

    # def _control_drone(current_pos, current_speed)

    # def _send_order_position(self, mission, order_6d):
    #     pub = self.target_pub_pool.get_pub(mission.namespace)
    #     rospy.loginfo("sending target to #{}  :{} [ns={}]".format(mission.active_id, order_6d, mission.namespace))
    #     tm = Position()
    #     tm.x = order_6d[0]
    #     tm.y = order_6d[1]
    #     tm.z = order_6d[2]
    #     tm.yaw = order_6d[3]
    #     pub.publish(tm)

    def _send_order_position(self, mission, order_6d):
        goto_srv_fun = self.target_srv_pool.get_pub(mission.namespace)
        rospy.loginfo("sending target to #{}  :{} [ns={}]".format(mission.active_id, order_6d, mission.namespace))
        tm = GoToRequest()
        tm.goal.x = order_6d[0]
        tm.goal.y = order_6d[1]
        tm.goal.z = order_6d[2]
        tm.yaw = order_6d[3]
        tm.duration.secs = 2
        tm.relative = False
        goto_srv_fun(tm)

        
    def control_swarm(self):
        for mission in self.missions:
            self.control_drone(mission)

    def spin(self):
        while not rospy.is_shutdown():
            self.control_swarm()
            self.rate_ros.sleep()
            
def rosmain():
    dc = DumbControl()
    dc.spin()

if __name__ == "__main__":
    rosmain()