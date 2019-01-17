"""
ROS io support for admiral.py
Define an interface class that takes care of data loads and publishs
"""
import math
import rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid
from sp_core.msg import AdmiralStatus, AdmiralOrders
# from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion

from .constants import M, Parameters
from .support import _prepare_map


DEFAULT_PARAM_NS = "/sp/admiral"
NODE_NAME = "admiral_node"

STATUS_TOPIC = "admiral_status"
SCORE_TOPIC = "admiral_score"
ORDERS_TOPIC = "admiral_orders"

MAP_ROOT_NS = ''

QUEUE_SIZE = 5


class AdmiralRosInterface(object):
    """
    ROS interface for Admiral.py
    - setup the node
    - load map
    - load parameters
    - publishes
        - score map
        - status report
        - orders for the Captain
    """

    def __init__(self):
        self._setup_node()
        self.status_pub = rospy.Publisher(
            "/sp/"+STATUS_TOPIC, AdmiralStatus, queue_size=QUEUE_SIZE)
        self.score_pub = rospy.Publisher(
            "/sp/"+SCORE_TOPIC, OccupancyGrid, queue_size=QUEUE_SIZE)
        self.orders_pub = rospy.Publisher(
            "/sp/"+ORDERS_TOPIC, AdmiralOrders, queue_size=QUEUE_SIZE)
        self.seq = 0
        self._raw_params = None
        self.params = None
        self.map_info = None

    def _setup_node(self, name=NODE_NAME):
        rospy.init_node(name=name)

    def init_map_params(self):
        """
        Load map and params simultaneously

        Returns:
            (np.array, constants.Parameters): observation_map, parameters
        """
        self._load_raw_params()
        load_ok, obs_map = self._load_map_ros(timeout=4)

        assert load_ok, "map should be correctly loaded"

        self._load_refined_params(self.map_info.resolution)
        _prepare_map(obs_map, self.params.WALL_RADIUS, self.params.SCORE_STEP_INCREMENT)
        return (obs_map, self.get_params())

    def _load_map_ros(self, timeout=4):
        map_topic = '{}/map'.format(MAP_ROOT_NS)
        try:
            map_msg = rospy.wait_for_message(
                map_topic, OccupancyGrid, timeout=timeout)
        except rospy.exceptions.ROSException as ex:
            rospy.logerr('loading failed with: {}'.format(ex))
            return (False, None)

        self.map_info = map_msg.info

        width = map_msg.info.width
        height = map_msg.info.height
        rospy.loginfo('got map of shape h={}, w={}'.format(height, width))
        data = np.array(map_msg.data, dtype=np.uint8).reshape((height, width))
        # invert in the occupancy grid, 0 is free and 100 is wall
        # we want : 255 is free and 0 is wall (more 'visual')
        data[data == 0] = 255
        data[data == 100] = 0

        obs_map = np.zeros((height, width, 3), dtype=np.uint8)
        obs_map[:, :, M.CONST] = 32
        # the occupancy grid has a normal frame coordinate system
        # therefore the y axis must be reversed to get the image CS
        obs_map[::-1, :, M.PHY] = data
        return (True, obs_map)

    def _load_raw_params(self):
        try:
            # check if namespace was defined as a private param
            absolute_ns = rospy.get_param("~namespace")
        except KeyError as _:
            absolute_ns = DEFAULT_PARAM_NS
        rno = absolute_ns + "/"
        num_drones = rospy.get_param(rno+'num_drones')
        wall_radius = rospy.get_param(rno+'wall_radius')
        initial_temp = rospy.get_param(rno+'initial_temp')
        n_iterations = rospy.get_param(rno+'n_iterations')
        drone_sight_radius = rospy.get_param(rno+'drone_sight_radius')
        score_step_increment = rospy.get_param(rno + 'score_step_increment')
        rate = rospy.get_param(rno+'rate')
        params = Parameters(num_drones, wall_radius, initial_temp,
                            n_iterations, drone_sight_radius, rate, score_step_increment)
        rospy.loginfo('loaded raw params : {}'.format(params))
        self._raw_params = params

    def _load_refined_params(self, resolution):
        raw_p = self._raw_params
        wall_radius = int(math.ceil(raw_p.WALL_RADIUS/resolution))
        drone_sight_radius = int(math.floor(
            raw_p.DRONE_SIGHT_RADIUS/resolution))
        
        refined_params = Parameters(
            raw_p.NUM_DRONES, wall_radius, raw_p.INITIAL_TEMP,
            raw_p.N_ITERATIONS, drone_sight_radius, raw_p.RATE, raw_p.SCORE_STEP_INCREMENT)
        rospy.loginfo('refined params : {}'.format(refined_params))
        self.params = refined_params

    def get_params(self):
        """
        Getter for the param fields
        runtime check if the param have already been loaded

        Raises:
            RuntimeError: if the param haven't been loaded yet

        Returns:
            constants.Parameters: namedtuple containing the parameters
        """
        if not self.params:
            raise RuntimeError(
                "Parameters are not ready yet. "
                "Make sure raw parameters were loaded from ROS, "
                "and converted after ROS map was loaded")
        return self.params

    def _transform_coordinates(self, state):
        ori = self.map_info.origin.orientation
        pos = self.map_info.origin.position
        explicit_quat = [ori.x, ori.y, ori.z, ori.w]
        yaw = euler_from_quaternion(explicit_quat)[2]
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        def _transform_drones(drone_i, drone_j):
            flat_x = (drone_j+.5) * self.map_info.resolution
            flat_y = (self.map_info.height - drone_i - .5) * \
                self.map_info.resolution
            drone_x = pos.x + flat_x * cos_yaw - flat_y * sin_yaw
            drone_y = pos.y + flat_x * sin_yaw + flat_y * cos_yaw
            return drone_x, drone_y
        list_x = []
        list_y = []
        for drone_coords in state:
            drx, dry = _transform_drones(*drone_coords)
            list_x.append(drx)
            list_y.append(dry)
        return list_x, list_y

    def publish_msgs(self,
                     obs_map, state_current, state_target, num_drones,
                     convergence_steps, score, avg_score):
        """
        Publish all messages Admiral is responsible for:
        - score map
        - status report
        - orders for the Captain

        Args:
            obs_map (np.array): current observation_map
            state_current (tuple[int, int] list): current observed state
                of the drones
            state_target (tuple[int, int] list): target state
                for the drones (orders for the Captain)
            num_drones (int): current number of drones
            convergence_steps (int): number of steps
                the algorithm took to converge
            score (float): score of the target configuration
            avg_score (float): reference average score for the past steps
        """
        self._publish_orders(state_current, state_target)
        self._publish_score(obs_map)
        self._publish_status(num_drones, state_current,
                             state_target, convergence_steps, score, avg_score)
        rospy.logdebug('Admiral messages published')

    def _publish_orders(self, state_current, state_target):
        target_xs, target_ys = self._transform_coordinates(state_target)
        current_xs, current_ys = self._transform_coordinates(state_current)

        orders = AdmiralOrders()
        orders.header.stamp = rospy.Time.now()
        orders.header.seq = self.seq
        orders.num_drones = self.params.NUM_DRONES
        orders.sight_radius = self.params.DRONE_SIGHT_RADIUS * self.map_info.resolution
        orders.period = 1./self.params.RATE

        orders.current_xs = current_xs
        orders.current_ys = current_ys
        orders.target_xs = target_xs
        orders.target_ys = target_ys

        self.orders_pub.publish(orders)

    def _publish_score(self, obs_map):
        SCALE = 255./100
        grid = OccupancyGrid()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = 'map'
        grid.header.seq = self.seq

        grid.info = self.map_info
        score_array = obs_map[::-1, :, M.SCORE] / SCALE
        score_list = list(map(int, score_array.reshape(-1)))

        grid.data = score_list
        self.score_pub.publish(grid)

    @staticmethod
    def _extract_xy(state):
        tup_x, tup_y = zip(*tuple(state))
        return list(tup_x), list(tup_y)

    def _publish_status(self,
                        num_drones, state_current, state_target,
                        convergence_steps, score, avg_score):
        curr_is, curr_js = self._extract_xy(state_current)
        target_is, target_js = self._extract_xy(state_target)

        ad_stat = AdmiralStatus()
        ad_stat.header.seq = self.seq
        ad_stat.header.stamp = rospy.Time.now()
        self.seq += 1

        ad_stat.num_drones = num_drones
        ad_stat.current_is = curr_is
        ad_stat.current_js = curr_js
        ad_stat.target_is = target_is
        ad_stat.target_js = target_js

        # ad_stat.i_coords = i_coords
        # ad_stat.j_coords = j_coords
        ad_stat.num_convergence_steps = convergence_steps
        ad_stat.score = score
        ad_stat.avg_score = avg_score

        self.status_pub.publish(ad_stat)
