from PIL import Image
import numpy as np
import rospy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from sp_msgs.msg import AdmiralStatus, AdmiralOrders
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
import math
# from

from .support import _prepare_map
from constants import *


DEFAULT_PARAM_NS = "/sp/admiral"
NODE_NAME = "admiral_node"

STATUS_TOPIC = "admiral_status"
SCORE_TOPIC = "admiral_score"
ORDERS_TOPIC = "admiral_orders"

QUEUE_SIZE = 5

class AdmiralRosInterface(object):
    def __init__(self):
        """
        We expect the node to be already setup (via setup_node)

        Args:
            object ([type]): [description]
            channel_name (str, optional): Defaults to 'sp/admiral_order'. [description]
            queue_size (int, optional): Defaults to 10. [description]
        """
        self._setup_node()
        self.status_pub = rospy.Publisher(
            "/sp/"+STATUS_TOPIC, AdmiralStatus, queue_size=QUEUE_SIZE)
        self.score_pub = rospy.Publisher(
            "/sp/"+SCORE_TOPIC, OccupancyGrid, queue_size=QUEUE_SIZE)
        self.orders_pub = rospy.Publisher(
            "/sp/"+ORDERS_TOPIC, AdmiralOrders, queue_size=QUEUE_SIZE)
        self.seq = 0
        # init in load_params
        self.params = self._load_params_ROS()
        # init in load_map_ROS
        self.map_info = None

    def _setup_node(self, name=NODE_NAME):
        rospy.init_node(name=name)

    def load_map_ROS(self, map_topic_root, wall_radius=4, timeout=4):
        map_topic = '{}/map'.format(map_topic_root)
        # meta_topic = '{}/map_metadata'.format(map_topic_root)
        try:
            # meta_msg = rospy.wait_for_message(meta_topic, MapMetaData, timeout=timeout)
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
        obs_map[:, :, M_CONST] = 32
        # the occupancy grid has a normal frame coordinate system
        # therefore the y axis must be reversed to get the image CS
        obs_map[::-1, :, M_PHY] = data
        _prepare_map(obs_map, wall_radius)
        return (True, obs_map)

    def _load_params_ROS(self):
        try:
            # check if namespace was defined as a private param
            absolute_ns = rospy.get_param("~namespace")
        except KeyError as _:
            absolute_ns = DEFAULT_PARAM_NS
        rno = absolute_ns + "/obs_map/"
        num_drones = rospy.get_param(rno+'num_drones')
        wall_radius = rospy.get_param(rno+'wall_radius')
        initial_temp = rospy.get_param(rno+'initial_temp')
        n_iterations = rospy.get_param(rno+'n_iterations')
        drone_sight_radius = rospy.get_param(rno+'drone_sight_radius')
        rate = rospy.get_param(rno+'rate')
        params = Parameters(num_drones, wall_radius, initial_temp,
                            n_iterations, drone_sight_radius, rate)
        rospy.loginfo('loaded params : {}'.format(params))
        self.params = params
        return params

    def get_params(self):
        return self.params

    def _transform_coordinates(self, state):
        ori = self.map_info.origin.orientation
        pos = self.map_info.origin.position
        explicit_quat = [ori.x, ori.y, ori.z, ori.w]
        yaw = euler_from_quaternion(explicit_quat)[2]
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        def transform_drones(drone_i, drone_j):
            flat_x = (drone_j+.5) * self.map_info.resolution
            flat_y = (self.map_info.height - drone_i - .5) * self.map_info.resolution
            drone_x = pos.x + flat_x * cos_yaw - flat_y * sin_yaw
            drone_y = pos.y + flat_x * sin_yaw + flat_y * cos_yaw
            return drone_x, drone_y
        list_x = []
        list_y = []
        for drone_coords in state:
            drx, dry  = transform_drones(*drone_coords)
            list_x.append(drx)
            list_y.append(dry)
        return list_x, list_y

    def publish_msg(self, obs_map, state, num_drones, convergence_steps, score, avg_score):
        self._publish_orders(state)
        self._publish_score(obs_map)
        self._publish_status(num_drones, state, convergence_steps, score, avg_score)
        rospy.logdebug('Admiral messages published')


    def _publish_orders(self, state):
        list_x, list_y = self._transform_coordinates(state)
        orders = AdmiralOrders()
        orders.header.stamp = rospy.Time.now()
        orders.header.seq = self.seq
        orders.num_drones = self.params.NUM_DRONES
        orders.sight_radius = self.params.DRONE_SIGHT_RADIUS * self.map_info.resolution
        orders.x_coords = list_x
        orders.y_coords = list_y
        self.orders_pub.publish(orders)
        

    def _publish_score(self, obs_map):
        SCALE = 255./100
        grid = OccupancyGrid()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = 'map'
        grid.header.seq = self.seq
        # grid.frame_id = 'map'
        h, w, _ = obs_map.shape
        grid.info = self.map_info
        score_array = obs_map[::-1,:,M.SCORE] / SCALE
        score_list = list(map(int, score_array.reshape(-1)))
        
        grid.data = score_list
        self.score_pub.publish(grid)

    def _publish_status(self, num_drones, ij_coords, convergence_steps, score, avg_score):
        i_coords = []
        j_coords = []
        for di, dj in ij_coords:
            i_coords.append(di)
            j_coords.append(dj)

        mes = AdmiralStatus()
        mes.header.seq = self.seq
        mes.header.stamp = rospy.Time.now()
        self.seq += 1

        mes.num_drones = num_drones
        mes.i_coords = i_coords
        mes.j_coords = j_coords
        mes.num_convergence_steps = convergence_steps
        mes.score = score
        mes.avg_score = avg_score

        self.status_pub.publish(mes)

