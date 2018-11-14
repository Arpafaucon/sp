from PIL import Image
import numpy as np
import rospy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from sp_msgs.msg import admiral_status


from constants import *

def setup_node(name='sp_admiral_node'):
    rospy.init_node(name=name)

def load_map_ROS(map_topic_root, ros_node, wall_radius=4, time_bonus=1, timeout=4):

    map_topic = '{}/map'.format(map_topic_root)
    meta_topic = '{}/map_metadata'.format(map_topic_root)
    try:
        meta_msg = rospy.wait_for_message(meta_topic, MapMetaData, timeout=timeout)
        map_msg = rospy.wait_for_message(map_topic, OccupancyGrid, timeout=timeout)
    except rospy.exceptions.ROSException as ex:
        print('loading failed with: {}'.format(ex))
        return (False, None)
        
    width = meta_msg.width
    height = meta_msg.height
    data = np.array(map_msg.data, dtype=np.uint8).reshape((height, width))
    # invert in the occupancy grid, 0 is free and 100 is wall
    # we want : 255 is free and 0 is wall (more 'visual')
    data[data==0]=255
    data[data==100]=0

    obs_map = np.zeros((height, width, 3), dtype=np.uint8)
    obs_map[:,:,M_CONST] = 32
    # the occupancy grid has a normal frame coordinate system
    # therefore the y axis must be reversed to get the image CS
    obs_map[::-1,:,M_PHY] = data
    return (True, obs_map)

class AdmiralRosInterface(object):
    def __init__(self, channel_name='sp/admiral_order', queue_size=10):
        """
        We expect the node to be already setup (via setup_node)
        
        Args:
            object ([type]): [description]
            channel_name (str, optional): Defaults to 'sp/admiral_order'. [description]
            queue_size (int, optional): Defaults to 10. [description]
        """
        self.pub = rospy.Publisher(channel_name, admiral_status, queue_size=queue_size)
        self.seq = 0
    
    def publish_status(self, num_drones, ij_coords, convergence_steps, score, avg_score):
        flattened_coords = []
        for d in range(num_drones):
            for i in [0,1]:
                flattened_coords.append(ij_coords[d][i])


        mes = admiral_status()
        mes.header.seq = self.seq
        mes.header.stamp = rospy.Time.now()
        self.seq += 1

        mes.num_drones = num_drones
        mes.xy_coords = flattened_coords
        mes.num_convergence_steps = convergence_steps
        mes.score = score
        mes.avg_score = avg_score

        self.pub.publish(mes)


