from PIL import Image
import numpy as np
import rospy
from nav_msgs.msg import MapMetaData, OccupancyGrid

from constants import *

def load_map_ROS(map_topic_root, ros_node, wall_radius=4, time_bonus=1, timeout=4):
    counter = 3
    done = False

    # def callback(data):
    #     global counter
    #     print(data)
    #     counter-=1
    #     if not counter:
    #         done = True

    map_topic = '{}/map'.format(map_topic_root)
    meta_topic = '{}/map_metadata'.format(map_topic_root)
    # map_sub = rospy.Subscriber(map_topic,
    #                  OccupancyGrid, callback=callback)
    # meta_sub = rospy.Subscriber(meta_topic,
                    #  MapMetaData, callback=callback)
    try:
        meta_msg = rospy.wait_for_message(meta_topic, MapMetaData, timeout=timeout)
        map_msg = rospy.wait_for_message(map_topic, OccupancyGrid, timeout=timeout)
    except rospy.exceptions.ROSException as ex:
        print('loading failed with: {}}'.format(ex))
        return (False, None)
    else:
        print('loading succedded')
        # print(meta_msg)
        # meta_msg = MapMetaData.deserialize(meta_msg)
        
    width = meta_msg.width
    height = meta_msg.height
    data = np.array(map_msg.data).reshape(( width, height))
    obs_map = np.zeros((height, width, 3))
    obs_map[:,:,M_CONST] = 32
    obs_map[:,:,M_PHY] = data.transpose()*255/100
    print(obs_map)
    return (True, obs_map)


        # print('-----------------')
        # print(map_msg)
    # rospy.spin()

    # # floorplan = Image.open(map_filename)
    # dynamic_map = floorplan.convert("RGB").convert("HSV")
    # dyn_map_array = np.array(dynamic_map)
    # dyn_map_array[:, :, M_CONST] = 32
    # print(dyn_map_array.shape)
    # return dyn_map_array
