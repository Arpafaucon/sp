import rospy
import numpy as np
print(__name__)
from ..src.ros_io import load_map_ROS
from ..src.support import disp_image, create_observation_map
from ..src.constants import *

def callback(data):
    rospy.logdebug('yo')

def test_load():
    rospy.init_node('test_ROSio')

    valid, obs_map = load_map_ROS('', None)
    obs_map2 = create_observation_map('src/obs_map/test/data/map1.pgm')
    if not valid:
        print('loading failed')
    # print(obs_map[:,:,M_PHY])
    # print(obs_map2[:,:,M_PHY])
    # ensure both images are equal
    print(np.all(obs_map==obs_map2))
    disp_image(obs_map)
    # disp_image(obs_map2)
    
if __name__ == "__main__":
    test_load()