import rospy
print(__name__)
from ..src.ros_io import load_map_ROS
from ..src.support import disp_image, create_observation_map

def callback(data):
    rospy.logdebug('yo')

def test_load():
    rospy.init_node('test_ROSio')

    valid, obs_map = load_map_ROS('', None)
    obs_map2 = create_observation_map('/home/greg/')
    if not valid:
        print('loading failed')
    disp_image(obs_map)
    
if __name__ == "__main__":
    test_load()