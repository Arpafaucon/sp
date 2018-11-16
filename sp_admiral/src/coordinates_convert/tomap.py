import rospy
from map_msgs.msg import MapMetaData

frame_id = ''
position = None
orientation = None
resolution = None

def load_map_ROS(map_topic_root, wall_radius=4, time_bonus=1, timeout=4):

    meta_topic = '{}/map_metadata'.format(map_topic_root)
    try:
        meta_msg = rospy.wait_for_message(meta_topic, MapMetaData, timeout=timeout)
    except rospy.exceptions.ROSException as ex:
        rospy.logerr('loading failed with: {}'.format(ex))
        return (False, None)

    frame_id = meta_msg.header.frame_id
    position = meta_msg.position
    orientation = meta_msg.orientation
    resolution = meta_msg.resolution