#!/usr/bin/python
import roslib
import rospy

import tf
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Quaternion


class TfMapBroadcaster(object):
    def __init__(self):
        rospy.init_node("map_tf_broadcaster")

        self.map_tf_set = False
        self.position_tf = None
        self.orientation_tf = None

        self.rate = rospy.Rate(1)

        self.meta_sub = rospy.Subscriber(
            "/map_metadata", MapMetaData, callback=self.cb_metadata)

        self.tf_broadcaster = tf.TransformBroadcaster()

    def cb_metadata(self, msg):
        rospy.loginfo("map tf broadcaster got map metadata")
        pos_o = msg.origin.position
        self.position_tf = (pos_o.x, pos_o.y, 0)
        ori_o = msg.origin.orientation
        self.orientation_tf = (ori_o.x, ori_o.y, ori_o.z, ori_o.w)
        self.map_tf_set = True

    # def pub_tf(self):
    #     self.tf_broadcaster.sendTransform(self.position_tf,
    #                                       self.orientation_tf,
    #                                       rospy.Time.now(),
    #                                       "/world",
    #                                       "/map")
    def pub_tf(self):
        self.tf_broadcaster.sendTransform((0,0,0),
                                          (0,0,0,1),
                                          rospy.Time.now(),
                                          "/world",
                                          "/map")

    def spin(self):
        while not rospy.is_shutdown():
            if self.map_tf_set:
                self.pub_tf()
            self.rate.sleep()


def rosmain():
    tf_map_bc = TfMapBroadcaster()
    tf_map_bc.spin()


if __name__ == '__main__':
    rosmain()
