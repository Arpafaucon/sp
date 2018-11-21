#!/usr/bin/python
# coding: utf8
import rospy

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from sp_core.msg import CaptainStatus
from geometry_msgs.msg import Vector3, Point

# target is red

QUEUE_SIZE = 2

class CaptainViz(object):
    def __init__(self):
        rospy.init_node('captain_viz')
        self.viz_pub = rospy.Publisher('/sp/captain_viz', Marker, queue_size=QUEUE_SIZE)
        rospy.Subscriber('/sp/captain_status', CaptainStatus, self._status_callback)
        self.seq = 0
        self.scale = Vector3()  
        self.scale.x = .05
        self.curr_color = ColorRGBA()
        self.curr_color.g = self.curr_color.a = 0.9
        self.target_color = ColorRGBA()
        self.target_color.r = self.target_color.a = .9
        

    def _status_callback(self, status):
        viz = Marker()
        viz.header.frame_id = '/map'
        viz.header.stamp = rospy.Time.now()
        viz.header.seq = self.seq

        viz.action = viz.MODIFY
        viz.ns = 'paths'
        viz.id = 0
        viz.type = viz.LINE_LIST
        # viz.color = color
        viz.scale = self.scale
        for i_d in range(status.num_drones):
            p_curr = Point()
            p_curr.x = status.current_xs[i_d]
            p_curr.y = status.current_ys[i_d]
            p_target = Point()
            p_target.x = status.target_xs[i_d]
            p_target.y = status.target_ys[i_d]
            viz.points.extend([p_curr, p_target])
            viz.colors.extend([self.curr_color, self.target_color])
        self.viz_pub.publish(viz)
        rospy.loginfo('Captain: published markers')

def rosmain():
    cviz = CaptainViz()
    rospy.spin()

if __name__ == "__main__":
    rosmain()

