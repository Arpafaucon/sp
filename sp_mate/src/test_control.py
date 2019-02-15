import rospy

from dumb_control import CAPTAIN_ORDERS_SUB
from sp_core.msg import CaptainOrders

def test_control():
    rospy.init_node("test_control")
    pub = rospy.Publisher(CAPTAIN_ORDERS_SUB, CaptainOrders, queue_size=10)
    rate = rospy.Rate(.1)

#     cm = CaptainOrders()
#     cm.num_drones = 2
#     cm.waypoints_start_indices = [0,1]
#     cm.waypoints_stop_indices = [1, 2]
#     cm.waypoints_x = [0,1]
#     cm.waypoints_y = [0,0]

    cm = CaptainOrders()
    cm.num_drones = 2
    cm.waypoints_start_indices = [0,4]
    cm.waypoints_stop_indices = [4, 8]
    cm.waypoints_x = [0, 1, 1, 0, 2, 3, 3, 2]
    cm.waypoints_y = [-1, -1, 0, 0, 1, 1, 2, 2]

    while not rospy.is_shutdown():
        pub.publish(cm)
        rospy.loginfo('pub')
        rate.sleep()

if __name__ == "__main__":
    test_control()