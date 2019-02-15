import rospy

from dumb_control import CAPTAIN_ORDERS_SUB
from sp_core.msg import CaptainOrders

def test_control():
    rospy.init_node("test_control")
    pub = rospy.Publisher(CAPTAIN_ORDERS_SUB, CaptainOrders, queue_size=10)
    rate = rospy.Rate(.05)

    cm = CaptainOrders()
    cm.num_drones = 2
    cm.waypoints_start_indices = [0,1]
    cm.waypoints_stop_indices = [1, 2]
    cm.waypoints_x = [0,1]
    cm.waypoints_y = [0,0]

#     cm = CaptainOrders()
#     cm.num_drones = 2
#     cm.waypoints_start_indices = [0,3]
#     cm.waypoints_stop_indices = [3, 6]
#     cm.waypoints_x = [2, 5, 2, 0, 0, 0]
#     cm.waypoints_y = [2, 2, 2, 1, 3, 1]

    while not rospy.is_shutdown():
        pub.publish(cm)
        rospy.loginfo('pub')
        rate.sleep()

if __name__ == "__main__":
    test_control()