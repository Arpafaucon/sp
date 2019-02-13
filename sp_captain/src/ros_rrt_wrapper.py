import rospy
import threading
import numpy as np
import math

from tf.transformations import euler_from_quaternion

from nav_msgs.msg import OccupancyGrid
from sp_core.msg import AdmiralStatus, AdmiralOrders, CaptainOrders
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import ColorRGBA
import rrt.arrts_multi


class RrtWrapperResults(object):
    def __init__(self):
        self.num_drones = None
        self.perm = None
        self.total_cost = None
        self.paths = None  # list of RRTPath objects
        self.starts = None
        self.ends = None
        self.orders = None


class RosRrtWrapper(object):

    def __init__(self, orders_publisher, viz_publisher):
        self.map_lock = threading.RLock()
        self.map = None
        self.map_sin_yaw = None
        self.map_cos_yaw = None
        self.map_orig_x = None
        self.map_orig_y = None
        self.map_resolution = None

        self.orders_lock = threading.RLock()
        self.orders = None
        self.last_orders_seq = None
        self.num_drones = None

        self.orders_pub = orders_publisher
        self.orders_seq = 0

        self.viz_pub = viz_publisher
        self.viz_seq = 0
        self.viz_scale = Vector3()
        self.viz_scale.x = 0.02
        self.viz_color = ColorRGBA()
        self.viz_color.r = self.viz_color.a = self.viz_color.g = 0.9
        # self.ordres_publisher = orders_publisher

    def set_map(self, map_msg):
        with self.map_lock:
            self.map = map_msg
            map_info = map_msg.info

            pos = map_info.origin.position
            self.map_orig_x = pos.x
            self.map_orig_y = pos.y

            # resolution = map_info.resolution
            self.map_resolution = map_info.resolution

            ori = map_info.origin.orientation
            explicit_quat = [ori.x, ori.y, ori.z, ori.w]
            yaw = euler_from_quaternion(explicit_quat)[2]
            self.map_cos_yaw = math.cos(yaw)
            self.map_sin_yaw = math.sin(yaw)

    def set_orders(self, orders_msg):
        with self.orders_lock:
            self.orders = orders_msg

        
    def spin_once_rate(self, rate_freq_hz=2):
        rate = rospy.Rate(rate_freq_hz)
        self._spin_once()
        rate.sleep()


    def _spin_once(self):
        with self.orders_lock:
            with self.map_lock:
                if self.map is None or self.orders is None:
                    rospy.logwarn_throttle(
                        10, 'Captain cannot plan path : missing map or orders')
                    return False, None
                if self.last_orders_seq == self.orders.header.seq:
                    # orders didn't change, no need to work
                    return False, None

                # let's compute !
                rospy.loginfo_throttle(1, "starting planification")

                success, results = self._planif_assign()
                self.last_orders_seq = self.orders.header.seq
                if not success:
                    rospy.logwarn_throttle(1, "RRT search did not succeed")
                else:
                    self._publish_captain_orders(results)
                    self._publish_captain_viz(results)

                

                

    def _planif_assign(self):
        map_array = self._build_map_array()
        starts, ends = self._read_admiral_orders()
        rrt_multi = rrt.arrts_multi.RRT(
            num_trees=self.num_drones, starts=starts, goals=ends, occupancyGrid=map_array, expandDis=0.5, maxIter=200)
        planning_ok, rrt_results, total_cost, perm = rrt_multi.Planning(
            animation=False)

        if not planning_ok:
            rospy.logerr(
                'RRT couldn\'t find valid path for every drone')
            return False, None

        rospy.loginfo_throttle(1, "got valid result")

        world_rrt_results = []
        for rrt_path in rrt_results:
            world_path = self._tf_rrt_result(rrt_path)
            world_rrt_results.append(world_path)
            print("{}->{} valid ! cost={}, pathlen={}"
                    .format(world_path.i_start,
                            world_path.i_end,
                            world_path.cost,
                            len(world_path.path)))

        wrap_res = RrtWrapperResults()
        wrap_res.num_drones = self.num_drones
        wrap_res.perm = perm
        wrap_res.total_cost = total_cost
        wrap_res.paths = world_rrt_results
        wrap_res.orders = self.orders
        return True, wrap_res

    def _tf_rrt_result(self, res):
        wres = rrt.arrts_multi.RRTPath(
            res.i_start, res.i_end, 0, None, valid=res.valid)
        wres.cost = res.cost * self.map_resolution
        w_path = []
        for waypoint in res.path:
            w_x, w_y = self._tf_map_to_world(waypoint[0], waypoint[1])
            w_path.append([w_x, w_y])
        wres.path = w_path
        return wres

    def _tf_world_to_map(self, x, y):
        with self.map_lock:
            cos_yaw = self.map_cos_yaw
            sin_yaw = self.map_sin_yaw

            dworld_x = x-self.map_orig_x
            dworld_y = y-self.map_orig_y
            map_x = (dworld_x * cos_yaw - dworld_y * sin_yaw) / \
                self.map_resolution
            map_y = (dworld_x * sin_yaw + dworld_y * cos_yaw) / \
                self.map_resolution

            return (map_x, map_y)

    def _tf_map_to_world(self, x, y):
        with self.map_lock:
            cos_yaw = self.map_cos_yaw
            sin_yaw = self.map_sin_yaw

            dworld_x = (x * cos_yaw + y * sin_yaw) * self.map_resolution
            dworld_y = (-x * sin_yaw + y * cos_yaw) * self.map_resolution
            world_x = self.map_orig_x + dworld_x
            world_y = self.map_orig_y + dworld_y

            return world_x, world_y

    def _build_map_array(self):
        with self.map_lock:
            map_info = self.map.info
            width = map_info.width
            height = map_info.height

            map_array = np.array(
                self.map.data, dtype=np.uint8).reshape((height, width))
            return map_array

    def _read_admiral_orders(self):
        with self.orders_lock:
            orders = self.orders
            starts = []
            ends = []
            tomap = self._tf_world_to_map

            self.num_drones = orders.num_drones
            for i in range(self.num_drones):
                startx, starty = tomap(
                    orders.current_xs[i], orders.current_ys[i])
                endx, endy = tomap(orders.target_xs[i], orders.target_ys[i])
                starts.append([startx, starty])
                ends.append([endx, endy])
            return starts, ends

    def _publish_captain_orders(self, res_wrapper):
        msg = CaptainOrders()

        msg.header.frame_id = '/map'
        msg.header.seq = self.orders_seq
        msg.header.stamp = rospy.Time.now()
        self.orders_seq += 1

        msg.num_drones = res_wrapper.num_drones
        msg.drone_association = res_wrapper.perm

        msg.current_xs = res_wrapper.orders.current_xs
        msg.current_ys = res_wrapper.orders.current_ys
        msg.target_xs = res_wrapper.orders.target_xs
        msg.target_ys = res_wrapper.orders.target_ys

        wp_x = []
        wp_y = []
        wp_start_indices = [-1] * res_wrapper.num_drones
        wp_stop_indices = [-1] * res_wrapper.num_drones
        distance = [0] * res_wrapper.num_drones
        valid = True

        for i, rrtpath in enumerate(res_wrapper.paths):
            print(rrtpath)
            valid = valid and rrtpath.valid

            idrone = rrtpath.i_start
            distance[idrone] = rrtpath.cost

            wp_start_indices[idrone] = len(wp_x)
            for point in rrtpath.path:
                wp_x.append(point[0])
                wp_y.append(point[1])
            wp_stop_indices[idrone] = len(wp_x)

        msg.waypoints_start_indices = wp_start_indices
        msg.waypoints_stop_indices = wp_stop_indices
        msg.waypoints_x = wp_x
        msg.waypoints_y = wp_y

        msg.distance = distance
        msg.total_distance = res_wrapper.total_cost
        msg.valid = valid

        self.orders_pub.publish(msg)
        rospy.loginfo('Captain: published orders')

    def _publish_captain_viz(self, res_wrapper):
        viz = Marker()
        viz.header.frame_id = '/map'
        viz.header.stamp = rospy.Time.now()
        viz.header.seq = self.viz_seq

        viz.action = viz.MODIFY
        viz.ns = 'paths'
        viz.id = 0
        viz.type = viz.LINE_LIST
        # viz.color = color
        viz.scale = self.viz_scale
        for i_drone, rrtpath in enumerate(res_wrapper.paths):
            # for each drone
            p = rrtpath.path
            for ip in range(1, len(p)):
                p_end = Point()
                p_start = Point()
                p_start.x = p[ip-1][0]
                p_start.y = p[ip-1][1]
                p_start.z = .05
                p_end.x = p[ip][0]
                p_end.y = p[ip][1]
                p_end.z = .05
                viz.points.extend([p_start, p_end])
                viz.colors.extend([self.viz_color, self.viz_color])
        # for i_d in range(res_wrapper.num_drones):
        #     p_curr = Point()
        #     p_curr.x = status.current_xs[i_d]
        #     p_curr.y = status.current_ys[i_d]
        #     p_target = Point()
        #     p_target.x = status.target_xs[i_d]
        #     p_target.y = status.target_ys[i_d]
        #     viz.points.extend([p_curr, p_target])
        #     viz.colors.extend([self.viz_color, self.viz_color])

        self.viz_pub.publish(viz)
        rospy.loginfo('Captain: published markers')

# def main2():
#     filepath = "/home/arpad/dev/sp/rosws/src/sp/sp_core/maps/map2.pgm"
#     map_array, origin_x, origin_y, upperright_x, upperright_y, resolution = load_image(
#         filepath)
#     starts = [[.5, .5], [1.5, .5], [2.5, 1]]
#     ends = [[.5, 1], [1.5, 1], [2.5, .75]]
#     rrt = RRT(num_trees=3, starts=starts, goals=ends, randArea=[
#               upperright_x, upperright_y], resolution=resolution, occupancyGrid=map_array, expandDis=.5*resolution, maxIter=400)
#     start_time = time.time()
#     rrt_results = rrt.Planning(animation=show_animation)
#     end_time = time.time()
#     print('process took {} sec'.format(end_time-start_time))
#     if rrt_results is None:
#         print("Cannot find path")
#     else:
#         print("found path!!")
#         if show_animation:
#             rrt.DrawGraph()
#             for i, rrt_path in enumerate(rrt_results):
#                 if rrt_path.valid:
#                     col = TREE_COLORS[-i%NUM_TREE_COLORS]
#                     print("{}->{} valid ! cost={}, pathlen={} [{}]".format(rrt_path.i_start, rrt_path.i_end, rrt_path.cost, len(rrt_path.path), col))
#                     # Draw final path
#                     path = rrt_path.path
#                     plt.plot([x for (x, y) in path], [y for (x, y) in path], '-', color=col)
#             plt.grid(True)
#             plt.pause(0.01)  # Need for Mac
#             plt.show()
