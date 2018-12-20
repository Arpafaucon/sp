"""
Path Planning Sample Code with RRT*

original author: AtsushiSakai(@Atsushi_twi)

modified for SP context
- mapping environnment is 0->width;0->height : coordinates transform is done afterwards
- obstacles don't exist as such : they're full cells in occupancy[height][width], where occupancy[i][j] refers to the point (origin.x + j*resolution, origin.y + i*resolution)
- 
"""

import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt
import itertools
import time
# dev
from PIL import Image

# above this value the cell is considered full, therefore not travelable to
WALL_THRESHOLD = 50
TREE_COLORS = ['green', 'blue', 'gold', 'crimson', 'lime', 'purple', 'darkorange', 'saddlebrown', 'indigo', 'coral']
NUM_TREE_COLORS = len(TREE_COLORS)
show_animation =  True


class RRTPath:
    def __init__(self, i_start, i_end, cost, path, valid=True):
        self.i_start = i_start
        self.i_end = i_end
        self.cost = cost
        self.path = path
        self.valid = valid


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, num_trees, starts, goals, randArea, resolution, occupancyGrid,
                 expandDis=0.5, goalSampleRate=20, maxIter=500):
        """
        Setting Parameter

        start:Start Position [x,y]*num_trees
        goal:Goal Position [x,y]*num_trees
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.starts = []
        self.ends = []
        self.num_trees = num_trees
        for i in range(num_trees):
            start = starts[i]
            self.starts.append(Node(start[0], start[1]))
            goal = goals[i]
            self.ends.append(Node(goal[0], goal[1]))
        # self.start = Node(start[0], start[1])
        # self.end = Node(goal[0], goal[1])
        self.minrand_x = 0
        self.maxrand_x = randArea[0]
        self.minrand_y = 0
        self.maxrand_y = randArea[1]
        self.expandDis = expandDis
        # self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.occupancy_grid = occupancyGrid
        self.resolution = resolution

        self.sample_rate_goal = goalSampleRate / 100.

    def Planning(self, animation=True):
        """
        Pathplanning

        animation: flag for animation on or off
        """

        self.nodeList_all = [[self.starts[i]] for i in range(self.num_trees)]

        for i in range(self.maxIter):
            for i_tree in range(self.num_trees):
                rnd = self.get_random_point(i_tree)
                nind = self.GetNearestListIndex(self.nodeList_all[i_tree], rnd)

                newNode = self.steer(i_tree, rnd, nind)
                #  print(newNode.cost)

                if self.__CollisionCheck(newNode):
                    nearinds = self.find_near_nodes(newNode, i_tree)
                    newNode = self.choose_parent(newNode, i_tree, nearinds)
                    self.nodeList_all[i_tree].append(newNode)
                    self.rewire(newNode, nearinds, i_tree)

            if animation and i % 20 == 0:
                self.DrawGraph(rnd)

        # return paths for every start-dest couple
        rrt_paths_mat = [[None for i in range(self.num_trees)] for j in range(self.num_trees)]
        for i_start in range(self.num_trees):
            for i_end in range(self.num_trees):
                lastIndex = self.get_best_last_index(i_start, i_end)
                if lastIndex is None:
                    # path not found
                    rrt_path = RRTPath(i_start, i_end, 0, None, valid=False)
                else:
                    cost = self.nodeList_all[i_start][lastIndex].cost
                    path = self.gen_final_course(lastIndex, i_start, i_end)
                    rrt_path = RRTPath(i_start, i_end, cost=cost, path=path)
                rrt_paths_mat[i_start][i_end] = rrt_path

        final_assign, min_cost, perm = self._assign_paths(rrt_paths_mat)
        return final_assign
        # # generate coruse
        # lastIndex = self.get_best_last_index()
        # if lastIndex is None:
        #     return None
        # path = self.gen_final_course(lastIndex)
        # return path

    def choose_parent(self, newNode, i_tree, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        nodeList = self.nodeList_all[i_tree]
        for i in nearinds:
            dx = newNode.x - nodeList[i].x
            dy = newNode.y - nodeList[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(nodeList[i], theta, d):
                dlist.append(nodeList[i].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return newNode

        newNode.cost = mincost
        newNode.parent = minind

        return newNode

    def steer(self, i_tree, rnd, nind):

        # expand tree
        nodeList = self.nodeList_all[i_tree]
        nearestNode = nodeList[nind]
        theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
        newNode = Node(rnd[0], rnd[1])
        currentDistance = math.sqrt(
            (rnd[1] - nearestNode.y) ** 2 + (rnd[0] - nearestNode.x) ** 2)
        # Find a point within expandDis of nind, and closest to rnd
        if currentDistance <= self.expandDis:
            pass
        else:
            newNode.x = nearestNode.x + self.expandDis * math.cos(theta)
            newNode.y = nearestNode.y + self.expandDis * math.sin(theta)
        newNode.cost = float("inf")
        newNode.parent = None
        return newNode

    def get_random_point(self, i_tree):
        if random.random() > self.sample_rate_goal:
            rnd = [random.uniform(self.minrand_x, self.maxrand_x),
                   random.uniform(self.minrand_y, self.maxrand_y)]
        else:  # goal point sampling
            # goal is chosen uniformly
            i_goal = random.randint(0, self.num_trees-1)
            rnd = [self.ends[i_goal].x, self.ends[i_goal].y]
            # rnd = [self.end.x, self.end.y]

        return rnd

    def get_best_last_index(self, i_tree, i_goal):
        nodeList = self.nodeList_all[i_tree]
        disglist = [self.calc_dist_to_goal(
            node.x, node.y, i_goal) for node in nodeList]
        goalinds = [disglist.index(i) for i in disglist if i <= self.expandDis]
        #  print(goalinds)

        if len(goalinds) == 0:
            return None

        mincost = min([nodeList[i].cost for i in goalinds])
        for i in goalinds:
            if nodeList[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind, i_tree, i_goal):
        nodeList = self.nodeList_all[i_tree]
        path = [[self.ends[i_goal].x, self.ends[i_goal].y]]
        while nodeList[goalind].parent is not None:
            node = nodeList[goalind]
            path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.starts[i_tree].x, self.starts[i_tree].y])
        return path

    def calc_dist_to_goal(self, x, y, goal_index):
        return np.linalg.norm([x - self.ends[goal_index].x, y - self.ends[goal_index].y])

    def find_near_nodes(self, newNode, i_tree):
        nodeList = self.nodeList_all[i_tree]
        nnode = len(nodeList)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        #  r = self.expandDis * 5.0
        dlist = [(node.x - newNode.x) ** 2 +
                 (node.y - newNode.y) ** 2 for node in nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    def rewire(self, newNode, nearinds, i_tree):
        nodeList = self.nodeList_all[i_tree]
        nnode = len(nodeList)
        for i in nearinds:
            nearNode = nodeList[i]

            dx = newNode.x - nearNode.x
            dy = newNode.y - nearNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = newNode.cost + d

            if nearNode.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(nearNode, theta, d):
                    nearNode.parent = nnode - 1
                    nearNode.cost = scost

    def check_collision_extend(self, nearNode, theta, d):

        tmpNode = copy.deepcopy(nearNode)

        for i in range(int(d / self.expandDis)):
            tmpNode.x += self.expandDis * math.cos(theta)
            tmpNode.y += self.expandDis * math.sin(theta)
            if not self.__CollisionCheck(tmpNode):
                return False

        return True

    def DrawGraph(self, rnd=None):
        """
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for i_tree in range(self.num_trees):
            nodeList = self.nodeList_all[i_tree]
            for node in nodeList:
                if node.parent is not None:
                    plt.plot([node.x, nodeList[node.parent].x], [
                        node.y, nodeList[node.parent].y], "-", linewidth=.25, color=TREE_COLORS[i_tree % NUM_TREE_COLORS])

        # for (ox, oy, size) in self.obstacleList:
        #     plt.plot(ox, oy, "ok", ms=30 * size)
        plt.imshow(self.occupancy_grid, cmap='binary', origin='lower',
                   extent=(0, self.maxrand_x, 0, self.maxrand_y))

        for i_tree in range(self.num_trees):
            plt.plot(self.starts[i_tree].x, self.starts[i_tree].y, "xr")
            plt.plot(self.ends[i_tree].x, self.ends[i_tree].y, "xr")

        plt.axis([-self.resolution, self.maxrand_x+self.resolution, -
                  self.resolution, self.maxrand_y+self.resolution])
        plt.grid(True)
        plt.pause(0.01)

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))

        return minind

    def __CollisionCheck(self, node):
        cell_i, cell_j = self._get_map_indices(node.x, node.y)
        cost = self.occupancy_grid[cell_i, cell_j]
        if cost > WALL_THRESHOLD:
            return False
        return True
        # for (ox, oy, size) in obstacleList:
        #     dx = ox - node.x
        #     dy = oy - node.y
        #     d = dx * dx + dy * dy
        #     if d <= size ** 2:
        #         return False  # collision
        # return True  # safe

    def _get_map_indices(self, x_target, y_target):
        xc_target = int(x_target / self.resolution)
        yc_target = int(y_target / self.resolution)
        return (yc_target, xc_target)

    def _assign_paths(self, rrt_paths_mat):
        cost_matrix = np.full((self.num_trees, self.num_trees), fill_value=np.inf)
        for i_start in range(self.num_trees):
            for i_end in range(self.num_trees):
                path = rrt_paths_mat[i_start][i_end]
        # for path in rrt_paths:
                if path.valid:  
                    start = path.i_start
                    end = path.i_end
                    cost = path.cost
                    cost_matrix[start][end] = cost
        print(cost_matrix)
        min_cost = np.inf
        min_perm = None
        for perm in itertools.permutations(range(self.num_trees)):
            cost_vector = [cost_matrix[i][perm[i]] for i in range(self.num_trees)]
            cost = sum(cost_vector)
            if cost < min_cost:
                # new min
                min_cost = cost
                min_perm = perm
        
        final_assign = []
        for i_start in range(self.num_trees):
            assigned_goal = min_perm[i_start]
            final_assign.append(rrt_paths_mat[i_start][assigned_goal])
        
        return (final_assign, min_cost, min_perm)



class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None
        self.tree = 0


def test_image(filepath):

    def check_obstacle(ogrid, x_orig, y_orig, resolution, x_target, y_target):
        height_cells, width_cells = np.shape(ogrid)
        y_max = y_orig + height_cells*resolution
        x_max = x_orig + width_cells*resolution
        xc_target = int((x_target - x_orig - 0*resolution/2) / resolution)
        yc_target = int((y_target - y_orig - 0*resolution/2) / resolution)
        cost = ogrid[yc_target, xc_target]
        return cost > 50

    map_array, origin_x, origin_y, upperright_x, upperright_y, resolution = load_image(
        filepath)

    plt.imshow(map_array, cmap='binary', origin='lower',
               extent=(origin_x, upperright_x, origin_y, upperright_y))

    # check_obstacle()
    for _ in range(600):
        xt = random.uniform(origin_x, upperright_x)
        yt = random.uniform(origin_y, upperright_y)
        obst = check_obstacle(map_array, origin_x,
                              origin_y, resolution, xt, yt)
        color = 'r' if obst else 'g'
        plt.plot([xt], [yt], color + '+')

    plt.show()


def load_image(filepath):
    floorplan = Image.open(filepath)
    map_array = 100./256 * (256 - np.array(floorplan)[::-1])
    height_cells, width_cells = np.shape(map_array)
    resolution = .1
    upperright_x = width_cells*resolution
    upperright_y = height_cells*resolution
    return (map_array, 0, 0, upperright_x, upperright_y, resolution)


# def main():
#     print("Start rrt planning")

#     # ====Search Path with RRT====
#     obstacleList = [
#         (5, 5, 1),
#         (3, 6, 2),
#         (3, 8, 2),
#         (3, 10, 2),
#         (7, 5, 2),
#         (9, 5, 2)
#     ]  # [x,y,size(radius)]

#     # Set Initial parameters
#     rrt = RRT(start=[0, 0], goal=[10, 10],
#               randArea=[-2, 15], obstacleList=obstacleList)
#     path = rrt.Planning(animation=show_animation)

#     if path is None:
#         print("Cannot find path")
#     else:
#         print("found path!!")

#         # Draw final path
#         if show_animation:
#             rrt.DrawGraph()
#             plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
#             plt.grid(True)
#             plt.pause(0.01)  # Need for Mac
#             plt.show()


def main2():
    filepath = "/home/arpad/dev/sp/rosws/src/sp/sp_core/maps/map2.pgm"
    map_array, origin_x, origin_y, upperright_x, upperright_y, resolution = load_image(
        filepath)
    starts = [[.5, .5], [1.5, .5], [2.5, 1]]
    ends = [[.5, 1], [1.5, 1], [2.5, .75]]
    rrt = RRT(num_trees=3, starts=starts, goals=ends, randArea=[
              upperright_x, upperright_y], resolution=resolution, occupancyGrid=map_array, expandDis=.5*resolution, maxIter=200)
    start_time = time.time()
    rrt_results = rrt.Planning(animation=show_animation)
    end_time = time.time()
    print('process took {} sec'.format(end_time-start_time))
    if rrt_results is None:
        print("Cannot find path")
    else:
        print("found path!!")
        if show_animation:
            rrt.DrawGraph()
            for i, rrt_path in enumerate(rrt_results):
                if rrt_path.valid:
                    col = TREE_COLORS[-i%NUM_TREE_COLORS]
                    print("{}->{} valid ! cost={}, pathlen={} [{}]".format(rrt_path.i_start, rrt_path.i_end, rrt_path.cost, len(rrt_path.path), col))
                    # Draw final path
                    path = rrt_path.path
                    plt.plot([x for (x, y) in path], [y for (x, y) in path], '-', color=col)
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()


if __name__ == '__main__':
    # test_image("/home/arpad/dev/sp/rosws/src/sp/sp_core/maps/map2.pgm")
    main2()
