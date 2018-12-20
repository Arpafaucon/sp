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
# dev
from PIL import Image
WALL_THRESHOLD = 50 # above this value the cell is considered full, therefore not travelable to
show_animation = True


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, randArea, resolution, occupancyGrid,
                 expandDis=0.5, goalSampleRate=20, maxIter=500):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.minrand_x = 0
        self.maxrand_x = randArea[0]
        self.minrand_y = 0
        self.maxrand_y = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.occupancy_grid = occupancyGrid
        self.resolution  = resolution

    def Planning(self, animation=True):
        """
        Pathplanning

        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        for i in range(self.maxIter):
            rnd = self.get_random_point()
            nind = self.GetNearestListIndex(self.nodeList, rnd)

            newNode = self.steer(rnd, nind)
            #  print(newNode.cost)

            if self.__CollisionCheck(newNode):
                nearinds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode, nearinds)
                self.nodeList.append(newNode)
                self.rewire(newNode, nearinds)

            if animation and i % 5 == 0:
                self.DrawGraph(rnd)

        # generate coruse
        lastIndex = self.get_best_last_index()
        if lastIndex is None:
            return None
        path = self.gen_final_course(lastIndex)
        return path

    def choose_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:
            dx = newNode.x - self.nodeList[i].x
            dy = newNode.y - self.nodeList[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(self.nodeList[i], theta, d):
                dlist.append(self.nodeList[i].cost + d)
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

    def steer(self, rnd, nind):

        # expand tree
        nearestNode = self.nodeList[nind]
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

    def get_random_point(self):

        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [random.uniform(self.minrand_x, self.maxrand_x),
                   random.uniform(self.minrand_y, self.maxrand_y)]
        else:  # goal point sampling
            rnd = [self.end.x, self.end.y]

        return rnd

    def get_best_last_index(self):

        disglist = [self.calc_dist_to_goal(
            node.x, node.y) for node in self.nodeList]
        goalinds = [disglist.index(i) for i in disglist if i <= self.expandDis]
        #  print(goalinds)

        if len(goalinds) == 0:
            return None

        mincost = min([self.nodeList[i].cost for i in goalinds])
        for i in goalinds:
            if self.nodeList[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind):
        path = [[self.end.x, self.end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeList)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        #  r = self.expandDis * 5.0
        dlist = [(node.x - newNode.x) ** 2 +
                 (node.y - newNode.y) ** 2 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    def rewire(self, newNode, nearinds):
        nnode = len(self.nodeList)
        for i in nearinds:
            nearNode = self.nodeList[i]

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
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-g")

        # for (ox, oy, size) in self.obstacleList:
        #     plt.plot(ox, oy, "ok", ms=30 * size)
        plt.imshow(self.occupancy_grid, cmap='binary', origin='lower',
               extent=(0, self.maxrand_x, 0, self.maxrand_y))

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-self.resolution, self.maxrand_x+self.resolution, -self.resolution, self.maxrand_y+self.resolution])
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
        xc_target = int( x_target  / self.resolution)
        yc_target = int( y_target  / self.resolution)
        return (yc_target, xc_target)


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


def test_image(filepath):

    def check_obstacle(ogrid, x_orig, y_orig, resolution, x_target, y_target):
        height_cells, width_cells = np.shape(ogrid)
        y_max = y_orig + height_cells*resolution
        x_max = x_orig + width_cells*resolution
        xc_target = int( (x_target - x_orig - 0*resolution/2) / resolution)
        yc_target = int( (y_target - y_orig - 0*resolution/2) / resolution)
        cost = ogrid[yc_target, xc_target]
        return cost > 50

    map_array, origin_x, origin_y, upperright_x, upperright_y, resolution = load_image(filepath)

    plt.imshow(map_array, cmap='binary', origin='lower',
               extent=(origin_x, upperright_x, origin_y, upperright_y))

    # check_obstacle()
    for _ in range(600):
        xt = random.uniform(origin_x, upperright_x)
        yt = random.uniform(origin_y, upperright_y)
        obst = check_obstacle(map_array, origin_x, origin_y, resolution, xt, yt)
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


def main():
    print("Start rrt planning")

    # ====Search Path with RRT====
    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2)
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    rrt = RRT(start=[0, 0], goal=[10, 10],
              randArea=[-2, 15], obstacleList=obstacleList)
    path = rrt.Planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt.DrawGraph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()



def main2():
    filepath = "/home/arpad/dev/sp/rosws/src/sp/sp_core/maps/map2.pgm"
    map_array, origin_x, origin_y, upperright_x, upperright_y, resolution = load_image(filepath)
    rrt = RRT(start=[.5,.5], goal=[1.5, 1.5], randArea=[upperright_x, upperright_y], resolution=resolution, occupancyGrid=map_array, expandDis=.5*resolution)
    path = rrt.Planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt.DrawGraph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()

if __name__ == '__main__':
    # test_image("/home/arpad/dev/sp/rosws/src/sp/sp_core/maps/map2.pgm")
    main2()