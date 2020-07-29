#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, Float64
from path_nav.msg import points, mylist
import math
import random

global obs_in_path
obs_in_path = []
global rate

def get_obs(request):                       # callback function
    global obs_in_path
    obs_in_path = request

rospy.init_node('path_planner', anonymous = True)
sub = rospy.Subscriber('/obstacles', mylist, get_obs)
pub = rospy.Publisher('/path', points, queue_size = 10)
rate = rospy.Rate(50)                       # 50 Hz

class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path = points()
            self.parent = None

    def __init__(self, start, goal, obstacle_list, rand_area, radius, 
                 expand_dis=3.0, path_resolution=0.5, goal_sample_rate=5, max_iter=500):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y],...]
        randArea:Random Sampling Area [min,max]
        """
        self.start = self.Node(start[0], start[1])  #Start Position
        self.end = self.Node(goal[0], goal[1])      #End/Goal Position
        self.min_rand = rand_area[0]                #randArea : Random Sampling Area [min_rand, max_rand]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.path = points()
        self.radius = radius
    def planning(self):
        """
        rrt path planning
        """

        self.node_list = [self.start]
        for _ in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_index = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_index]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision(new_node, self.obstacle_list, self.radius):
                self.node_list.append(new_node)
  
            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list, self.radius):
                    return self.generate_final_course(len(self.node_list) - 1)
  
        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path.x = [new_node.x]
        new_node.path.y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(int(n_expand)):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path.x.append(new_node.x)
            new_node.path.y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path.x.append(to_node.x)
            new_node.path.y.append(to_node.y)

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        
        path = points()

        path.x = [self.end.x]
        path.y = [self.end.y]

        node = self.node_list[goal_ind]
        while node.parent is not None:
            
            path.x.append(node.x)
            path.y.append(node.y)
            node = node.parent
        
        path.x.append(node.x)
        path.y.append(node.y)
        path.x.reverse()
        path.y.reverse()
        del path.x[0]
        del path.y[0]
        
        print("--------------")
        print(path.x)
        print(path.y)
        print("..............")
        return path
        
    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(random.uniform(self.min_rand, self.max_rand), random.uniform(self.min_rand, self.max_rand))
        else:                       # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd                  #random node

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y)** 2 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind               #minimum index

    @staticmethod
    def check_collision(node, obstacleList, radius):

        if node is None:
            return False

        for (ox, oy) in obstacleList:
            dx_list = [ox - x for x in node.path.x]
            dy_list = [oy - y for y in node.path.y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= 7*(radius ** 2):
                return False                    # collision

        return True                             # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


def main(gx = 6.0, gy = 6.0):       #Goal point is (6.0, 6.0) Start point is origin
    print("start " + __file__)
    radius = 0.25
    # ====Search Path with RRT====
    global obs_in_path
    obstacleList = obs_in_path #obs_in_path = [(0, 1.5), (0, 3), (0, 4.5), (3.0, 0), (3.0, 1.5), (3.0, 3), (3.0, 4.5), (1.5, 0), (1.5, 1.5), (1.5, 3), (1.5,4.5), (4.5, 0), (4.5, 1.5), (4.5, 3), (4.5,4.5)] 
    # Set Initial parameters
    rrt = RRT(start=[0, 0], goal=[gx, gy], rand_area=[0, 6], obstacle_list=obstacleList, radius = radius)
    
    path = rrt.planning()


    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")
        while not rospy.is_shutdown():
            
            global rate
            pub.publish(path)
            rate.sleep()

        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
