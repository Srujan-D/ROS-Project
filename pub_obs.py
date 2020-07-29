#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
import sys
import numpy as np 
from path_nav.msg import points, mylist

rospy.init_node('pub_obs', anonymous = True)

class Publish():
    def __init__(self):
        
        self.goals = Path()
        self.goals.header.frame_id = "/qstp_omnibase"
        self.goal_path = PoseStamped()
        self.goal = Point()
        self.n_point = 0
        self.obs = []
        self.rate = rospy.Rate(50) # 50 Hz
        self.pub = rospy.Publisher('/obstacles', mylist, queue_size = 10)
        self.flag = True
        self.j = 0
    def Publish_obs(self):
        self.n_point = int(input("Enter number of obstacles in Path : "))
        if(self.flag == True):
            for _ in range(self.n_point):
                """
                Each obstacle has radius 0.25 units and height 10 units
                Uncomment lines 33-37 in case the location of obstacles ([x,y]) is different from the ones mentioned in line 38, and comment line 38. Type any non-zero number when prompted to enter number of obstacles.
                """
                #self.goal.x = float(input("Enter x coordinate of centre of obstacle : "))         
                #self.n_x = self.goal.x                                                             
                #self.goal.y = float(input("Enter y coordinate of centre of obstacle : "))         
                #self.n_y = self.goal.y                                                             
                #self.obs += [(self.n_x, self.n_y)]                                                 
                self.obs = [(0, 1.5), (0, 3), (0, 4.5), (3.0, 0), (3.0, 1.5), (3.0, 3), (3.0, 4.5), (1.5, 0), (1.5, 1.5), (1.5, 3), (1.5,4.5), (4.5, 0), (4.5, 1.5), (4.5, 3), (4.5,4.5)]  
            print("...........")
            self.flag = False    
        

        while (not rospy.is_shutdown()):
            
            rospy.loginfo(self.obs)
            self.pub.publish(self.obs)
            self.rate.sleep()

if __name__ == "__main__":
    o = Publish()
    o.Publish_obs()
