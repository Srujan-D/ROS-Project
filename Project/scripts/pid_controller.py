#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from std_msgs.msg import Float32MultiArray
from math import pow, atan2, sqrt
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
import time
from path_nav.msg import points
 
class Trotbot:
 
    def __init__(self, current_time=None):
        rospy.init_node('pid_controller', anonymous = True)
        self.x = 0.0
        self.y = 0.0
        self.flag = False             # flag value will be used for switching between rotation and translation      
        self.theta = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.goal_points = Path()
        self.goal = Point()

        self.new_x = self.goal.x - self.x       # this will be an input to actuator response
        self.new_y = self.goal.y - self.y       # this will be an input to actuator response
        self.n_points = 0                       # number of points in the path   Z
        self.n = points()
        self.n.x = [0.0]
        self.n.y = [0.0]
        self.sub = rospy.Subscriber('/path', points, self.get_path)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.get_odom)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.rate = rospy.Rate(50)              # 50 Hz
    
        self.Kp = 1.0                           # Default Proportionality Constant                            
        self.Ki = 0.0                           # Default Integral Constant
        self.Kd = 0.0                           # Default Derivative Constant
        self.feedback_value = 0.0               # This will help us in controlling the actuator response  
        self.SetPoint = 100                     # We will compare our feedback_value to the SetPoint value
        self.current_time = current_time if current_time is not None else time.time()  # We will use time difference of short intervals to calculate Integral and Derivative Terms
        self.last_time = self.current_time      # After each iteration we will update the last_time and current time

        self.PTerm = 0.0                        # Default Proportionality Term
        self.ITerm = 0.0                        # Default Integral Term
        self.DTerm = 0.0                        # Default Derivative Term
        self.last_error = 0.0                   # After each iteration we will update the last_error.       
        self.error = 1000.0
        # Windup Guard - In case the integral term accumulates a significant error during the rise (windup), thus overshooting.
        self.windup_guard = 1000000000.0             

    def set_pid(self):
        self.PTerm = 0.0                        # Default Proportionality Term
        self.ITerm = 0.0                        # Default Integral Term
        self.DTerm = 0.0                        # Default Derivative Term
        self.last_error = 0.0                   # After each iteration we will update the last_error.       
        self.error = 0.0
        
    def get_path(self, request):
        self.n = request
        print("in callback 1")
        rospy.loginfo(self.n)
        self.n_points = len(self.n.x)
        
            
    def get_odom(self, request):
        self.turtle = request.pose.pose.orientation
        self.x = request.pose.pose.position.x   # Current x coordinate of turtlebot
        self.y = request.pose.pose.position.y   # Current y coordinate of turtlebot
        
        (self.roll, self.pitch, self.theta) = euler_from_quaternion([self.turtle.x, self.turtle.y, self.turtle.z, self.turtle.w])   # We will only use the yaw value, or the angle (theta) with z axis.
        print("in callback 2")
        rospy.loginfo(self.theta)
    
    def dist(self, new_y, new_x):               # Calculates distance
        return sqrt(pow((new_x), 2) + pow((new_y), 2))

    def turn_angle(self, new_y, new_x):     # Calculates the angle by which the turtlebot has to turn
        return (atan2(new_y, new_x))

    def calculate(self, new_y, new_x, Kp, Ki, Kd, feedback_value, current_time = None): # Calculates Angular Velocity
        self.error = (self.SetPoint - feedback_value)
        
        current_time = current_time if current_time is not None else time.time()
        self.delta_time = (current_time - self.last_time)
        self.delta_error = self.error - self.last_error
        self.PTerm = self.error
        self.ITerm += self.error * self.delta_time
            
        if (self.ITerm < -self.windup_guard):
            self.ITerm = -self.windup_guard
        elif (self.ITerm > self.windup_guard):
            self.ITerm = self.windup_guard

        self.DTerm = 0.0
        if self.delta_time > 0:
            self.DTerm = self.delta_error / self.delta_time

        # Update the last time and last error for next iteration
        self.last_time = self.current_time
        self.last_error = self.error

        self.output = (Kp * self.PTerm) + (Ki * self.ITerm) + (Kd * self.DTerm)  # Output is the updated angular or linear velocity
        return self.output

    def cal_easy(self, new_y, new_x, Kp, Ki, Kd, feedback_value, current_time = None):
        self.PTerm = self.SetPoint - feedback_value
        return Kp*self.PTerm
       
    def move2goal(self, current_time=None):     # Function that commands the turtlebot
        
        vel_msg = Twist()
        i = 0
        self.goal.x = self.n.x[i]
        self.goal.y = self.n.y[i]
        self.new_x = self.goal.x - self.x
        self.new_y = self.goal.y - self.y
        
        while not rospy.is_shutdown(): 
            if(i < self.n_points) :
                self.goal.x = self.n.x[i]
                self.goal.y = self.n.y[i]
            
            else:
                vel_msg.angular.z = 0
                vel_msg.linear.x = 0
                self.pub.publish(vel_msg)
            
            self.new_x = self.goal.x - self.x
            self.new_y = self.goal.y - self.y
            while(((self.dist(self.new_y, self.new_x)) > 0.1) and (i < self.n_points)):
                self.goal.x = self.n.x[i]
                self.goal.y = self.n.y[i]
                self.new_x = self.goal.x - self.x
                self.new_y = self.goal.y - self.y
                     
                if((self.flag == False) and (round(abs(self.SetPoint-(self.theta)), 3) >= 0.005)): # Precision upto 3 decimal places
                    print("Rotating")
                    if (i == 0):
                        self.SetPoint = atan2(self.goal.y, self.goal.x)
                    else:
                        self.SetPoint = atan2(self.goal.y - self.n.y[i-1], self.goal.x - self.n.x[i-1])
                    print(self.SetPoint)
                    self.feedback_value = (self.theta)
                    print("----------")
                    print(vel_msg.angular.z)
                    print("----------")
                    rospy.loginfo(self.feedback_value)
                    
                    self.Kp = 1
                    self.Kd = 0 #0.00001  Recommended value in case PID contoller is used
                    self.Ki = 0 #0.00001  Recommended value in case PID contoller is used
                    vel_msg.linear.x = 0
                    #vel_msg.angular.z = self.calculate(self.new_y, self.new_x, self.Kp, self.Ki, self.Kd, self.feedback_value)   PID Controller
                    vel_msg.angular.z = self.cal_easy(self.new_y, self.new_x, self.Kp, self.Ki, self.Kd, self.feedback_value)     # Simple P Controller
                else :
                    if(self.flag == False ):
                        vel_msg.angular.z = 0.0
                        vel_msg.linear.x = 0.0
                               
                    self.flag = True
                    vel_msg.angular.z = 0.0
                self.pub.publish(vel_msg)
                    
                if ((self.flag == True) and (self.dist(self.new_y, self.new_x) > (0.1 + i/20))):   # Precision is 0.1
                    print("Translating")
                    if (i == 0):
                        self.SetPoint = self.dist(self.goal.y, self.goal.x)
                        self.feedback_value = self.dist(self.y, self.x)
                    else:
                        self.SetPoint = self.dist(self.goal.y - self.n.y[i-1], self.goal.x - self.n.x[i-1])
                        self.feedback_value = self.dist(self.y - self.n.y[i-1], self.x - self.n.x[i-1])
                    print("--------")
                    print(self.SetPoint)
                    print(self.goal)
                    print(self.feedback_value)
                    print("........")
                    
                    self.Kp = 1
                    self.Kd = 0 #0.000001 Recommended value in case PID contoller is used
                    self.Ki = 0 #0.000001 Recommended value in case PID contoller is used
                    vel_msg.angular.z = 0.0
                    #vel_msg.linear.x = self.cal_easy(self.new_y, self.new_x, self.Kp, self.Ki, self.Kd, self.feedback_value)       PID Controller
                    vel_msg.linear.x = self.cal_easy(self.new_y, self.new_x, self.Kp, self.Ki, self.Kd, self.feedback_value)        # Simple P Controller
                    print("===========")
                    print(vel_msg.linear.x) 
                    print("===========")           
                else:
                    if(self.flag == True):
                        vel_msg.linear.x = 0.0
                        
                    self.flag = False
                    vel_msg.linear.x = 0
                self.pub.publish(vel_msg)
                    
                

            
            if((i < self.n_points)) :
               self.set_pid()
               i += 1
                
            elif (i >= self.n_points):
                vel_msg.angular.z = 0
                vel_msg.linear.x = 0
                self.pub.publish(vel_msg)
            
            self.rate.sleep()

        rospy.spin()

 
if __name__ == '__main__':
    try:
        o = Trotbot()
        o.move2goal()
    except rospy.ROSInterruptException:
        pass

