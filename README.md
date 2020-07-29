# ROS-Project
This is a project based on path planning and control theory of robots. What this means is that I need to plan a path from the given start to goal points through a given environment using one of the path planning algorithms. I then need to create a code for a controller for the robot to traverse the path point by point to reach the goal. The environment consists of a series of static obstacles, and my robot. [Trotbot](https://github.com/ERC-BPGC/Trotbot) (a model of robot) has been used to test my code. Trotbot is based on the [Omnibase](https://github.com/ERC-BPGC/omnibase?files=1) package. Both of these are developed by the [Electronics and Robotics Club, BITS Goa](https://github.com/ERC-BPGC/). 
![Screenshot](https://github.com/Srujan-D/Robotics-Automation-QSTP-2020/blob/master/WEEK%205/Screenshot%20from%202020-07-29%2015-17-50.png)

_Screenshot from the recording of my project. The 4 wheeled robot is the Trotbot, surrounded by the cylindrical static obstacles. The simulation is seen using the Gazebo software._

**INSTRUCTIONS TO INSTALL THE REQUIRED SOFTWARES IN ORDER TO RUN THE PROJECT:**

Robotics Development relies heavily on Linux. Hece, I have used Ubuntu 18.04 to create and test my project. A software framework is needed in order to communicate with the robot. I have used ROS for this purpose. [ROS](http://wiki.ros.org/ROS/Introduction) stands for Robot Operating System, is an open-source robotics middleware (i.e. collection of software frameworks for robot software development). It is currently developed by OSRF/Open Robotics. Since I used Ubuntu 18.04, I had to use the ROS Melodic version to write the code. You can install this by following the steps given [here](http://wiki.ros.org/melodic/Installation/Ubuntu). Since testing out my code on an actual robot was not feasible, I used a robot simulation software called Gazebo. Gazebo is the most popular simulator for robotics development. It can simulate robots in a 3D environment and can be fully integrated into ROS using the gazebo_ros ROS package. One can interface the robots in the simulation using ROS and control them using ROS messages and services. Gazebo is automatically installed when you install ROS. To make sure you have all the ROS packages necessary for running Gazebo simulations are installed, run ```sudo apt-get install ros-melodic-gazebo-*``` in the terminal. The ROS framework is easy to implement in any modern programming language. The already implemented languages are [Python](http://wiki.ros.org/rospy), [C++](http://wiki.ros.org/roscpp), and [Lisp](http://wiki.ros.org/roslisp),. I have used Python to write my codes.

**SO WHAT DOES PATH PLANNING MEAN?**

![RRT](https://sites.psu.edu/zqy5086/files/2017/08/RRTsim-1nhzi69.png)

_Credits: Penn University_

Given a series of static/non-static obstacles (here: static cylindrical obstacles of radius 0.25 units and height 10 units), the robot has to traverse along a collision-free path. There are many algorithms tried and tested by researchers all over the world. The algorithms are based on trees, graphs, grids, etc. One of the most famous path planning algorithms is Rapidly Exploring Random Tree, or simply [RRT](http://msl.cs.illinois.edu/~lavalle/papers/Lav98c.pdf). Today, there are many variations of RRT, each one offering to plan a path unique in the sense of the distance of path, time required to plan a path, etc. Most famous of these are [Rapidly Exploring Random Graph (RRG), RRT-Star](http://roboticsproceedings.org/rss06/p34.pdf) and its variations. Here, I have used the fundamental RRT. Taking into account the size of the robot along with those of the obstacles, I have tweaked the algorithm to suit my needs.

***WHAT IS A CONTROLLER, AND WHY DO I NEED ONE?***

![PID](https://upload.wikimedia.org/wikipedia/commons/4/43/PID_en.svg)

_PID Controller | Credits: Wikipedia_                                                             

After my path planner plans a path, our robot will still be at the start point! To make the robot move from one place to another along any path, we need a controller. There are many types of controllers, most famous ones being PID controllers. PID (proportional integral derivative) controllers use a control loop feedback mechanism to control process variables and are the most accurate and stable controller. PID control is a well-established way of driving a system towards a target position or level. It's practically ubiquitous as a means of controlling temperature and finds application in myriad chemical and scientific processes as well as automation. PID control uses closed-loop control feedback to keep the actual output from a process as close to the target or setpoint output as possible. To learn more about PID Controllers, see these brilliant tutorials created by [MATLAB](https://www.youtube.com/playlist?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y). 

***SO WHERE DOES ROS COME IN?***

Now that we have our robot, environment of obstacles,path planner and controller, we need a framework so that all these can communicate among each other, forming a ROS network. That’s where ROS comes in. We use ROS to send commands from one code to another. We do this by using the Publisher-Subscriber, or [Pub-Sub](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29), model in ROS. "[Node](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)" is the ROS term for an executable that is connected to the ROS network. What this means is that one ROS node, say node1, sends or publishes messages on some channel. All the other nodes which want this published messages/data need to subscribe to that same channel. In ROS, this channel is termed as “[Topic](http://wiki.ros.org/Topics)”. These topics are represented as /topic_name. In this project, I have used 4 topics: 
1. '/obstacles' : carries the centres of the cylindrical obstacles to the subscriber (Path planner node).
2. ‘/path’: gives the planned path to the subscriber (Controller node). The Path planner publishes the path on this node.
3. '/odom’ : gives the current odometry (position and direction the robot is facing) to the subscriber (Controller node). The Controller Node needs the current state of the robot to calculate the new velocity.
4. ‘/cmd_vel’ : The Controller node keeps calculating velocity (linear and angular velocity) and then publishing it to this topic. Our Trotbot by default subscribes to this topic, and thus gets the command to move (command is the velocity here). 
![ROS Nodes](https://in.mathworks.com/help/examples/ros/win64/ExchangeDataWithROSPublishersAndSubscribersExample_01.png)

_Credits : MathWorks_

**STRUCTURE OF THE PROJECT:**

1. _Obstacle detector_ node which publishes a fixed list of obstacles. Here I published the list of centre point of the cylindrical obstacles.
2. _Planner_ node which subscribes to obstacles and publishes a path.
3. _Controller_ node which subscribes to the path and publishes /cmd_vel. This
controller will traverse the path one point at a time using PID Controller.

**INSTRUCTIONS TO RUN THE PROJECT:**

For running the codes, you have to clone the project’s [repo](), and the [Omnibase](https://github.com/ERC-BPGC/omnibase?files=1) repo. In the /Project/scripts, there are three files or ROS nodes - pub_obs.py, path_planner.py and pid_controller.py. The pub_obs.py node publishes the centres of the given cylindrical obstacles of radius 0.25 units and height 10 units. The path_planner.py node subscribes to the above list of obstacles, plans and publishes a path for the Trotbot to move from the start point to goal point using the Rapidly Exploring Random Trees (RRT) algorithm. The pid_controller.py node subscribes to the published path, keeps track of the robot’s odometry (position and angles in the cartesian coordinate frame), thus calculating and publishing the required rotational and translational velocities of the Trotbot on /cmd_vel topic, which results in the motion of Trotbot. Follow these steps to execute the project:
1. Clone the Omnibase repo.
2. Copy ```obstacles.world``` into ```omnibase_gazebo/worlds```.
3. Copy ```qstp_omnibase.launch``` into ```omnibase_gazebo/launch```.
4. To launch the world/environment consisting of the static cylindrical obstacles and the Trotbot, run this in the terminal:

```bash
roslaunch omnibase_gazebo qstp_omnibase.launch
```

5. To publish obstacles on the required topic, run: 

```bash
rosrun pub_obs.py
```

6. To execute the path planner, run this in the terminal: 

```bash
rosrun path_planner.py
```

7. To run the controller, run this in the terminal: 

```bash
rosrun omnibase_control pid_controller.py
```

And voila! You will see the Trotbot moving in the Gazebo world.
You can see the recording of the Trotbot going from start point (0,0,0) to goal point (6,6,0), also avoiding a series of cylindrical obstacles, by playing the Screencast_Trotbot.webm file avilable in this repo.
