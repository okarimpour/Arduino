# Practices in ROS

## Cheatsheet

**rospack:** allows you to get information about packages.

**roscd:** is part of the rosbash suite. It allows you to change directory 

**pwd:** Now let's print the working directory using the Unix command

**rosls:** is part of the rosbash suite. It allows you to ls directly in a package by name rather than by absolute path.

**rospack:** When using catkin_create_pkg earlier, a few package dependencies were provided. These first-order dependencies can now be reviewed with the rospack tool.

**Nodes:** A node is an executable that uses ROS to communicate with other nodes.

**Messages:** ROS data type used when subscribing or publishing to a topic.

**Topics:** Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.

**Master:** Name service for ROS (i.e. helps nodes find each other)

**rosout:** ROS equivalent of stdout/stderr

**roscore:** Master + rosout + parameter server (parameter server will be introduced later)
'''
rospy = python client library
roscpp = c++ client library
'''

**roscore:** is the first thing you should run when using ROS.

**rosnode:** Open up a new terminal, and let's use rosnode to see what running roscore did

**rosrun:** allows you to use the package name to directly run a node within a package (without having to know the package path).(One powerful feature of ROS is that you can reassign Names from the command-line __name:=new_name)
_______________________________________________
```
# Create a ROS workspace:

$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ echo $ROS_PACKAGE_PATH

# Create a new package:
$ catkin_create_pkg <package_name> [depend1] [depend2] [depend3]

# Build a catkin workspace and sourcing: 
$ cd ~/catkin_ws
$ catkin_make
$ . ~/catkin_ws/devel/setup.bash

# build a catkin package (In a catkin workspace)
$ catkin_make
$ catkin_make install  # (optionally)

$roscore

# New terminal
$ rosnode list
$ rosnode info /rosout

$ rosrun [package_name] [node_name] __name:=<new_name>
$ rosnode ping my_turtle
```
_______________________________________________
## In a CMake project
```
$ mkdir build
$ cd build
$ cmake ..
$ make
$ make install  # (optionally)
```
_______________________________________________
## package.xml Sample
```
   1 <?xml version="1.0"?>
   2 <package format="2">
   3   <name>beginner_tutorials</name>
   4   <version>0.1.0</version>
   5   <description>The beginner_tutorials package</description>
   6 
   7   <maintainer email="you@yourdomain.tld">Your Name</maintainer>
   8   <license>BSD</license>
   9   <url type="website">http://wiki.ros.org/beginner_tutorials</url>
  10   <author email="you@yourdomain.tld">Jane Doe</author>
  11 
  12   <buildtool_depend>catkin</buildtool_depend>
  13 
  14   <build_depend>roscpp</build_depend>
  15   <build_depend>rospy</build_depend>
  16   <build_depend>std_msgs</build_depend>
  17 
  18   <exec_depend>roscpp</exec_depend>
  19   <exec_depend>rospy</exec_depend>
  20   <exec_depend>std_msgs</exec_depend>
  21 
  22 </package>
```

