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
**rqt** rqt_graph which shows the nodes and topics currently running.
**rostopic** The rostopic tool allows you to get information about ROS topics.
**rostopic type**  returns the message type of any topic being published.
**rostopic pub** publishes data on to a topic currently advertised.
**rostopic hz** reports the rate at which data is published. 
**rqt_plot** displays a scrolling time plot of the data published on topics.
**rosservice** Services are another way that nodes can communicate with each other. Services allow nodes to send a request and receive a response.
**rosparam** allows you to store and manipulate data on the ROS Parameter Server. The Parameter Server can store integers, floats, boolean, dictionaries, and lists. rosparam uses the YAML markup language for syntax.
**rqt_console** attaches to ROS's logging framework to display output from nodes.
**rqt_logger_level** allows us to change the verbosity level (DEBUG, WARN, INFO, and ERROR) of nodes as they run.
**roslaunch** starts nodes as defined in a launch file.
**rosed** is part of the rosbash suite. It allows you to directly edit a file within a package by using the package name rather than having to type the entire path to the package.
**msg** files are simple text files that describe the fields of a ROS message. They are used to generate source code for messages in different languages.
**srv** file describes a service. It is composed of two parts: a request and a response.
_______________________________________________
### ROS general practices
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

$ rosrun rqt_graph rqt_graph

$ rostopic -h

$ rostopic echo <topic>
$ rostopic list -h
$ rostopic type [topic]
//show detail of a message
$ rosmsg show [The message]

$ rostopic pub [topic] [msg_type] [args]
//Example: rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'

$ rostopic hz [topic]

$ rosrun rqt_plot rqt_plot

$ rosservice list
$ rosservice type [service]
$ rosservice call [service] [args]

$rosparam list (set, get, ...)

$ rosrun rqt_console rqt_console
$ rosrun rqt_logger_level rqt_logger_level

$ roslaunch [package] [filename.launch]
//create a launch file
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roscd beginner_tutorials
$ mkdir launch
$ cd launch
$ vim sample.launch (and modify it as you need)

$ roslaunch <package_name> sample.launch
$ rostopic pub ...

$ rosed [package_name] [filename]
```
_______________________________________________
### making messages and services
```
//Making a msg
$ roscd beginner_tutorials
$ mkdir msg
$ echo "int64 num" > msg/Num.msg

//Open package.xml, and make sure these two lines are in it and uncommented:
#  <build_depend>message_generation</build_depend>
#  <exec_depend>message_runtime</exec_depend>

//On CmakeLists.txt
# Add message_generation to find_package(
# add CATKIN_DEPENDS message_runtime to catkin_package(
# On add_message_files, Uncomment it by removing the # symbols and then replace the stand in Message*.msg files with your .msg file, such that it
# Uncomment the generate_messages() function

$ rosmsg show [message type]

//Creating a srv
$ roscd beginner_tutorials
$ mkdir srv
//we will copy an existing one from another package.
$ roscp [package_name] [file_to_copy_path] [copy_path]

//Open package.xml, and make sure these two lines are in it and uncommented:
#  <build_depend>message_generation</build_depend>
#  <exec_depend>message_runtime</exec_depend>

//On CmakeLists.txt
# Add message_generation to find_package(
# add CATKIN_DEPENDS message_runtime to catkin_package(
# On add_service_files, Uncomment it by removing the # symbols and then replace the stand in Message*.msg files with your .msg file, such that it
# Uncomment the generate_messages() function
$ rossrv show <service type>
# In your catkin workspace

$ roscd beginner_tutorials
$ cd ../..
$ catkin_make install
$ cd -
```
_______________________________________________
### Writing a Simple Publisher and Subscriber (C++)
```
# roscd beginner_tutorials
$ mkdir -p src
$ vim src/talker.cpp
$ vim src/listener.cpp

//add these few lines to the bottom of your CMakeLists.txt:

# add_executable(talker src/talker.cpp)
# target_link_libraries(talker ${catkin_LIBRARIES})
# add_dependencies(talker beginner_tutorials_generate_messages_cpp)

# add_executable(listener src/listener.cpp)
# target_link_libraries(listener ${catkin_LIBRARIES})
# add_dependencies(listener beginner_tutorials_generate_messages_cpp)

$ cd ~/catkin_ws
$ catkin_make

//Running the publisher and subscriber
$ roscore
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosrun beginner_tutorials talker
$ rosrun beginner_tutorials listener 
```
_______________________________________________
## Writting a service client
```
$ roscd beginner_tutorials
//Create the src/add_two_ints_server.cpp
//Create the src/add_two_ints_client.cpp

//Add the following to the ~/catkin_ws/src/beginner_tutorials/CMakeLists.txt 
# add_executable(add_two_ints_server src/add_two_ints_server.cpp)
# target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
# add_dependencies(add_two_ints_server beginner_tutorials_gencpp)
# add_executable(add_two_ints_client src/add_two_ints_client.cpp)
# target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
# add_dependencies(add_two_ints_client beginner_tutorials_gencpp)

$ cd ~/catkin_ws
$ catkin_make

$ roscore
$ rosrun beginner_tutorials add_two_ints_server
$ rosrun beginner_tutorials add_two_ints_client
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
_______________________________________________
## mimic.launch Sample
```
<launch>

  <group ns="turtlesim1">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <group ns="turtlesim2">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
  </node>

</launch>

```
_______________________________________________
### publisher.cpp sample

```
#lude "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**
* This tutorial demonstrates simple sending of messages over the ROS system.
*/
int main(int argc, char **argv)
{
/**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
ros::init(argc, argv, "talker");

/**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
ros::NodeHandle n;

/**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

ros::Rate loop_rate(10);

/**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
int count = 0;
while (ros::ok())
{
   /**
   * This is a message object. You stuff it with data, and then publish it.
   */
   std_msgs::String msg;

   std::stringstream ss;
   ss << "hello world " << count;
   msg.data = ss.str();

   ROS_INFO("%s", msg.data.c_str());

   /**
   * The publish() function is how you send messages. The parameter
   * is the message object. The type of this object must agree with the type
   * given as a template parameter to the advertise<>() call, as was done
   * in the constructor above.
      */
   chatter_pub.publish(msg);

   ros::spinOnce();

   loop_rate.sleep();
   ++count;
   }


   return 0;
}
```
_______________________________________________
### subscriber.cpp sample
```
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
```
_______________________________________________
### sample service node
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"

bool add(beginner_tutorials::AddTwoInts::Request  &req,
        beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
_______________________________________________
### Sample client node

#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
_______________________________________________
### References

[Reference ros.org (Click here)][http://wiki.ros.org/ROS/Tutorials]
