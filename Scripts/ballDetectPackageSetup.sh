#!/bin/bash

# http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
source /opt/ros/melodic/setup.bash

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

source devel/setup.bash

# http://wiki.ros.org/ROS/Tutorials/CreatingPackage
cd ~/catkin_ws/src

# dependencies may need to change
catkin_create_pkg ball_detect std_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs cv_bridge rospy roscpp

cd ~/catkin_ws
catkin_make

. ~/catkin_ws/devel/setup.bash

# edit package.xml

# http://wiki.ros.org/ROS/Tutorials/BuildingPackages
source /opt/ros/melodic/setup.bash

