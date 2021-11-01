#!/bin/bash

# http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
echo "Checking for catkin workspace..."
source /opt/ros/melodic/setup.bash
if [ ! -d ~/catkin_ws/ ]
then 
echo "Setting up catkin workspace..."
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
fi

source ~/catkin_ws/devel/setup.bash

# http://wiki.ros.org/ROS/Tutorials/CreatingPackage
# Creating package
cd ~/catkin_ws/src

echo "Enter name of package: "
read PKG_NAME

echo "Enter dependencies: "
read DEPENDENCIES
# dependencies may need to change
catkin_create_pkg $PKG_NAME $DEPENDENCIES

cd ~/catkin_ws
catkin_make

source ~/catkin_ws/devel/setup.bash

# http://wiki.ros.org/ROS/Tutorials/BuildingPackages
source ~/catkin_ws/opt/ros/melodic/setup.bash

