#!/bin/bash

set -e

#source ~/RobotCode/Scripts/dualCamera.bash

#gnome-terminal 

export ROS_IP=192.168.1.73
export ROS_MASTER_URI=http://192.168.1.73:11311

roslaunch rtabmap_ros rtabmap.launch \
   args:="-d --Mem/UseOdomGravity true --Optimizer/GravitySigma 0.3" \
   odom_topic:=/t265/odom/sample \
   frame_id:=t265_link \
   rgbd_sync:=true \
   depth_topic:=/d400/aligned_depth_to_color/image_raw \
   rgb_topic:=/d400/color/image_raw \
   camera_info_topic:=/d400/color/camera_info \
   approx_rgbd_sync:=false \
   visual_odometry:=false

#http://wiki.ros.org/rtabmap_ros/Tutorials/HandHeldMapping