#!/bin/bash


export ROS_IP=192.168.1.73
export ROS_MASTER_URI=http://192.168.1.73:11311

roslaunch realsense2_camera rs_d400_and_t265.launch
