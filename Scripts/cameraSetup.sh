#!/bin/bash

export ROS_IP=192.168.1.73
export ROS_MASTER_URI=http://192.168.1.73:11311

chmod 777 /dev/video0

rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _pixel_format:=yuyv

