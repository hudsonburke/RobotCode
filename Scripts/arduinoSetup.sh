#!/bin/bash

export ROS_IP=192.168.1.73
export ROS_MASTER_URI=http://192.168.1.73:11311

echo "Enter the name of the sketch folder to compile and upload"
read sketch

arduino-cli compile --fqbn arduino:avr:uno ~/RobotCode/$sketch
arduino-cli upload --port /dev/ttyACM0 --fqbn arduino:avr:uno ~/RobotCode/$sketch

rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0



