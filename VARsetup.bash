#!/bin/bash
echo "STARTING SETUP FOR ENVIROMENT VARIABLES"
IP="192.168.1"
read -p "Enter your computer's unique IP address (73, 110, 111 or 114) : " UNIQUE_IP 
IP="${IP}.${UNIQUE_IP}"
export ROS_IP="${IP}"
echo "Your computer's ROS_IP is "${ROS_IP}""

echo "Making the NUC the master: ROS_MASTER_URI=http://192.168.1.73:11311"
export ROS_MASTER_URI=http://192.168.1.73:11311

echo -e "\e[93mROS_IP: \e[38;5;82m$ROS_IP \e[0m"
echo -e "\e[93mROS_MASTER_URI: \e[38;5;82m$ROS_MASTER_URI \e[0m"
