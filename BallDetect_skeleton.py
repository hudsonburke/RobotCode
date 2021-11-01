#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from math import pi
import roslib
#roslib.load_manifest('my_package')
import cv2
import rospy
import sys
import imutils
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import pdb
import tf


bridge = CvBridge()


class Tracker3D():
    
    center_pixel = (320,240)

    ################
    # <raw_image> : Raw image topic
    # <point_cloud> : POint cloud from depth camera
    ################
    
    def __init__(self,img_topic_name=<raw_image>,depth_topic_name=<point_cloud>,see_image=False):
        
        self.image_sub = rospy.Subscriber(img_topic_name,Image,self.image_cb)
        self.depth_sub = rospy.Subscriber(depth_topic_name,Image,self.depth_cb)

        self.ballloc_pixel = [0,0]
        self.ballloc_xyz = [0,0,0]

        self.cv_image = None
        self.depth_image = None
        self.K = []
        self.Rt = np.array([[1,0,0],[0,0,1],[0,-1,-0]])
        self.mask = None
        self.center = None

        self.listener = tf.TransformListener()
        
        

        # Wait for messages to be published on image and depth topics
        print("Waiting for image and depth topic")
        rospy.wait_for_message(img_topic_name,Image)
        rospy.wait_for_message(depth_topic_name,Image)
        print("-----> Messages received")

        self.rate = rospy.Rate(20)


    def get_depth(self):
        # Function to get depth of the ball
        pass


    def get_xyz(self):
        # Function to compute the x,y,z coordinates of the ball
        pass


    def pub_viz(self):
        # Publish the marker for the ball
        pub = rospy.Publisher('markedImage', Image)
        rospy.init_node('pub_viz', anonymous=True)
        while not rospy.is_shutdown():
            pub.publish(self.cv_image)
        

    def image_cb(self,data):
        try:
		    self.cv_image = bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image_hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV)
	    

	    # Define lower and upper range of the colors to generate a mask using hsv 
        sensitivity = 15
        lower_green = np.array([60-sensitivity, 100, 100])
        upper_green = np.array([60+sensitivity, 255, 255])
        mask = cv2.inRange(cv_image_hsv, lower_green, upper_green)

        # Process your mask to reduce noise
        self.mask = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)

	    # find contours in the mask and initialize the current
	    # (x, y) center of the ball and publish the (x,y) pixel coordinates of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	    	cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        # only proceed if at least one contour was found
        if cnts:
            # loop over the contours
            for c in cnts:
                # compute the center of the contour https://www.pyimagesearch.com/2016/02/01/opencv-center-of-contour/
                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                # draw the contour and center of the shape on the image
                cv2.drawContours(self.cv_image, [c], -1, (0, 255, 0), 2)
                cv2.circle(self.cv_image, (cX, cY), 7, (255, 255, 255), -1)
            self.center = (cX, cY)


    def depth_cb(self,data):
        try:
            self.depth_image = bridge.imgmsg_to_cv2(data,"16UC1")
        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":
    rospy.init_node("measure_3d")
    tracker = Tracker3D()
    viz_img = True
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Publish the (x,y) coordinates of the ball in the pixel coordinates
        tracker.image_cb()
        tracker.pub_viz()
        rate.sleep()