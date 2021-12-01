#!/usr/bin/env python


# https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
# http://edu.gaitech.hk/turtlebot/map-navigation.html#writing-the-navigation-program
import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, PoseStamped


class CTFStateMachine():

    def __init__(self):
        self.states = ["NavigateToBallRegion",
            "GrabBall", "NavigateToGoalRegion", "Done"]

        self.state = "NavigateToBallRegion"

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.gripper_pub = rospy.Publisher("gripper", Twist, queue_size=10)
        self.drive_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        
        self.goal_pub = rospy.Publisher("move_base/current_goal", PoseStamped, queue_size=100)

        self.override = False
        
        
    def override_cb(self, data):
        if(data.linear.x != 0.0 and self.override == False):
            rospy.loginfo("Changing override to True.")
            self.override = True
            self.drive(0,0)
        else:
            rospy.loginfo("Changing override to False.")
            self.override = False
    
    def gripper(self, open):
        if open:
            rospy.loginfo("Opening gripper!")
        else:
            rospy.loginfo("Closing gripper!")
        msg = Twist()
        msg.linear.x = 1.0 if open else 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.gripper_pub.publish(msg)

    def drive(self, forward=0.0, turn=0.0):
        msg = Twist()
        msg.linear.x = forward
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = turn
        self.drive_pub.publish(msg)

    def navigate(self, x, y, frame="map"):
        # self.goal = PoseStamped()
        # self.goal.header.frame_id = frame
        # self.goal.header.stamp = rospy.Time.now()
        # self.goal.pose.position.x = x
        # self.goal.pose.position.y = y
        # self.goal.pose.orientation.w = 1.0

        # self.goal_pub.publish(self.goal)

        ##OR##


        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = frame
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.orientation.w = 1.0  # no rotation

        self.client.send_goal(self.goal)

        # Wait for server to finish
        wait = self.client.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            result = self.client.get_result()
            if result:
                rospy.loginfo("Goal execution done!")
                return True  # could use this to determine if complete

    def navToBallRegion(self, x=0, y=0):
        self.navigate(x, y)
        self.state = "GrabBall" #TODO: Uncomment this
        # detect how close to region we are to enter grabBall state?

    def grabBall(self):
        # launch BallDetect (or just start reading from it)

        # open gripper
        self.gripper(True)

        # get ball coords
        x = rospy.get_param("ball_x")
        y = rospy.get_param("ball_y")
        rospy.loginfo("Ball is at (" + str(x) + ", " + str(y) + ").")
        # if ball seen:
        if x != 0 and y != 0:
            # navigate to ball
            self.navigate(x+1, y+1, "base_link")
            # close gripper
            self.gripper(False)
            self.state = "NavigateToGoalRegion"
        else:
            # spin
            rospy.loginfo("Can't see the ball.")
            self.drive(turn=0.2)

    def navToGoalRegion(self, x=5.5, y=-4.5):
        self.navigate(x, y)
        self.state = "Done"


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        machine = CTFStateMachine()
        override_sub = rospy.Subscriber("/override", Twist, machine.override_cb)
        while not rospy.is_shutdown():
            # add in check for rqt topics
            if not machine.override: 
                if machine.state == "NavigateToBallRegion":
                    machine.navToBallRegion()
                elif machine.state == "GrabBall":
                    machine.grabBall()
                elif machine.state == "NavigateToGoalRegion":
                        machine.navToGoalRegion()
                else:
                    # Stop
                    machine.drive(forward=0.0, turn=0.0) 
            else: 
                rospy.loginfo("Switching to telop!")
                while machine.override:
                    continue
                machine.state = "NavigateToGoalRegion" #TODO: if statements to switch based on prev state
                rospy.loginfo("Switching back to autonomous")      

    except rospy.ROSInterruptException:
        rospy.loginfo("Node Terminated.")
