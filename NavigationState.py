#!/usr/bin/env python


# https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
# http://edu.gaitech.hk/turtlebot/map-navigation.html#writing-the-navigation-program
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class CTFStateMachine():
    def __init__(self):
        states = ["NavigateToBallRegion", "GrabBall", "NavigateToBall"]

        self.state = "NavigateToBallRegion"
        self.gripperClosed = False
        self.ballGrabbed = False

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # self.goal = MoveBaseGoal()
        # self.goal.target_pose.header.frame_id = "map"
        # self.goal.target_pose.header.stamp = rospy.Time.now()
        # self.goal.target_pose.pose.position.x = 0
        # self.goal.target_pose.pose.position.y = 0
        # self.goal.target_pose.pose.orientation.w = 1.0  # no rotation
        
    
    def navigate(self, x, y, frame="map"):
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = frame
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.orientation.w = 1.0  # no rotation
        
        self.client.send_goal(self.goal)

                

    def navToBallRegion(self, x=0, y=0):
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.orientation.w = 1.0  # no rotation

        self.client.send_goal(self.goal)


    def grabBall(self):
        self.gripperClosed = False # open gripper

        #navigate to ball's location relative to robot
        self.goal.target_pose.header.frame_id = "base_link"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = # read ball location
        self.goal.target_pose.pose.position.y = # read ball location
        self.goal.target_pose.pose.orientation.w = 1.0  # no rotation

        self.client.send_goal(self.goal)

        if ball in position: # not sure what to do with this
            self.gripperClosed = True # close gripper around ball


    def navToGoalRegion(self, x=4, y=5):
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.orientation.w = 1.0  # no rotation

        self.client.send_goal(self.goal)


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        machine = CTFStateMachine()
        while not rospy.is_shutdown():

            wait = machine.client.wait_for_result()
            # If the result doesn't arrive, assume the Server is not available
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                # Result of executing the action
                result = machine.client.get_result()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
