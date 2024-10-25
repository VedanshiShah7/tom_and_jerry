#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# the code is supposed to load the map, then goes to the target coordinate
# actionlib - roswiki documentation: http://wiki.ros.org/actionlib

def move_to_goal(x, y):
    # Initialize the ROS node
    rospy.init_node('navigate_to_goal', anonymous=True)

    # Create an action client for the move_base action server
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    # Wait for the action server to start
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server")

    # Create a goal to send to the robot
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"  # Use the map frame
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the target coordinates
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0  # No rotation

    # Send the goal to the move_base action server
    rospy.loginfo("Sending goal: (%f, %f)" % (x, y))
    client.send_goal(goal)

    # Wait for the result
    client.wait_for_result()
    
    # Check if the robot reached the goal
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Reached the target coordinate!")
    else:
        rospy.loginfo("Failed to reach the target coordinate.")

if __name__ == '__main__':
    try:
        # Load the map before moving to the goal
        # Replace with the actual coordinates of your target
        target_x = 2.0  # Change to your target x coordinate
        target_y = 1.0  # Change to your target y coordinate
        move_to_goal(target_x, target_y)
    except rospy.ROSInterruptException:
        pass
