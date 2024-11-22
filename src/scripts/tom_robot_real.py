#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt
from time import sleep

class TomChaser:
    def __init__(self, delay_start_tom):
        rospy.init_node('tom_chaser')

        # Velocity publisher for Tom
        self.velocity_publisher = rospy.Publisher('/tom/cmd_vel', Twist, queue_size=10)

        # Odometry subscriber for Tom
        self.tom_pose_subscriber = rospy.Subscriber('/tom/my_odom', Odometry, self.update_tom_pose)

        # Odometry subscriber for Jerry
        self.jerry_pose_subscriber = rospy.Subscriber('/jerry/my_odom', Odometry, self.update_jerry_pose)

        self.tom_pose = None
        self.jerry_pose = None

        # Initializing chase state
        self.chase_started = False
        self.start_time = rospy.get_time()

        # Set the delay for Tom's start (in seconds)
        self.delay_start_tom = delay_start_tom
        rospy.loginfo(f"Delaying Tom's start by {self.delay_start_tom} seconds...")

        # Timer for chasing Jerry
        self.chase_timer = rospy.Timer(rospy.Duration(0.1), self.chase_jerry)

    def update_tom_pose(self, msg):
        # Update Tom's position from odometry data
        self.tom_pose = msg.pose.pose.position

    def update_jerry_pose(self, msg):
        # Update Jerry's position from odometry data
        self.jerry_pose = msg.pose.pose.position

    def chase_jerry(self, event):
        # Wait for the delay of 20 seconds
        if not self.chase_started:
            elapsed_time = rospy.get_time() - self.start_time
            if elapsed_time < self.delay_start_tom:
                rospy.loginfo(f"Waiting for {self.delay_start_tom - elapsed_time:.2f} seconds before starting chase.")
                return
            else:
                self.chase_started = True
                rospy.loginfo("Starting the chase!")

        if not self.tom_pose or not self.jerry_pose:
            # Wait for both positions to be initialized
            return

        # Calculate the distance and angle to Jerry
        dx = self.jerry_pose.x - self.tom_pose.x
        dy = self.jerry_pose.y - self.tom_pose.y
        distance = sqrt(dx**2 + dy**2)
        angle_to_jerry = atan2(dy, dx)

        # Create a Twist message
        cmd_vel = Twist()

        if distance > 0.1:  # Threshold to stop when near Jerry
            # Linear velocity proportional to the distance (scaled to prevent too fast movement)
            cmd_vel.linear.x = min(0.5 * distance, 0.5)  # Limit max speed to 0.5 m/s

            # Angular velocity proportional to the angle difference
            cmd_vel.angular.z = 1.0 * angle_to_jerry
        else:
            # Stop if close enough
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            rospy.loginfo("Tom tagged Jerry!")

        # Publish velocity
        self.velocity_publisher.publish(cmd_vel)

if __name__ == '__main__':
    # Get the delay parameter from the launch file
    delay_start_tom = rospy.get_param('~delay_start_tom', 20.0)  # Default to 20 seconds if not specified

    tom_chaser = TomChaser(delay_start_tom)
    rospy.spin()
