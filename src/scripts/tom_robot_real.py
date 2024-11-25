#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import tf
from math import atan2, sqrt

class TomChaser:
    def __init__(self, delay_start_tom):
        # Initialize the ROS node
        rospy.init_node('tom_chaser', anonymous=True)

        # Velocity publisher for Tom
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Set the delay for Tom's start (in seconds)
        self.delay_start_tom = delay_start_tom
        rospy.loginfo(f"Delaying Tom's start by {self.delay_start_tom} seconds...")

        # Timer for chasing Jerry
        self.chase_started = False
        self.start_time = rospy.get_time()

        # Initialize the TF listener
        self.listener = tf.TransformListener()

    def chase_jerry(self):
        # Wait for the delay before starting the chase
        if not self.chase_started:
            elapsed_time = rospy.get_time() - self.start_time
            if elapsed_time < self.delay_start_tom:
                rospy.loginfo(f"Waiting for {self.delay_start_tom - elapsed_time:.2f} seconds before starting chase.")
                return
            else:
                self.chase_started = True
                rospy.loginfo("Starting the chase!")

        try:
            # Wait until the transform between Tom and Jerry is available
            self.listener.waitForTransform('odom', 'tom', rospy.Time(), rospy.Duration(1.0))
            self.listener.waitForTransform('odom', 'jerry', rospy.Time(), rospy.Duration(1.0))

            # Get the transform from 'odom' to 'tom' and 'odom' to 'jerry'
            (trans_tom, rot_tom) = self.listener.lookupTransform('odom', 'tom', rospy.Time(0))
            (trans_jerry, rot_jerry) = self.listener.lookupTransform('odom', 'jerry', rospy.Time(0))

            # Compute the difference in position between Tom and Jerry
            dx = trans_jerry[0] - trans_tom[0]
            dy = trans_jerry[1] - trans_tom[1]
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

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF lookup failed, waiting for the transforms...")

    def start_chasing(self):
        # Loop to continuously chase Jerry
        rate = rospy.Rate(10)  # 10 Hz loop
        while not rospy.is_shutdown():
            self.chase_jerry()
            rate.sleep()

if __name__ == '__main__':
    try:
        # Get the delay parameter from the launch file (default to 20 seconds if not specified)
        delay_start_tom = rospy.get_param('~delay_start_tom', 20.0)
        tom_chaser = TomChaser(delay_start_tom)
        tom_chaser.start_chasing()
    except rospy.ROSInterruptException:
        pass
