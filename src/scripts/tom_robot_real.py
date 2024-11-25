#!/usr/bin/env python

import rospy
import math
import tf2_ros
import geometry_msgs.msg
from math import atan2, sqrt

class TomChaser:
    def __init__(self, delay_start_tom):
        # Initialize the ROS node
        rospy.init_node('tom_chaser', anonymous=True)

        # Velocity publisher for Tom
        self.velocity_publisher = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

        # Set the delay for Tom's start (in seconds)
        self.delay_start_tom = delay_start_tom
        rospy.loginfo(f"Delaying Tom's start by {self.delay_start_tom} seconds...")

        # Timer for chasing Jerry
        self.chase_started = False
        self.start_time = rospy.get_time()

        # Initialize tf2 buffer and listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

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
            # Try to get the transform between 'odom' and 'tom', 'odom' and 'jerry'
            rospy.loginfo("Waiting for transform...")
            trans_tom = self.tfBuffer.lookup_transform('odom', 'tom', rospy.Time(), rospy.Duration(1.0))
            trans_jerry = self.tfBuffer.lookup_transform('odom', 'jerry', rospy.Time(), rospy.Duration(1.0))

            # Log that transforms were successfully received
            rospy.loginfo(f"Transform from 'odom' to 'tom' received: {trans_tom}")
            rospy.loginfo(f"Transform from 'odom' to 'jerry' received: {trans_jerry}")

            # Compute the difference in position between Tom and Jerry
            dx = trans_jerry.transform.translation.x - trans_tom.transform.translation.x
            dy = trans_jerry.transform.translation.y - trans_tom.transform.translation.y
            distance = sqrt(dx**2 + dy**2)
            angle_to_jerry = atan2(dy, dx)

            # Create a Twist message for velocity
            cmd_vel = geometry_msgs.msg.Twist()

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

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF2 lookup failed: {e}, waiting for transforms...")

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
