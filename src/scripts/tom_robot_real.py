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

        # Velocity publishers for Tom and Jerry
        self.tom_velocity_publisher = rospy.Publisher('/rafael/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
        self.jerry_velocity_publisher = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

        # Delay before Tom starts chasing Jerry
        self.delay_start_tom = delay_start_tom
        rospy.loginfo(f"Delaying Tom's start by {self.delay_start_tom} seconds...")

        # Timer for starting the chase
        self.chase_started = False
        self.start_time = rospy.get_time()

        # TF2 buffer and listener to get transforms
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def chase_jerry(self):
        # Wait for the delay before starting the chase
        if not self.chase_started:
            elapsed_time = rospy.get_time() - self.start_time
            if elapsed_time < self.delay_start_tom:
                rospy.loginfo(f"Waiting for {self.delay_start_tom - elapsed_time:.2f} seconds before starting the chase.")
                return
            else:
                self.chase_started = True
                rospy.loginfo("Tom is now chasing Jerry!")

        try:
            # Get the transform between 'odom' and both robots
            trans_tom = self.tfBuffer.lookup_transform('odom', 'rafael/base_footprint', rospy.Time(), rospy.Duration(1.0))
            trans_jerry = self.tfBuffer.lookup_transform('odom', 'base_footprint', rospy.Time(), rospy.Duration(1.0))

            # Calculate the difference in positions
            dx = trans_jerry.transform.translation.x - trans_tom.transform.translation.x
            dy = trans_jerry.transform.translation.y - trans_tom.transform.translation.y
            distance = sqrt(dx**2 + dy**2)
            angle_to_jerry = atan2(dy, dx)

            # Create a Twist message for Tom's velocity
            cmd_vel_tom = geometry_msgs.msg.Twist()

            # Stop if Tom is close enough to Jerry
            if distance <= 0.1:  # Safe distance to stop
                cmd_vel_tom.linear.x = 0.0
                cmd_vel_tom.angular.z = 0.0

                # Stop Jerry as well
                cmd_vel_jerry = geometry_msgs.msg.Twist()
                cmd_vel_jerry.linear.x = 0.0
                cmd_vel_jerry.angular.z = 0.0

                self.tom_velocity_publisher.publish(cmd_vel_tom)
                self.jerry_velocity_publisher.publish(cmd_vel_jerry)

                rospy.loginfo("Tom tagged Jerry! Both have stopped.")
                rospy.signal_shutdown("Tag complete")  # Stop the node
            else:
                # Chase Jerry: Set linear and angular velocities
                cmd_vel_tom.linear.x = min(0.5 * distance, 0.5)  # Limit max speed to 0.5 m/s
                cmd_vel_tom.angular.z = 1.5 * angle_to_jerry  # Adjust angular velocity for precise turning

                self.tom_velocity_publisher.publish(cmd_vel_tom)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF2 lookup failed: {e}, waiting for valid transforms...")

    def start_chasing(self):
        # Main loop to continuously chase Jerry
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
