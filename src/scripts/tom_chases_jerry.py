#!/usr/bin/env python

import rospy
import math
import geometry_msgs.msg
from math import atan2, sqrt

class TomChasesJerry:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('tom_chases_jerry', anonymous=True)

        # Velocity publishers for Tom and Jerry
        self.tom_velocity_publisher = rospy.Publisher('/rafael/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
        self.jerry_velocity_publisher = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

        self.jerry_odom_sub = rospy.Subscriber('/jerry_odom', geometry_msgs.msg.Point, self.jerry_odom_cb)
        self.tom_odom_sub = rospy.Subscriber('/tom_odom', geometry_msgs.msg.Point, self.tom_odom_cb)

        # Current x, y, and yaw of the Jerry robot.
        self.jerry_cur_x = None
        self.jerry_cur_y = None
        self.jerry_cur_yaw = None
        # Start x, y of the Jerry robot
        self.jerry_start_x = None
        self.jerry_start_y = None

        # Current x, y, and yaw of the Tom robot.
        self.tom_cur_x = None
        self.tom_cur_y = None
        self.tom_cur_yaw = None
        # Start x, y of the Tom robot
        self.tom_start_x = None
        self.tom_start_y = None

        self.offset_x = 0
        self.offset_y = 1
    
    def jerry_odom_cb(self, msg):
        """Callback function for `self.my_odom_sub`."""
        self.jerry_cur_x = msg.y
        self.jerry_cur_y = msg.x

        # The first x,y the robot receives from the odom is treated as the origin
        if self.jerry_start_x is None:
            self.jerry_start_x = msg.y
        
        if self.jerry_start_y is None:
            self.jerry_start_y = msg.x
        
        if self.jerry_start_x is not None and self.jerry_start_y is not None:
            self.jerry_cur_x = msg.y - self.jerry_start_x
            self.jerry_cur_y = msg.x - self.jerry_start_y
        
        # msg.z is the angular z i.e. yaw of the robot
        self.jerry_cur_yaw = msg.z
    
    def tom_odom_cb(self, msg):
        """Callback function for `self.my_odom_sub`."""
        self.tom_cur_x = msg.y
        self.tom_cur_y = msg.x

        # The first x,y the robot receives from the odom is treated as the origin
        if self.tom_start_x is None:
            self.tom_start_x = msg.y
        
        if self.tom_start_y is None:
            self.tom_start_y = msg.x
        
        if self.tom_start_x is not None and self.tom_start_y is not None:
            self.tom_cur_x = msg.y - self.tom_start_x
            self.tom_cur_y = msg.x - self.tom_start_y
        
        # msg.z is the angular z i.e. yaw of the robot
        self.tom_cur_yaw = msg.z

    def chase_jerry(self):
        # Calculate the difference in positions
        offset_vector_x = self.offset_x - self.tom_cur_x
        offset_vector_y = self.offset_y - self.tom_cur_y

        dx = offset_vector_x + self.jerry_cur_x
        dy = offset_vector_y + self.jerry_cur_y

        distance = sqrt(dx**2 + dy**2)
        angle_to_jerry = atan2(dy, dx)

        # Create a Twist message for Tom's velocity
        cmd_vel_tom = geometry_msgs.msg.Twist()
        cmd_vel_jerry = geometry_msgs.msg.Twist()

        # Stop if Tom is close enough to Jerry
        if distance <= 0.1:  # Safe distance to stop
            cmd_vel_tom.linear.x = 0.0
            cmd_vel_tom.angular.z = 0.0

            # Stop Jerry as well
            cmd_vel_jerry.linear.x = 0.0
            cmd_vel_jerry.angular.z = 0.0

            self.tom_velocity_publisher.publish(cmd_vel_tom)
            self.jerry_velocity_publisher.publish(cmd_vel_jerry)

            rospy.loginfo("Tom tagged Jerry! Both have stopped.")
            rospy.signal_shutdown("Tag complete")  # Stop the node
        else:
            # Jerry moves in a circle
            cmd_vel_jerry.linear.x = 0.2
            cmd_vel_jerry.angular.z = 0.2

            # Chase Jerry: Set linear and angular velocities
            cmd_vel_tom.linear.x = min(0.5 * distance, 0.5)  # Limit max speed to 0.5 m/s
            cmd_vel_tom.angular.z = 1.5 * angle_to_jerry  # Adjust angular velocity for precise turning

            self.tom_velocity_publisher.publish(cmd_vel_tom)
            self.jerry_velocity_publisher.publish(cmd_vel_jerry)

    def start_chasing(self):
        while self.jerry_cur_x is None or self.tom_cur_x is None:
            rospy.loginfo("Tom odom or Jerry odom callback not yet called")
        
        # Main loop to continuously chase Jerry
        rate = rospy.Rate(10)  # 10 Hz loop
        while not rospy.is_shutdown():
            self.chase_jerry()
            rate.sleep()


if __name__ == '__main__':
    try:
        tom_chaser = TomChasesJerry()
        tom_chaser.start_chasing()
    except rospy.ROSInterruptException:
        pass