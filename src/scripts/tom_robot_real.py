#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class TomAndJerry:
    def __init__(self):
        rospy.init_node('tom_robot_real', anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        rospy.Subscriber('/odom', Odometry, self.update_tom_position)
        self.jerry_position = (0.57, 0.27)  # Jerry's fixed position
        self.tom_position = (0.0, 0.0)
        self.tom_yaw = 0.0

        self.caught_threshold = 0.05  # More precise threshold

    def update_tom_position(self, msg):
        self.tom_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
        orientation_q = msg.pose.pose.orientation
        (_, _, self.tom_yaw) = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )

    def compute_angle_and_distance(self):
        jerry_x, jerry_y = self.jerry_position
        tom_x, tom_y = self.tom_position

        distance = math.sqrt((jerry_x - tom_x)**2 + (jerry_y - tom_y)**2)
        desired_angle = math.atan2(jerry_y - tom_y, jerry_x - tom_x)
        angle_diff = desired_angle - self.tom_yaw
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalize

        return distance, angle_diff

    def chase_jerry(self):
        while not rospy.is_shutdown():
            distance, angle_diff = self.compute_angle_and_distance()

            rospy.loginfo(
                f"Tom Position: {self.tom_position}, Jerry Position: {self.jerry_position}, "
                f"Distance: {distance:.4f}, Angle Diff: {angle_diff:.4f}"
            )

            if distance < self.caught_threshold:
                rospy.loginfo("Caught Jerry! Threshold Met")
                break

            cmd_vel = Twist()
            if abs(angle_diff) > 0.1:  # Adjust turning precision
                cmd_vel.angular.z = 0.5 * angle_diff  # Scaled turn speed
            else:
                cmd_vel.linear.x = min(0.5 * distance, 0.2)  # Scaled forward speed
            
            self.vel_pub.publish(cmd_vel)
            self.rate.sleep()

    def shutdown(self):
        rospy.loginfo("Shutting down...")
        self.vel_pub.publish(Twist())  # Stop the robot
        rospy.sleep(1)

if __name__ == "__main__":
    try:
        game = TomAndJerry()
        rospy.on_shutdown(game.shutdown)
        game.chase_jerry()
    except rospy.ROSInterruptException:
        pass
