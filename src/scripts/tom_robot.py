#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class TomAndJerry:
    def __init__(self):
        rospy.init_node('tom_robot_real', anonymous=True)
        self.vel_pub = rospy.Publisher('rafael/cmd_vel', Twist, queue_size=10)
        self.jerry_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        # Subscribe to Tom's odom and Jerry's odom
        rospy.Subscriber('/tom_odom', Point, self.update_tom_position)
        rospy.Subscriber('/jerry_odom', Point, self.update_jerry_position)

        self.tom_position = None
        self.jerry_position = None
        self.tom_yaw = None

        self.caught_threshold = 0.5  # How close Tom needs to be to catch Jerry

        # Offset for Jerry's position
        self.offset_x = 0  # Adjust the offset values as needed
        self.offset_y = 1

        # Start x, y of the Tom robot
        self.tom_start_x = None
        self.tom_start_y = None

        # Start x, y of the Jerry robot
        self.jerry_start_x = None
        self.jerry_start_y = None

    def update_tom_position(self, msg):
        """Callback for Tom's position."""
        # The first x,y the robot receives from the odom is treated as the origin
        if self.tom_start_x is None:
            self.tom_start_x = msg.y
        
        if self.tom_start_y is None:
            self.tom_start_y = msg.x
        
        if self.tom_start_x is not None and self.tom_start_y is not None:
            self.tom_position = (msg.y - self.tom_start_x, msg.x - self.tom_start_y)
        else:
            self.tom_position = (msg.y, msg.x)
        
        self.tom_yaw = msg.z  # Yaw is stored in the z field
        # rospy.loginfo(f"Updated Tom Position: {self.tom_position}")
        # rospy.loginfo(f"Updated Tom Yaw: {self.tom_yaw}")

    def update_jerry_position(self, msg):
        """Callback for Jerry's position with offset."""
        # Apply offset to Jerry's position
        if self.jerry_start_x is None:
            self.jerry_start_x = msg.y
        
        if self.jerry_start_y is None:
            self.jerry_start_y = msg.x
        
        if self.jerry_start_x is not None and self.jerry_start_y is not None:
            self.jerry_position = (msg.y + self.offset_x - self.jerry_start_x, msg.x + self.offset_y - self.jerry_start_y)
        else:
            self.jerry_position = (msg.y + self.offset_x, msg.x + self.offset_y)
        # rospy.loginfo(f"Updated Jerry Position with Offset: {self.jerry_position}")

    def compute_angle_and_distance(self):
        """Calculate the distance and angle between Tom and Jerry."""
        jerry_x, jerry_y = self.jerry_position
        tom_x, tom_y = self.tom_position

        distance = math.sqrt((jerry_x - tom_x)**2 + (jerry_y - tom_y)**2)
        desired_angle = math.atan2(jerry_y - tom_y, jerry_x - tom_x)
        angle_diff = desired_angle - self.tom_yaw
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

        return distance, angle_diff

    def chase_jerry(self):
        """Chase Jerry by controlling Tom's motion."""
        while not rospy.is_shutdown():
            if self.tom_position and self.jerry_position:
                distance, angle_diff = self.compute_angle_and_distance()

                rospy.loginfo(
                    f"Tom Position: {self.tom_position}, Jerry Position: {self.jerry_position}, "
                    f"Distance: {distance:.4f}, Angle Diff: {angle_diff:.4f}"
                )

                if distance < self.caught_threshold:
                    rospy.loginfo("Caught Jerry!")
                    break

                cmd_vel = Twist()
                cmd_vel_jerry = Twist()

                # Calculate the linear and angular velocities
                linear_velocity = min(0.5 * distance, 0.2)  # Scaled forward speed
                angular_velocity = 0.05 * angle_diff  # Scaled turn speed

                # Combine both linear and angular velocities
                cmd_vel.linear.x = linear_velocity
                cmd_vel.angular.z = angular_velocity

                cmd_vel_jerry.linear.x = 0.2
                cmd_vel_jerry.angular.z = -0.08

                self.vel_pub.publish(cmd_vel)
                self.jerry_vel_pub.publish(cmd_vel_jerry)
            else:
                rospy.loginfo("Waiting for positions...")

            self.rate.sleep()


    def shutdown(self):
        """Stop the robot when shutting down."""
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
