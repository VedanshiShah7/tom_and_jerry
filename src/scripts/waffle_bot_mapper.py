#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import random

class WaffleBotMapper:
    def __init__(self):
        rospy.init_node('waffle_bot_mapper', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        self.rate = rospy.Rate(10)  # 10 Hz
        self.map = None

    def map_callback(self, data):
        self.map = data

    def move(self, linear_speed=0.2, angular_speed=0.0):
        move_cmd = Twist()
        move_cmd.linear.x = linear_speed
        move_cmd.angular.z = angular_speed
        self.velocity_publisher.publish(move_cmd)

    def stop(self):
        self.move(0, 0)

    def explore(self):
        while not rospy.is_shutdown():
            if self.map:
                # Basic exploration logic
                for _ in range(100):  # Number of exploration steps
                    # Move forward
                    self.move(linear_speed=0.2)
                    self.rate.sleep()

                    # Randomly choose to rotate
                    if random.random() < 0.5:
                        self.move(angular_speed=random.choice([-0.5, 0.5]))  # Rotate left or right
                        self.rate.sleep()
                
                # Stop the robot after exploring
                self.stop()
                rospy.loginfo("Exploration complete.")
            else:
                rospy.loginfo("Waiting for map data...")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        mapper = WaffleBotMapper()
        mapper.explore()
    except rospy.ROSInterruptException:
        pass
