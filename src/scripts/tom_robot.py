#!/usr/bin/env python3

import rospy
import math
import numpy as np
import cv2
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan, CompressedImage
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge

# Subdivision of angles in the lidar scanner
ANGLE_THRESHOLD = 45
# Obstacle threshhold, objects below this distance are considered obstacles
TOM_OBJ_THRESHOLD = 0.3
JERRY_OBJ_THRESHOLD = 0.3

class TomAndJerry:
    def __init__(self):
        rospy.init_node('tom_robot', anonymous=True)
        self.tom_vel_pub = rospy.Publisher('rafael/cmd_vel', Twist, queue_size=10)
        self.jerry_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        
        # Subscribe to Tom and Jerry's LIDAR
        self.tom_scan_sub = rospy.Subscriber('/rafael/scan', LaserScan, self.tom_scan_cb)
        self.jerry_scan_sub = rospy.Subscriber('/scan', LaserScan, self.jerry_scan_cb)

        # Subscribe to Tom's odom and Jerry's odom
        rospy.Subscriber('/tom_odom', Point, self.tom_odom_cb)
        rospy.Subscriber('/jerry_odom', Point, self.jerry_odom_cb)

        self.tom_x = None
        self.tom_y = None

        self.jerry_x = None
        self.jerry_y = None

        self.tom_yaw = None
        self.jerry_yaw = None

        self.tom_linear_vel = 0
        self.tom_angular_vel = 0

        self.jerry_linear_vel = 0
        self.jerry_angular_vel = 0

        self.jerry_goal_x = 1.7
        self.jerry_goal_y = 0

        self.jerry_offset = 0.15 # Distance between Jerry and Tom at the start of the game
        self.caught_threshold = 0.1  # How close Tom needs to be to catch Jerry

        # If an object is detected in front of robot, "obstacle_detected" is set to True,
        # and an avoid_angular_vel is calculated to avoid the obstacle
        self.tom_robot_state = {"obstacle_detected": False, "avoid_angular_vel": 0}
        # div_distance keeps track of the LIDAR distances in each region,
        # 0 is the front region, 1 is front-left, 2 is left, etc.
        self.tom_div_distance = {"0": [], "1": [], "2": [], "3": [], "4": [], "5": [], "6": [], "7": []}
        # div_cost calculates the cost of region based on how far it is from 0, and the sign gives the direction
        self.tom_div_cost = {"0": 0, "1": 1, "2": 2, "3": 3, "4": 4, "5": -3, "6": -2, "7": -1}

        # The finding_block state becomes True when Jerry has navigated through the obstacles and now has to head to the colored block
        self.jerry_robot_state = {"obstacle_detected": False, "finding_block": False, "avoid_angular_vel": 0}
        self.jerry_div_distance = {"0": [], "1": [], "2": [], "3": [], "4": [], "5": [], "6": [], "7": []}
        self.jerry_div_cost = {"0": 0, "1": 1, "2": 2, "3": 3, "4": 4, "5": -3, "6": -2, "7": -1}
    
    def jerry_odom_cb(self, msg):
        self.jerry_x = msg.x
        self.jerry_y = msg.y
        self.jerry_yaw = msg.z
    
    def tom_odom_cb(self, msg):
        self.tom_x = msg.x
        self.tom_y = msg.y
        self.tom_yaw = msg.z
    
    def tom_scan_cb(self, msg):
        for key in self.tom_div_distance.keys():
            values = []
            if key == "0":
                # The front region is wider compared to the other regions (60 vs 45),
                # because we need to avoid obstacles in the front
                for x in msg.ranges[int((335/360) * len(msg.ranges)):] + msg.ranges[:int((25/360) * len(msg.ranges))]:
                    if x <= TOM_OBJ_THRESHOLD and not(math.isinf(x)) and not(math.isnan(x)) and x > msg.range_min:
                        values.append(x)
            else:
                for x in msg.ranges[int((23/360) * len(msg.ranges)) + int((ANGLE_THRESHOLD/360) * len(msg.ranges)) * (int(key)-1) : int((23/360) * len(msg.ranges)) + int((ANGLE_THRESHOLD/360) * len(msg.ranges)) * int(key)]:
                    if x <= TOM_OBJ_THRESHOLD and not(math.isinf(x)) and not(math.isnan(x)) and x > msg.range_min:
                        values.append(x)
            self.tom_div_distance[key] = values
    
    def jerry_scan_cb(self, msg):
        for key in self.jerry_div_distance.keys():
            values = []
            if key == "0":
                # The front region is wider compared to the other regions (60 vs 45),
                # because we need to avoid obstacles in the front
                for x in msg.ranges[int((335/360) * len(msg.ranges)):] + msg.ranges[:int((25/360) * len(msg.ranges))]:
                    if x <= JERRY_OBJ_THRESHOLD and not(math.isinf(x)) and not(math.isnan(x)) and x > msg.range_min:
                        values.append(x)
            else:
                for x in msg.ranges[int((23/360) * len(msg.ranges)) + int((ANGLE_THRESHOLD/360) * len(msg.ranges)) * (int(key)-1) : int((23/360) * len(msg.ranges)) + int((ANGLE_THRESHOLD/360) * len(msg.ranges)) * int(key)]:
                    if x <= JERRY_OBJ_THRESHOLD and not(math.isinf(x)) and not(math.isnan(x)) and x > msg.range_min:
                        values.append(x)
            self.jerry_div_distance[key] = values
    
    def calc_tom_robot_state(self):
        nearest = math.inf
        region_diff = 0
        # Regional differences are calculated relative to the front region
        goal = "0"
        # The 4th region gives the highest regional diff so we start with that
        max_destination = "4"
        max_distance = 0

        for key, value in self.tom_div_distance.items():
            region_diff = abs(self.tom_div_cost[key] - self.tom_div_cost[goal])
            
            # If there're no obstacles in that region
            if not len(value):
                # Find the obstacle-free region closest to the front
                if (region_diff < nearest):
                    nearest = region_diff
                    max_distance = TOM_OBJ_THRESHOLD
                    max_destination = key
            # Check if the region is the most "obstacle-free", i.e. the LIDAR distance is the highest
            elif max(value) > max_distance:
                max_distance = max(value)
                max_destination = key

        # Difference between the most obstacle-free region and the front
        region_diff = self.tom_div_cost[max_destination] - self.tom_div_cost[goal]

        # If the obstacle free path closest to the front is not the front (i.e. nearest != 0),
        # this means that there is an obstacle in the front
        self.tom_robot_state["obstacle_detected"] = (nearest != 0)
        # The avoid_angular_vel is 0.8, and it's sign is the same as the sign of the regional difference
        # We do the max(1, ) thing to avoid division by 0 when the regional difference is 0
        self.tom_robot_state["avoid_angular_vel"] = ((region_diff/max(1, abs(region_diff))) * 0.8)

    def tom_go_to_jerry(self):
        # Calculate distance between Tom and Jerry
        distance = math.sqrt(((self.jerry_x + self.jerry_offset) - self.tom_x)**2 + (self.jerry_y - self.tom_y)**2)
        
        # Calculate what yaw would Tom need to point at to face Jerry
        phi = math.atan2(self.jerry_y - self.tom_y, (self.jerry_x + self.jerry_offset) - self.tom_x)
        
        # Calulcate angular difference between Jerry's current yaw and the target yaw
        delta_theta = phi - self.tom_yaw
        delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]
        
        if self.tom_robot_state["obstacle_detected"]:
            # If an obstacle is in the way, rotate back to point in a new direction
            self.tom_linear_vel = -0.1
            self.tom_angular_vel = self.tom_robot_state["avoid_angular_vel"]
        else:
            # If no obstacle is in way, move towards Jerry
            self.tom_linear_vel = 0.2
            self.tom_angular_vel = delta_theta

    def run_game(self):
        while self.jerry_x is None or self.tom_x is None:
            pass
        
        while not rospy.is_shutdown():
            # Make Tom move towards Jerry while avoiding obstacles
            self.calc_tom_robot_state()
            self.tom_go_to_jerry()

            rospy.loginfo(f"Tom coordinates: ({self.tom_x}, {self.tom_y})")
            rospy.loginfo(f"Jerry coordinates: ({self.jerry_x}, {self.jerry_y})")
            rospy.loginfo(f"Tom yaw: {self.tom_yaw}")
            rospy.loginfo(f"Jerry yaw: {self.jerry_yaw}")
            rospy.loginfo(f"Tom Linear and Angular Vel: ({self.tom_linear_vel}, {self.tom_angular_vel})")
            rospy.loginfo(f"Jerry Linear and Angular Vel: ({self.jerry_linear_vel}, {self.jerry_angular_vel})")

            tom_distance_to_jerry = math.sqrt(((self.jerry_x + self.jerry_offset) - self.tom_x)**2 + (self.jerry_y - self.tom_y)**2)
            # Goal coordinate here is after the obstacle course and before the colored blocks
            jerry_distance_to_goal = math.sqrt((self.jerry_goal_x - self.jerry_x)**2 + (self.jerry_goal_y - self.jerry_y)**2)

            rospy.loginfo(f"Tom distance to Jerry: {tom_distance_to_jerry}")
            rospy.loginfo(f"Jerry distance to Goal: {jerry_distance_to_goal}")
            rospy.loginfo("")

            cmd_vel_tom = Twist()

            # If Tom catches up to Jerry, pause everything and declare Tom wins
            if tom_distance_to_jerry < self.caught_threshold:
                cmd_vel_tom.linear.x = 0
                cmd_vel_tom.angular.z = 0
                self.tom_vel_pub.publish(cmd_vel_tom)
                rospy.loginfo("Tom caught Jerry! Tom Wins!")
                break

            cmd_vel_tom.linear.x = self.tom_linear_vel
            cmd_vel_tom.angular.z = self.tom_angular_vel

            self.tom_vel_pub.publish(cmd_vel_tom)
            self.rate.sleep()


    def shutdown(self):
        """Stop the robot when shutting down."""
        rospy.loginfo("Shutting down...")
        self.tom_vel_pub.publish(Twist())  # Stop the robot
        self.jerry_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == "__main__":
    try:
        game = TomAndJerry()
        rospy.on_shutdown(game.shutdown)
        game.run_game()
    except rospy.ROSInterruptException:
        pass