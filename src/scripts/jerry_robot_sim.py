#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
import math

# Angular size of each region
REGION_SIZE = 45
# Obstacle threshhold, objects closer than this distance are considered obstacles
OBS_THRESHOLD = 0.45
# P Value for Linear Velocity P-Controller
P_LIN = 0.7
# P Value for Angular Velocity P-Controller
P_ANG = 1.0

class JerryRobot():
    def __init__(self, goal_x, goal_y):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.odom_cb)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        # Current x, y, and yaw of the robot.
        self.cur_x = None
        self.cur_y = None
        self.cur_yaw = None
        # Goal coordinate to be reached
        self.goal_x = goal_x
        self.goal_y = goal_y
        # If an object is detected in front of robot, "obstacle_detected" is set to True,
        # and an avoid_angular_vel is calculated to avoid the obstacle
        self.robot_state = {"obstacle_detected": False, "avoid_angular_vel": 0}
        # region_distance keeps track of the LIDAR distances in each region,
        # 0 is the front region, 1 is front-left, 2 is left, etc.
        self.region_distance = {"0": [], "1": [], "2": [], "3": [], "4": [], "5": [], "6": [], "7": []}
        # region_cost calculates the cost of region based on how far it is from 0, and the sign gives the direction
        self.region_cost = {"0": 0, "1": 1, "2": 2, "3": 3, "4": 4, "5": -3, "6": -2, "7": -1}
        # Measure the previous angle/yaw of the robot
        self.prev_angle = 0
        # Distance between robot and goal coordinate
        self.distance_to_goal = None
    
    def odom_cb(self, msg):
        """Callback function for `self.my_odom_sub`."""
        self.cur_x = msg.y
        self.cur_y = msg.x
        # msg.z is the angular z i.e. yaw of the robot
        self.cur_yaw = msg.z
        # compute the distance from the current position to the goal
        self.distance_to_goal = math.hypot(self.goal_x - msg.y, self.goal_y - msg.x)
    
    def scan_cb(self, msg):
        for key in self.region_distance.keys():
            values = []
            if key == "0":
                # The front region is wider compared to the other regions (60 vs 45),
                # because we need to avoid obstacles in the front
                for x in msg.ranges[int((330/360) * len(msg.ranges)):] + msg.ranges[:int((30/360) * len(msg.ranges))]:
                    if x <= OBS_THRESHOLD and not(math.isinf(x)) and not(math.isnan(x)) and x > msg.range_min:
                        values.append(x)
            else:
                for x in msg.ranges[int((23/360) * len(msg.ranges)) + int((REGION_SIZE/360) * len(msg.ranges)) * (int(key)-1) : int((23/360) * len(msg.ranges)) + int((REGION_SIZE/360) * len(msg.ranges)) * int(key)]:
                    if x <= OBS_THRESHOLD and not(math.isinf(x)) and not(math.isnan(x)) and x > msg.range_min:
                        values.append(x)
            self.region_distance[key] = values
    
    def calc_robot_state(self):
        nearest = 9999999
        region_diff = 0
        # Regional differences are calculated relative to the front region
        goal = "0"
        # The 4th region gives the highest regional diff so we start with that
        max_destination = "4"
        max_distance = 0.0000001

        for key, value in self.region_distance.items():
            region_diff = abs(self.region_cost[key] - self.region_cost[goal])
            
            # If there're no obstacles in that region
            if not len(value):
                # Find the obstacle-free region closest to the front
                if (region_diff < nearest):
                    nearest = region_diff
                    max_distance = OBS_THRESHOLD
                    max_destination = key
            # Check if the region is the most "obstacle-free", i.e. the LIDAR distance is the highest
            elif max(value) > max_distance:
                max_distance = max(value)
                max_destination = key

        # Difference between the most obstacle-free region and the front
        region_diff = self.region_cost[max_destination] - self.region_cost[goal]

        # If the obstacle free path closest to the front is not the front (i.e. nearest != 0),
        # this means that there is an obstacle in the front
        self.robot_state["obstacle_detected"] = (nearest != 0)
        # The avoid_angular_vel is 0.7, and it's sign is the same as the sign of the regional difference
        # We do the max(1, ) thing to avoid division by 0 when the regional difference is 0
        self.robot_state["avoid_angular_vel"] = ((region_diff/max(1, abs(region_diff))) * 0.7)
    
    # This function is called when the robot is about to collide into an obstacle
    # The robot will turn back according to the avoid_angular_vel calculated
    def avoid_obstacle(self):
        # After detecting an obstacle, the robot will back up a bit while rotating to point in a new direction
        twist = Twist()
        twist.linear.x = -0.1
        twist.angular.z = self.robot_state["avoid_angular_vel"]
        return twist
    
    def head_to_goal(self):
        current_angle = self.cur_yaw
    
        x_start = self.cur_x
        y_start = self.cur_y
        angle_to_goal = math.atan2(self.goal_x - x_start, self.goal_y - y_start)

        # Convert the range of angle_to_goal to be from 0 to 2pi, instead of -inf to inf as returned by math.atan2
        if angle_to_goal < -math.pi/4 or angle_to_goal > math.pi/4:
            if self.goal_y < 0 and self.goal_y > y_start:
                angle_to_goal = -2 * math.pi + angle_to_goal
            elif self.goal_y >= 0 and self.goal_y < y_start:
                angle_to_goal = 2 * math.pi + angle_to_goal
        
        # Adjust current_angle to be from 0 to 2pi
        if self.prev_angle > math.pi - 0.1 and current_angle <= 0:
            current_angle = 2 * math.pi + current_angle
        elif self.prev_angle < -math.pi + 0.1 and current_angle > 0:
            current_angle = -2 * math.pi + current_angle

        twist = Twist()
        # P-Controller for Angular Velocity
        twist.angular.z = P_ANG * (angle_to_goal - current_angle)
        
        # P-Controller for Linear Velocity, with a maximum of 0.3
        twist.linear.x = min(P_LIN * self.distance_to_goal, 0.3)

        # Bound the angular velocity between -0.5 and 0.5
        if twist.angular.z > 0:
            twist.angular.z = min(twist.angular.z, 0.5)
        else:
            twist.angular.z = max(twist.angular.z, -0.5)

        self.prev_angle = current_angle
        
        return twist

    def run(self):
        while self.cur_x is None or self.cur_y is None or self.cur_yaw is None:
            pass
        
        rate = rospy.Rate(10)
        self.distance_to_goal = math.hypot(self.goal_x - self.cur_x, self.goal_y - self.cur_y)

        while not rospy.is_shutdown():
            # Keep the robot running until we are very close to goal (9cm)
            if self.distance_to_goal > 0.09:
                # Log the robot's state and distance to goal
                rospy.loginfo("Distance to goal {0}".format(self.distance_to_goal))
                rospy.loginfo("Robot state {0}".format(self.robot_state))

                if self.distance_to_goal < 0.15:
                    # If we're close enough to goal, move directly towards goal
                    twist = self.head_to_goal()
                else:
                    self.calc_robot_state()
                    if self.robot_state["obstacle_detected"]:
                        twist = self.avoid_obstacle()
                    else:
                        twist = self.head_to_goal()

                self.cmd_vel_pub.publish(twist)
            else:
                rospy.loginfo("Goal Reached")
                twist = Twist()
                twist.linear.x = 0
                twist.angular.z = 0
                self.cmd_vel_pub.publish(twist)
                break
            
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('jerry_robot', anonymous=True)
    JerryRobot(0, 4).run()
