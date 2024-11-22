#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
import math

# Subdivision of angles in the lidar scanner
ANGLE_THRESHOLD = 45
# Obstacle threshhold, objects below this distance are considered obstacles
OBJ_THRESHOLD = 0.45
# K Value for Linear Velocity P-Controller
K_LIN = 0.7
# K Value for Angular Velocity P-Controller
K_ANG = 1.0

class JerryRobot():
    def __init__(self, goal_x, goal_y):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.my_odom_sub = rospy.Subscriber('jerry/my_odom', Point, self.odom_cb)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        # Current x, y, and yaw of the robot.
        self.cur_x = None
        self.cur_y = None
        self.cur_yaw = None
        self.start_x = None
        self.start_y = None
        # Goal coordinate to be reached
        self.goal_x = goal_x
        self.goal_y = goal_y
        # If an object is detected in front of robot, "obstacle_detected" is set to True,
        # and an avoid_angular_vel is calculated to avoid the obstacle
        self.robot_state = {"obstacle_detected": False, "avoid_angular_vel": 0}
        # div_distance keeps track of the LIDAR distances in each region,
        # 0 is the front region, 1 is front-left, 2 is left, etc.
        self.div_distance = {"0": [], "1": [], "2": [], "3": [], "4": [], "5": [], "6": [], "7": []}
        # div_cost calculates the cost of region based on how far it is from 0, and the sign gives the direction
        self.div_cost = {"0": 0, "1": 1, "2": 2, "3": 3, "4": 4, "5": -3, "6": -2, "7": -1}
        # Measure the previous angle/yaw of the robot
        self.last_angle = 0
        # Distance between robot and goal coordinate
        self.distance_to_goal = None
    
    def odom_cb(self, msg):
        """Callback function for `self.my_odom_sub`."""
        self.cur_x = msg.y
        self.cur_y = msg.x
        if self.start_x is None:
            self.start_x = msg.y
        if self.start_y is None:
            self.start_y = msg.x
        if self.start_x is not None and self.start_y is not None:
            self.cur_x = msg.y - self.start_x
            self.cur_y = msg.x - self.start_y
        # msg.z is the angular z i.e. yaw of the robot
        self.cur_yaw = msg.z
        # compute the distance from the current position to the goal
        self.distance_to_goal = math.hypot(self.goal_x - self.cur_x, self.goal_y - self.cur_y)
    
    def scan_cb(self, msg):
        for key in self.div_distance.keys():
            values = []
            if key == "0":
                # The front region is wider compared to the other regions (60 vs 45),
                # because we need to avoid obstacles in the front
                for x in msg.ranges[int((330/360) * len(msg.ranges)):] + msg.ranges[:int((30/360) * len(msg.ranges))]:
                    if x <= OBJ_THRESHOLD and not(math.isinf(x)) and not(math.isnan(x)) and x > msg.range_min:
                        values.append(x)
            else:
                for x in msg.ranges[int((23/360) * len(msg.ranges)) + int((ANGLE_THRESHOLD/360) * len(msg.ranges)) * (int(key)-1) : int((23/360) * len(msg.ranges)) + int((ANGLE_THRESHOLD/360) * len(msg.ranges)) * int(key)]:
                    if x <= OBJ_THRESHOLD and not(math.isinf(x)) and not(math.isnan(x)) and x > msg.range_min:
                        values.append(x)
            self.div_distance[key] = values
    
    def calc_robot_state(self):
        nearest = 9999999
        region_diff = 0
        # Regional differences are calculated relative to the front region
        goal = "0"
        # The 4th region gives the highest regional diff so we start with that
        max_destination = "4"
        max_distance = 0.0000001

        for key, value in self.div_distance.items():
            region_diff = abs(self.div_cost[key] - self.div_cost[goal])
            
            # If there're no obstacles in that region
            if not len(value):
                # Find the obstacle-free region closest to the front
                if (region_diff < nearest):
                    nearest = region_diff
                    max_distance = OBJ_THRESHOLD
                    max_destination = key
            # Check if the region is the most "obstacle-free", i.e. the LIDAR distance is the highest
            elif max(value) > max_distance:
                max_distance = max(value)
                max_destination = key

        # Difference between the most obstacle-free region and the front
        region_diff = self.div_cost[max_destination] - self.div_cost[goal]

        # If the obstacle free path closest to the front is not the front (i.e. nearest != 0),
        # this means that there is an obstacle in the front
        self.robot_state["obstacle_detected"] = (nearest != 0)
        # The avoid_angular_vel is 0.7, and it's sign is the same as the sign of the regional difference
        # We do the max(1, ) thing to avoid division by 0 when the regional difference is 0
        self.robot_state["avoid_angular_vel"] = ((region_diff/max(1, abs(region_diff))) * 0.7)
    
    # This function is called when the robot is about to collide into an obstacle
    # The robot will turn back according to the avoid_angular_vel calculated
    def avoid_obstacle(self):
        angular_velocity = self.robot_state["avoid_angular_vel"]
        # After detecting an obstacle, the robot will back up a bit while rotating to point in a new direction
        velocity = Twist()
        velocity.linear.x = -0.1
        velocity.angular.z = angular_velocity
        return velocity
    
    def go_to_goal(self):
        current_angle = self.cur_yaw
    
        x_start = self.cur_x
        y_start = self.cur_y
        # Final coordinates are given by goal_x and goal_y
        goal = Twist()

        angle_to_goal = math.atan2(self.goal_y - y_start, self.goal_x - x_start)
        # Then we should turn robot facing goal coordinate
        angular_velocity = angle_to_goal - current_angle
        goal.linear.x = 0.2
        goal.angular.z = angular_velocity
        return goal

    def update(self):
        self.calc_robot_state()
        if self.robot_state["obstacle_detected"]:
            velocity = self.avoid_obstacle()
        else:
            velocity = self.go_to_goal()

        self.cmd_vel_pub.publish(velocity)
        

def main():
    rospy.init_node('jerry')
    goal_x = float(input("Input goal x coordinate:"))
    goal_y = float(input("Input goal y coordinate:"))
    robot = JerryRobot(goal_x, goal_y)
    rate = rospy.Rate(10) # 10Hz

    while not rospy.is_shutdown():
        robot.update()
        rate.sleep()


if __name__ == '__main__':
    main()
