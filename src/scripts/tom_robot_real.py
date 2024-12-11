#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Subdivision of angles in the lidar scanner
ANGLE_THRESHOLD = 45
# Obstacle threshhold, objects below this distance are considered obstacles
OBJ_THRESHOLD = 0.45
# K Value for Linear Velocity P-Controller
K_LIN = 0.7
# K Value for Angular Velocity P-Controller
K_ANG = 1.0

class TomAndJerry:
    def __init__(self):
        rospy.init_node('tom_robot_real', anonymous=True)
        self.vel_pub = rospy.Publisher('rafael/cmd_vel', Twist, queue_size=10)
        self.jerry_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        # Subscribe to Tom's odom and Jerry's odom
        rospy.Subscriber('/tom_odom', Point, self.update_tom_position)
        rospy.Subscriber('/jerry_odom', Point, self.update_jerry_position)
        
        # Subscribe to Tom and Jerry's LIDAR
        self.tom_scan_sub = rospy.Subscriber('/rafael/scan', LaserScan, self.tom_scan_cb)
        self.jerry_scan_sub = rospy.Subscriber('/scan', LaserScan, self.jerry_scan_cb)

        self.tom_position = None
        self.jerry_position = None
        self.tom_yaw = None
        self.jerry_yaw = None

        self.jerry_goal_x = 0
        self.jerry_goal_y = 2

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

        # If an object is detected in front of robot, "obstacle_detected" is set to True,
        # and an avoid_angular_vel is calculated to avoid the obstacle
        self.tom_robot_state = {"obstacle_detected": False, "avoid_angular_vel": 0}
        # div_distance keeps track of the LIDAR distances in each region,
        # 0 is the front region, 1 is front-left, 2 is left, etc.
        self.tom_div_distance = {"0": [], "1": [], "2": [], "3": [], "4": [], "5": [], "6": [], "7": []}
        # div_cost calculates the cost of region based on how far it is from 0, and the sign gives the direction
        self.tom_div_cost = {"0": 0, "1": 1, "2": 2, "3": 3, "4": 4, "5": -3, "6": -2, "7": -1}

        # If an object is detected in front of robot, "obstacle_detected" is set to True,
        # and an avoid_angular_vel is calculated to avoid the obstacle
        self.jerry_robot_state = {"obstacle_detected": False, "avoid_angular_vel": 0}
        # div_distance keeps track of the LIDAR distances in each region,
        # 0 is the front region, 1 is front-left, 2 is left, etc.
        self.jerry_div_distance = {"0": [], "1": [], "2": [], "3": [], "4": [], "5": [], "6": [], "7": []}
        # div_cost calculates the cost of region based on how far it is from 0, and the sign gives the direction
        self.jerry_div_cost = {"0": 0, "1": 1, "2": 2, "3": 3, "4": 4, "5": -3, "6": -2, "7": -1}

        self.jerry_distance_to_goal = 2
    
    def tom_scan_cb(self, msg):
        for key in self.tom_div_distance.keys():
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
            self.tom_div_distance[key] = values
    
    def jerry_scan_cb(self, msg):
        for key in self.jerry_div_distance.keys():
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
                    max_distance = OBJ_THRESHOLD
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
        # The avoid_angular_vel is 0.7, and it's sign is the same as the sign of the regional difference
        # We do the max(1, ) thing to avoid division by 0 when the regional difference is 0
        self.tom_robot_state["avoid_angular_vel"] = ((region_diff/max(1, abs(region_diff))) * 0.7)
    
    def calc_jerry_robot_state(self):
        nearest = math.inf
        region_diff = 0
        # Regional differences are calculated relative to the front region
        goal = "0"
        # The 4th region gives the highest regional diff so we start with that
        max_destination = "4"
        max_distance = 0

        for key, value in self.jerry_div_distance.items():
            region_diff = abs(self.jerry_div_cost[key] - self.jerry_div_cost[goal])
            
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
        region_diff = self.jerry_div_cost[max_destination] - self.jerry_div_cost[goal]

        # If the obstacle free path closest to the front is not the front (i.e. nearest != 0),
        # this means that there is an obstacle in the front
        self.jerry_robot_state["obstacle_detected"] = (nearest != 0)
        # The avoid_angular_vel is 0.7, and it's sign is the same as the sign of the regional difference
        # We do the max(1, ) thing to avoid division by 0 when the regional difference is 0
        self.jerry_robot_state["avoid_angular_vel"] = ((region_diff/max(1, abs(region_diff))) * 0.7)

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
        
        self.jerry_yaw = msg.z
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
    
    def jerry_go_to_goal(self):
        current_angle = self.jerry_yaw
    
        jerry_x, jerry_y = self.jerry_position
        angle_to_goal = math.atan2(self.jerry_goal_x - jerry_x, self.jerry_goal_y - jerry_y)

        # angle_to_goal = angle_to_goal if angle_to_goal >= 0 else angle_to_goal + (2 * math.pi)
        # if angle_to_goal < -math.pi/4 or angle_to_goal > math.pi/4:
        #     if 0 > self.goal_y > jerry_y:
        #         angle_to_goal = -2 * math.pi + angle_to_goal
        #     elif 0 <= self.goal_y < jerry_y:
        #         angle_to_goal = 2 * math.pi + angle_to_goal
        
        # Adjust current_angle to be from 0 to 2pi
        # if self.last_angle > math.pi - 0.1 and current_angle <= 0:
        #     current_angle = 2 * math.pi + current_angle
        # elif self.last_angle < -math.pi + 0.1 and current_angle > 0:
        #     current_angle = -2 * math.pi + current_angle
        
        # P-Controller for Angular Velocity
        jerry_angular_vel = K_ANG * (angle_to_goal - current_angle)
        
        # P-Controller for Linear Velocity, with a maximum of 0.3
        self.jerry_distance_to_goal = math.hypot(self.jerry_goal_y - jerry_y, self.jerry_goal_x - jerry_x)
        jerry_linear_vel = min(K_LIN * self.jerry_distance_to_goal, 0.3)

        # Bound the angular velocity between -0.5 and 0.5
        if jerry_angular_vel > 0:
            jerry_angular_vel = min(jerry_angular_vel, 0.5)
        else:
            jerry_angular_vel = max(jerry_angular_vel, -0.5)

        # Update the last_angle for the next loop
        # self.last_angle = current_angle
        
        return jerry_linear_vel, jerry_angular_vel

    def chase_jerry(self):
        """Chase Jerry by controlling Tom's motion."""
        while not rospy.is_shutdown():
            if self.tom_position and self.jerry_position:
                distance, angle_diff = self.compute_angle_and_distance()

                rospy.loginfo(f"Tom Position: {self.tom_position}, Jerry Position: {self.jerry_position}")
                rospy.loginfo(f"Distance: {distance:.4f}, Angle Diff: {angle_diff:.4f}")
                rospy.loginfo(f"Jerry's Distance to Goal: {self.jerry_distance_to_goal}")
                rospy.loginfo(f"Jerry's LIDAR DIV Distances: {self.jerry_div_distance}")

                if distance < self.caught_threshold:
                    rospy.loginfo("Tom Caught Jerry! Tom wins!")
                    break
                
                if self.jerry_distance_to_goal < 0.1:
                    rospy.loginfo("Jerry reached the cheese! Jerry wins!")

                cmd_vel_tom = Twist()
                cmd_vel_jerry = Twist()

                # Calculate the linear and angular velocities
                linear_velocity = min(0.5 * distance, 0.2)  # Scaled forward speed
                angular_velocity = 0.05 * angle_diff  # Scaled turn speed

                # Combine both linear and angular velocities
                self.calc_tom_robot_state()
                if self.tom_robot_state["obstacle_detected"]:
                    # Avoid obstacle by turning back
                    cmd_vel_tom.linear.x = -0.1
                    cmd_vel_tom.angular.z = self.tom_robot_state["avoid_angular_vel"]
                else:
                    cmd_vel_tom.linear.x = linear_velocity
                    cmd_vel_tom.angular.z = angular_velocity

                if self.jerry_robot_state["obstacle_detected"]:
                    # Avoid obstacle by turning back
                    cmd_vel_jerry.linear.x = -0.1
                    cmd_vel_jerry.angular.z = self.tom_robot_state["avoid_angular_vel"]
                else:
                    cmd_vel_jerry.linear.x, cmd_vel_jerry.angular.z = self.jerry_go_to_goal()

                self.vel_pub.publish(cmd_vel_tom)
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
