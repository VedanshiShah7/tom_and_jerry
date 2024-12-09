#!/usr/bin/env python

import rospy
import math
import tf2_ros
import geometry_msgs.msg
from math import atan2, sqrt

# Subdivision of angles in the lidar scanner
ANGLE_THRESHOLD = 45
# Obstacle threshhold, objects below this distance are considered obstacles
OBJ_THRESHOLD = 0.35

class TomChaser:
    def __init__(self, delay_start_tom):
        # Initialize the ROS node
        rospy.init_node('tom_chaser', anonymous=True)

        # Velocity publishers for Tom and Jerry
        self.tom_velocity_publisher = rospy.Publisher('/rafael/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
        self.jerry_velocity_publisher = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
        self.tom_scan_sub = rospy.Subscriber('/rafael/scan', LaserScan, self.tom_scan_cb)

        # Delay before Tom starts chasing Jerry
        self.delay_start_tom = delay_start_tom
        rospy.loginfo(f"Delaying Tom's start by {self.delay_start_tom} seconds...")

        # Timer for starting the chase
        self.chase_started = False
        self.start_time = rospy.get_time()

        # TF2 buffer and listener to get transforms
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # If an object is detected in front of robot, "obstacle_detected" is set to True,
        # and an avoid_angular_vel is calculated to avoid the obstacle
        self.tom_robot_state = {"obstacle_detected": False, "avoid_angular_vel": 0}
        # div_distance keeps track of the LIDAR distances in each region,
        # 0 is the front region, 1 is front-left, 2 is left, etc.
        self.tom_div_distance = {"0": [], "1": [], "2": [], "3": [], "4": [], "5": [], "6": [], "7": []}
        # div_cost calculates the cost of region based on how far it is from 0, and the sign gives the direction
        self.tom_div_cost = {"0": 0, "1": 1, "2": 2, "3": 3, "4": 4, "5": -3, "6": -2, "7": -1}
    
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
    
    def calc_tom_robot_state(self):
        nearest = math.inf
        region_diff = 0
        # Regional differences are calculated relative to the front region
        goal = "0"
        # The 4th region gives the highest regional diff so we start with that
        max_destination = "4"
        max_distance = 0

        for key, value in self.tom_div_distance.items():
            region_diff = abs(self.tom_div_cost[key] - self.div_cost[goal])
            
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
                self.calc_tom_robot_state()
                if self.tom_robot_state["obstacle_detected"]:
                    # Avoid obstacle by turning back
                    cmd_vel_tom.linear.x = -0.1
                    cmd_vel_tom.angular.z = self.tom_robot_state["avoid_angular_vel"]
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