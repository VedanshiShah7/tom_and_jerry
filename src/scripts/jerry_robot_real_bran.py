#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image, CompressedImage, LaserScan
import math
import tf2_ros
from tf2_ros import TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

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
        self.my_odom_sub = rospy.Subscriber('/jerry_odom', Point, self.odom_cb)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()  # TF2 broadcaster

        # Goal x, y of the robot
        self.goal_x = goal_x
        self.goal_y = goal_y
        # Current x, y, and yaw of the robot.
        self.cur_x = None
        self.cur_y = None
        self.cur_yaw = None
        # Start x, y of the robot
        self.start_x = None
        self.start_y = None
        # If an object is detected in front of robot, "obstacle_detected" is set to True,
        # and an avoid_angular_vel is calculated to avoid the obstacle
        self.robot_state = {"obstacle_detected": False, "avoid_angular_vel": 0, "head_to_colored_block": False}
        # div_distance keeps track of the LIDAR distances in each region,
        # 0 is the front region, 1 is front-left, 2 is left, etc.
        self.div_distance = {"0": [], "1": [], "2": [], "3": [], "4": [], "5": [], "6": [], "7": []}
        # div_cost calculates the cost of region based on how far it is from 0, and the sign gives the direction
        self.div_cost = {"0": 0, "1": 1, "2": 2, "3": 3, "4": 4, "5": -3, "6": -2, "7": -1}
        # Measure the previous angle/yaw of the robot
        self.last_angle = 0
        # Distance between robot and goal coordinate
        self.distance_to_goal = None

        # Get the target color parameter, defaulting to 'red'
        self.target_color = rospy.get_param('~target_color', 'red')
        # Subscribe to the camera images to detect blocks
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.image_callback)
        # Initialize CvBridge to convert ROS images to OpenCV format
        self.bridge = CvBridge()
        # Publisher to send the bounding box coordinates (center)
        self.bounding_box_pub = rospy.Publisher('/bounding_box', Point, queue_size=1)
        self.bounding = None
        # Initialize smoothing values for the block center
        self.smoothed_center_x = 320  # Initial center is the middle of the image (640x480)
        self.smoothed_center_y = 240  # Initial center is the middle of the image (640x480)
    
    def odom_cb(self, msg):
        """Callback function for `self.my_odom_sub`."""
        self.cur_x = msg.y
        self.cur_y = msg.x
        # The first x,y the robot receives from the odom is treated as the origin
        if self.start_x is None:
            self.start_x = msg.y
            # self.goal_x += msg.y
        if self.start_y is None:
            self.start_y = msg.x
            # self.goal_y += msg.x
        if self.start_x is not None and self.start_y is not None:
            self.cur_x = msg.y - self.start_x
            self.cur_y = msg.x - self.start_y
        # msg.z is the angular z i.e. yaw of the robot
        self.cur_yaw = msg.z
        # compute the distance from the current position to the goal
        self.distance_to_goal = math.hypot(self.goal_y - self.cur_y, self.goal_x - self.cur_x)
        # Publish the transform
        self.broadcast_pose()

    def broadcast_pose(self):
        """Broadcast the current pose of the robot."""
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "world"  # Reference frame
        transform.child_frame_id = "jerry"  # Frame for Jerry Robot

        transform.transform.translation.x = self.cur_x
        transform.transform.translation.y = self.cur_y
        transform.transform.translation.z = 0.0

        # Convert yaw to quaternion
        qx, qy, qz, qw = 0.0, 0.0, math.sin(self.cur_yaw / 2.0), math.cos(self.cur_yaw / 2.0)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)
    
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
        nearest = math.inf
        region_diff = 0
        # Regional differences are calculated relative to the front region
        goal = "0"
        # The 4th region gives the highest regional diff so we start with that
        max_destination = "4"
        max_distance = 0

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
        angle_to_goal = math.atan2(self.goal_x - x_start, self.goal_y - y_start)
        
        # angle_to_goal = angle_to_goal if angle_to_goal >= 0 else angle_to_goal + (2 * math.pi)
        # if angle_to_goal < -math.pi/4 or angle_to_goal > math.pi/4:
        #     if 0 > self.goal_y > y_start:
        #         angle_to_goal = -2 * math.pi + angle_to_goal
        #     elif 0 <= self.goal_y < y_start:
        #         angle_to_goal = 2 * math.pi + angle_to_goal
        
        # Adjust current_angle to be from 0 to 2pi
        if self.last_angle > math.pi - 0.1 and current_angle <= 0:
            current_angle = 2 * math.pi + current_angle
        elif self.last_angle < -math.pi + 0.1 and current_angle > 0:
            current_angle = -2 * math.pi + current_angle
        
        velocity_msg = Twist()
        # P-Controller for Angular Velocity
        velocity_msg.angular.z = K_ANG * (angle_to_goal - current_angle)
        
        # P-Controller for Linear Velocity, with a maximum of 0.3
        velocity_msg.linear.x = min(K_LIN * self.distance_to_goal, 0.3)

        # Bound the angular velocity between -0.5 and 0.5
        if velocity_msg.angular.z > 0:
            velocity_msg.angular.z = min(velocity_msg.angular.z, 0.5)
        else:
            velocity_msg.angular.z = max(velocity_msg.angular.z, -0.5)

        # Update the last_angle for the next loop
        self.last_angle = current_angle
        
        return velocity_msg

    def image_callback(self, msg):
        # Convert the incoming ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Run object detection to find blocks
        detected_block = self.detect_block(cv_image)

        if detected_block is not None and self.robot_state["head_to_colored_block"]:
            # Get the bounding box and display it
            self.move_to_block(cv_image, detected_block)

            rospy.sleep(1)  # Wait for the claw to close if applicable

    def get_color_range(self):
        # Define color ranges for different colors in HSV format
        color_ranges = {
            "red": ((74, 105, 129), (180, 255, 255)),
            "blue": ((100, 150, 0), (140, 255, 255)),
            "green": ((35, 40, 40), (85, 255, 255)),
            "yellow": ((20, 100, 100), (30, 255, 255))  # Typical HSV range for yellow
        }
        # Return the HSV range for the selected color
        return color_ranges.get(self.target_color, ((0, 0, 0), (0, 0, 0)))

    def detect_block(self, cv_image):
        # Convert the image to HSV color space for better color segmentation
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Get the color range based on the target color
        lower, upper = self.get_color_range()

        # Create a mask that only includes the specified color range
        mask = cv2.inRange(hsv_image, lower, upper)

        # Find contours in the mask to detect objects
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # If any contours are found, return the largest one (assumed to be the block)
        if contours:
            return max(contours, key=cv2.contourArea)
        return None  # Return None if no blocks are detected

    def smooth_center(self, block_center_x, block_center_y):
        # Apply exponential smoothing (simple moving average)
        alpha = 0.5  # Smoothing factor (0.0 = no smoothing, 1.0 = no smoothing)
        self.smoothed_center_x = alpha * block_center_x + (1 - alpha) * self.smoothed_center_x
        self.smoothed_center_y = alpha * block_center_y + (1 - alpha) * self.smoothed_center_y

        return self.smoothed_center_x, self.smoothed_center_y

    def check_obstacle_ahead(self):
        # If LiDAR data is available, check if there's an obstacle within a threshold distance (e.g., 1 meter)
        if self.lidar_data:
            # Check the front part of the LiDAR scan (centered at the middle of the LiDAR range)
            front_data = self.lidar_data[len(self.lidar_data)//2 - 10: len(self.lidar_data)//2 + 10]
            
            min_distance = np.mean(front_data)  # Get the closest distance from the front
            rospy.loginfo(f"Obstacle distance: {min_distance} meters")  # Print the obstacle distance
            
            # If an obstacle is within 1 meter, return True
            return min_distance < 0.05  # Threshold for obstacle distance
        return False

    def move_to_block(self, cv_image, block):
        # Get the bounding box of the largest contour (the detected block)
        x, y, w, h = cv2.boundingRect(block)
        self.bounding = (x, y, w, h)

        # Calculate the center of the block
        block_center_x = x + w / 2
        block_center_y = y + h / 2

        # Remove the inversion of the y-coordinate (no longer flipping)
        smoothed_block_center_x, smoothed_block_center_y = self.smooth_center(block_center_x, block_center_y)

        # Assume the robot’s camera is aligned with the robot's center
        rospy.loginfo(f"Smoothed block detected at (x, y): ({smoothed_block_center_x}, {smoothed_block_center_y})")

        # Define a proximity threshold for stopping (in pixels)
        proximity_threshold = 5  # Pixel distance from the block center at which to stop (adjusted to 180px)

        # Calculate the Euclidean distance from the block center to the image center
        distance_to_block = math.sqrt((smoothed_block_center_x - 320) ** 2 + (smoothed_block_center_y - 240) ** 2)

        # Check for obstacles in front of the robot
        if self.check_obstacle_ahead():
            rospy.loginfo("Obstacle detected in front, stopping robot.")
            move_command = Twist()
            move_command.linear.x = 0  # Stop moving forward
            move_command.angular.z = 0  # Stop turning
            self.velocity_pub.publish(move_command)
            return

        # Create a Twist message for velocity control
        move_command = Twist()

        # If the robot is close enough to the block, stop moving
        if distance_to_block < proximity_threshold:
            move_command.linear.x = 0  # Stop moving forward/backward
            move_command.angular.z = 0  # Stop turning
            rospy.loginfo("Block is close enough, stopping the robot.")
        else:
            # Adjust the robot's movement based on the smoothed bounding box position
            tolerance_x = 50  # Adjust as necessary for stability
            tolerance_y = 50  # Adjust as necessary for stability

            if abs(smoothed_block_center_x - 320) < tolerance_x:
                # Adjust angular speed to turn towards the block (smaller adjustments)
                angular_speed = 0.5 * (smoothed_block_center_x - 320) / 320  # Normalize the speed to avoid large turns
                move_command.angular.z = - angular_speed  
                rospy.loginfo(f"Adjusting angular velocity: {move_command.angular.z}")
            else:
                move_command.angular.z = 0  # Stop turning when centered
                rospy.loginfo(f"not turning!")

            # Adjust linear speed to move forward or backward
            if abs(smoothed_block_center_y - 240) < tolerance_y:
                if smoothed_block_center_y > 240:
                    move_command.linear.x = 0.5  # Move forward 
                    rospy.loginfo("Moving forward")
                else:
                    move_command.linear.x = -0.5  # Move backward
                    rospy.loginfo("Moving backward")
            else:
                move_command.linear.x = 0  # Stop moving forward/backward when centered

        # Apply smoothing to velocity command to avoid sudden jumps
        if not hasattr(self, 'prev_linear_x'):
            self.prev_linear_x = 0
            self.prev_angular_z = 0

        # Smooth the movement commands
        alpha = 0.5  # Smooth factor for linear and angular velocities
        move_command.linear.x = alpha * move_command.linear.x + (1 - alpha) * self.prev_linear_x
        move_command.angular.z = alpha * move_command.angular.z + (1 - alpha) * self.prev_angular_z

        # Store previous values for next iteration
        self.prev_linear_x = move_command.linear.x
        self.prev_angular_z = move_command.angular.z

        # Publish the velocity command to move the robot
        self.velocity_pub.publish(move_command)

        # Publish bounding box center for debugging purposes
        self.bounding_box_pub.publish(Point(smoothed_block_center_x, smoothed_block_center_y, 0))

        # Display the result in the OpenCV window
        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Draw the bounding box
        cv2.imshow("Detected Block", cv_image)  # Ensure the window is shown
        cv2.waitKey(1)  # Add waitKey to update the window

    def run(self):
        while self.cur_x is None or self.cur_y is None or self.cur_yaw is None:
            pass
        
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if not(self.robot_state["head_to_colored_block"]):
                self.distance_to_goal = math.hypot(self.goal_y - self.cur_y, self.goal_x - self.cur_x)
                # Keep the robot running until we are very close to goal (9cm)
                if self.distance_to_goal > 0.09:
                    # Log the robot's state and distance to goal
                    rospy.loginfo("Distance to goal {0}".format(self.distance_to_goal))
                    rospy.loginfo("Goal coordinates ({0}, {1})".format(self.goal_x, self.goal_y))
                    # rospy.loginfo("Robot state {0}".format(self.robot_state))
                    # rospy.loginfo("Div Distances {0}".format(self.div_distance))
                    rospy.loginfo("Current coordinates ({0}, {1})".format(self.cur_x, self.cur_y))
                    # rospy.loginfo("Current angle {0}".format(self.cur_yaw))

                    if self.distance_to_goal < 0.15:
                        # If we're close enough to goal, move directly towards goal
                        vel = self.go_to_goal()
                    else:
                        self.calc_robot_state()
                        if self.robot_state["obstacle_detected"]:
                            vel = self.avoid_obstacle()
                        else:
                            vel = self.go_to_goal()

                    self.cmd_vel_pub.publish(vel)
                else:
                    rospy.loginfo("Goal Reached, now heading to colored block")
                    velocity_msg = Twist()
                    velocity_msg.linear.x = 0
                    velocity_msg.angular.z = 0
                    self.cmd_vel_pub.publish(velocity_msg)
                    self.robot_state["head_to_colored_block"] = True
            
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('jerry_robot', anonymous=True)
    JerryRobot(0, 2).run()