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
JERRY_OBJ_THRESHOLD = 0.3

class JerryRobot:
    def __init__(self):
        rospy.init_node('jerry_robot', anonymous=True)
        self.jerry_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        
        # Subscribe to Jerry's LIDAR
        self.jerry_scan_sub = rospy.Subscriber('/scan', LaserScan, self.jerry_scan_cb)

        # Subscribe to Jerry's odom
        rospy.Subscriber('/jerry_odom', Point, self.jerry_odom_cb)

        self.jerry_image_sub = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.jerry_image_cb)
        self.target_color = 'red'  # Target color (e.g., 'red', 'green', 'blue')
        self.bridge = CvBridge()

        # Define color range in HSV
        self.color_ranges = {
            "red": ((74, 105, 129), (180, 255, 255)),
            "blue": ((100, 150, 0), (140, 255, 255)),
            "green": ((35, 40, 40), (85, 255, 255)),
            "yellow": ((20, 150, 150), (30, 255, 255)) 
        }

        self.jerry_x = None
        self.jerry_y = None
        self.jerry_yaw = None

        self.jerry_linear_vel = 0
        self.jerry_angular_vel = 0

        self.jerry_goal_x = 1.7
        self.jerry_goal_y = 0

        self.jerry_robot_state = {"obstacle_detected": False, "finding_block": False, "avoid_angular_vel": 0}
        self.jerry_div_distance = {"0": [], "1": [], "2": [], "3": [], "4": [], "5": [], "6": [], "7": []}
        self.jerry_div_cost = {"0": 0, "1": 1, "2": 2, "3": 3, "4": 4, "5": -3, "6": -2, "7": -1}
    
    def jerry_odom_cb(self, msg):
        self.jerry_x = msg.x
        self.jerry_y = msg.y
        self.jerry_yaw = msg.z
    
    def jerry_image_cb(self, msg):
        # Convert the compressed image to OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)  # Convert byte data to numpy array
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode into an image

        # Check if the frame was read successfully
        if frame is None:
            rospy.logwarn("Failed to decode image!")
            return

        # Convert image to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create a mask for the target color
        mask = cv2.inRange(hsv_frame, self.lower_color, self.upper_color)

        # Find contours of the detected color
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            self.target_found = True
            largest_contour = max(contours, key=cv2.contourArea)
            # Get the center of the largest contour
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                if self.jerry_robot_state["finding_block"]:
                    self.move_towards_block(cX, cY, frame)
        else:
            self.target_found = False
    
    def move_towards_block(self, x, y, frame):
        # Robot motion parameters
        center_x = 320  # Assume image width is 640px
        center_y = 240  # Assume image height is 480px
        stop_distance = 30  # pixels, when to stop before the block
        threshold_distance = 50  # pixels, threshold for movement

        # Compute error in x and y directions
        error_x = center_x - x
        error_y = center_y - y

        # Visualize the detection
        cv2.circle(frame, (x, y), 10, (0, 255, 0), -1)
        cv2.imshow("Detected Frame", frame)
        cv2.waitKey(1)

        # Print the error values for debugging
        rospy.loginfo(f"Error X: {error_x}, Error Y: {error_y}")

        # Move the robot directly towards the block by adjusting its linear velocity
        if abs(error_y) > threshold_distance:
            self.jerry_linear_vel = 0.1  # Move slower to approach the object
        else:
            self.jerry_linear_vel = 0.0  # Stop moving forward when close enough

        # Adjust the angular velocity based on the error_x
        if abs(error_x) > threshold_distance:
            # Proportional control for angular velocity
            angular_velocity_factor = 0.005  # Adjust this factor for more/less rotation speed
            self.jerry_angular_vel = angular_velocity_factor * error_x  # Rotate proportionally to error_x
        else:
            self.jerry_angular_vel = 0.0  # Stop rotating when aligned
    
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
                    max_distance = JERRY_OBJ_THRESHOLD
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
    
    def jerry_go_to_goal(self):
        distance = math.sqrt((self.jerry_goal_x - self.jerry_x)**2 + (self.jerry_goal_y - self.jerry_y)**2)
        
        phi = math.atan2(self.jerry_goal_y - self.jerry_y, self.jerry_goal_x - self.jerry_x)
        
        delta_theta = phi - self.jerry_yaw
        delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]
        
        if self.jerry_robot_state["obstacle_detected"]:
            self.jerry_linear_vel = -0.05
            self.jerry_angular_vel = self.jerry_robot_state["avoid_angular_vel"]
        else:
            self.jerry_linear_vel = 0.2
            self.jerry_angular_vel = delta_theta

    def run_game(self):
        while self.jerry_x is None:
            pass
        
        while not rospy.is_shutdown():

            # If Jerry is still in the obstacle course, then Jerry moves towards goal coordinate while avoiding obstacles
            if not(self.jerry_robot_state["finding_block"]):
                self.calc_jerry_robot_state()
                self.jerry_go_to_goal()
            
            rospy.loginfo(f"Jerry coordinates: ({self.jerry_x}, {self.jerry_y})")
            rospy.loginfo(f"Jerry yaw: {self.jerry_yaw}")
            rospy.loginfo(f"Jerry Linear and Angular Vel: ({self.jerry_linear_vel}, {self.jerry_angular_vel})")

            # Goal coordinate here is after the obstacle course and before the colored blocks
            jerry_distance_to_goal = math.sqrt((self.jerry_goal_x - self.jerry_x)**2 + (self.jerry_goal_y - self.jerry_y)**2)

            rospy.loginfo(f"Jerry distance to Goal: {jerry_distance_to_goal}")
            rospy.loginfo("")

            cmd_vel_jerry = Twist()
            
            # If Jerry reached end of obstacle course, switch to finding colored block mode
            if jerry_distance_to_goal < 0.11:
                self.jerry_robot_state["finding_block"] = True
            
            # If Jerry is close enough to colored block, pause everything and declare Jerry wins
            if self.jerry_x > 2.5:
                cmd_vel_jerry.linear.x = 0
                cmd_vel_jerry.angular.z = 0
                self.jerry_vel_pub.publish(cmd_vel_jerry)
                rospy.loginfo("Jerry reached Goal! Jerry Wins!")
                break

            cmd_vel_jerry.linear.x = self.jerry_linear_vel
            cmd_vel_jerry.angular.z = self.jerry_angular_vel

            self.jerry_vel_pub.publish(cmd_vel_jerry)
            self.rate.sleep()


    def shutdown(self):
        """Stop the robot when shutting down."""
        rospy.loginfo("Shutting down...")
        self.jerry_vel_pub.publish(Twist())  # Stop the robot
        rospy.sleep(1)

if __name__ == "__main__":
    try:
        game = JerryRobot()
        rospy.on_shutdown(game.shutdown)
        game.run_game()
    except rospy.ROSInterruptException:
        pass