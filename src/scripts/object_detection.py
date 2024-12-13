#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, LaserScan
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Point, Twist
import math
import numpy as np

class ObjectDetection:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('object_detection', anonymous=True)

        # Get the target color parameter, defaulting to 'red'
        self.target_color = rospy.get_param('~target_color', 'red')

        # Create a publisher for the claw commands
        self.servo_pub = rospy.Publisher('/servo', String, queue_size=1)

        # Subscribe to the camera images to detect blocks
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.image_callback)

        # Subscribe to the LiDAR data
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

        # Initialize CvBridge to convert ROS images to OpenCV format
        self.bridge = CvBridge()

        # Create a publisher for the robot's movement commands (using Twist for cmd_vel)
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Publisher to send the bounding box coordinates (center)
        self.bounding_box_pub = rospy.Publisher('/bounding_box', Point, queue_size=1)

        self.bounding = None

        # Initialize smoothing values for the block center
        self.smoothed_center_x = 320  # Initial center is the middle of the image (640x480)
        self.smoothed_center_y = 240  # Initial center is the middle of the image (640x480)

        # Initialize LiDAR data
        self.lidar_data = None

        # Set the refresh rate to 10Hz
        self.rate = rospy.Rate(10)

    def lidar_callback(self, msg):
        # Store the LiDAR distance data
        self.lidar_data = msg.ranges

    def image_callback(self, msg):
        # Convert the incoming ROS Image message to OpenCV format
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Run object detection to find blocks
        detected_block = self.detect_block(cv_image)

        if detected_block is not None:
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

if __name__ == '__main__':
    try:
        # Instantiate the object detection class to start processing
        detector = ObjectDetection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass