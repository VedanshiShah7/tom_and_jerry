#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Point, Twist

class ObjectDetection:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('object_detection', anonymous=True)

        # Get the target color parameter, defaulting to 'red'
        self.target_color = rospy.get_param('~target_color', 'red')

        # Create a publisher for the claw commands
        self.servo_pub = rospy.Publisher('/servo', String, queue_size=1)

        # Subscribe to the camera images to detect blocks
        self.image_sub = rospy.Subscriber('/cv_camera/image_raw', Image, self.image_callback)

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

        # Set the refresh rate to 10Hz
        self.rate = rospy.Rate(10)  

    def image_callback(self, msg):
        # Convert the incoming ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

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

    def move_to_block(self, cv_image, block):
        # Get the bounding box of the largest contour (the detected block)
        x, y, w, h = cv2.boundingRect(block)
        self.bounding = (x, y, w, h)

        # Calculate the center of the block
        block_center_x = x + w / 2
        block_center_y = y + h / 2

        # Smooth the center positions
        smoothed_block_center_x, smoothed_block_center_y = self.smooth_center(block_center_x, block_center_y)

        # Assume the robot’s camera is aligned with the robot's center
        rospy.loginfo(f"Smoothed block detected at (x, y): ({smoothed_block_center_x}, {smoothed_block_center_y})")

        # Define the tolerance range for the center of the image
        tolerance_x = 50  # Adjust as necessary for stability
        tolerance_y = 50  # Adjust as necessary for stability

        # Create a Twist message for velocity control
        move_command = Twist()

        # Adjust the robot's movement based on the smoothed bounding box position
        if abs(smoothed_block_center_x - 320) > tolerance_x:
            # Adjust angular speed to turn towards the block
            if smoothed_block_center_x < 320:
                move_command.angular.z = -0.5  # Turn right
                rospy.loginfo("Turning right")
            else:
                move_command.angular.z = 0.5  # Turn left
                rospy.loginfo("Turning left")
                
        else:
            move_command.angular.z = 0  # Stop turning when centered

        # Adjust linear speed to move forward or backward
        if abs(smoothed_block_center_y - 240) > tolerance_y:
            if smoothed_block_center_y < 240:
                move_command.linear.x = -0.5  # Move backward
                rospy.loginfo("Moving backward")
            else:
                move_command.linear.x = 0.5  # Move forward
                rospy.loginfo("Moving forward")
        else:
            move_command.linear.x = 0  # Stop moving forward/backward when centered

        # Publish the velocity command
        self.velocity_pub.publish(move_command)

        # Show the image with the bounding box
        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.imshow("Bounding Box", cv_image)
        cv2.waitKey(10)


    def toggle_claw(self, command):
        # Publish the command to open or close the claw
        self.servo_pub.publish(command)  # Send the command string
        rospy.loginfo(f"Claw command sent: {command}")  # Log the command for debugging
        rospy.sleep(0.5)  # Adding a small delay to ensure claw has time to move

if __name__ == '__main__':
    try:
        robot = ObjectDetection()  # Create an instance of the robot
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass  # Handle ROS interruption exceptions gracefully
