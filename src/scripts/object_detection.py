#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class ColorBlockDetector:
    def __init__(self, target_color):
        self.target_color = target_color  # Target color (e.g., 'red', 'green', 'blue')
        self.bridge = CvBridge()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.image_callback)
        self.twist = Twist()
        self.target_found = False

        # Define color range in HSV
        self.color_ranges = {
            "red": ((74, 105, 129), (180, 255, 255)),
            "blue": ((100, 150, 0), (140, 255, 255)),
            "green": ((35, 40, 40), (85, 255, 255)),
            "yellow": ((20, 150, 150), (30, 255, 255)) 
        }
        
        if target_color not in self.color_ranges:
            raise ValueError("Invalid color name. Choose from 'red', 'green', or 'blue'.")
        
        self.lower_color, self.upper_color = self.color_ranges[target_color]

    def image_callback(self, msg):
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
                self.move_towards_block(cX, cY, frame)
        else:
            self.target_found = False
            self.stop_movement()

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

        # Stop the robot if it's close to the block
        if abs(error_x) < stop_distance and abs(error_y) < stop_distance:
            self.stop_movement()
            rospy.loginfo("Block detected, stopping.")
            return

        # Move the robot directly towards the block by adjusting its linear velocity
        if abs(error_y) > threshold_distance:
            self.twist.linear.x = 0.1  # Move slower to approach the object
        else:
            self.twist.linear.x = 0.0  # Stop moving forward when close enough

        # Adjust the angular velocity based on the error_x
        if abs(error_x) > threshold_distance:
            # Proportional control for angular velocity
            angular_velocity_factor = 0.005  # Adjust this factor for more/less rotation speed
            self.twist.angular.z = angular_velocity_factor * error_x  # Rotate proportionally to error_x
        else:
            self.twist.angular.z = 0.0  # Stop rotating when aligned

        # Publish the velocity command
        self.cmd_vel_pub.publish(self.twist)

    def stop_movement(self):
        # Stop all movement
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

def main():
    rospy.init_node('color_block_detector', anonymous=True)

    # Example: detect red blocks
    target_color = 'green'  # Change this to 'green' or 'blue' to detect other colors
    detector = ColorBlockDetector(target_color)

    rospy.spin()

if __name__ == '__main__':
    main()
