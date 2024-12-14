#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class ObjectDetection:
    def __init__(self, target_color):
        # Initialize the target color parameter, defaulting to 'red' if not provided.
        self.target_color = rospy.get_param('~target_color', 'red')

        # Bridge to convert ROS image messages to OpenCV images.
        self.bridge = CvBridge()

        # Publisher to control robot movement via Twist messages.
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber for compressed camera image data.
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.image_callback)

        # Subscriber for LIDAR scan data.
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

        # Initialize Twist message for robot velocity control.
        self.twist = Twist()

        # Flags and variables for tracking the target and managing LIDAR data.
        self.target_found = False
        self.closest_distance = float('inf')  # Closest object distance initialized to a very large value.
        self.lidar = None  # Placeholder for current LIDAR data.
        self.prev_lidar = None  # Placeholder for previous LIDAR data.

        # Define HSV color ranges for detecting specific colors.
        self.color_ranges = {
            "red": ((74, 105, 129), (180, 255, 255)),
            "blue": ((100, 150, 0), (140, 255, 255)),
            "green": ((35, 40, 40), (85, 255, 255)),
            "yellow": ((20, 100, 100), (30, 255, 255)) 
        }

        # Validate the target color and retrieve its HSV range.
        if target_color not in self.color_ranges:
            raise ValueError("Invalid color name. Choose from 'red', 'green', 'blue', or 'yellow'.")

        self.lower_color, self.upper_color = self.color_ranges[target_color]

    def image_callback(self, msg):
        """Callback to process incoming camera images and detect the target color."""
        # Convert the compressed image data to a numpy array.
        np_arr = np.frombuffer(msg.data, np.uint8)

        # Decode the numpy array into an OpenCV image.
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Check if the image frame is valid.
        if frame is None:
            rospy.logwarn("Failed to decode image!")
            return

        # Convert the image to HSV color space for better color segmentation.
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create a binary mask where the target color range is white, and other regions are black.
        mask = cv2.inRange(hsv_frame, self.lower_color, self.upper_color)

        # Find contours in the mask to identify potential target regions.
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            self.target_found = True  # Indicate that the target is found.

            # Get the largest contour based on area, assuming it's the target.
            largest_contour = max(contours, key=cv2.contourArea)

            # Compute the center of the largest contour using moments.
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                self.move_towards_block(cX, cY, frame)  # Adjust movement based on the target's location.
        else:
            self.target_found = False  # No target detected.
            self.stop_movement()  # Stop the robot.

    def lidar_callback(self, msg):
        """Callback to process LIDAR data for obstacle detection and avoidance."""
        # Save the previous LIDAR data if available.
        if self.lidar is not None:
            self.prev_lidar = self.lidar

        # Extract and clean LIDAR data.
        lidar_data = list(msg.ranges)  # Convert the LIDAR ranges to a list.

        # Filter out invalid or out-of-range LIDAR readings.
        for i in range(len(lidar_data)):
            if lidar_data[i] < 0.12 or lidar_data[i] > 3.5:  # Remove values too close or too far.
                lidar_data[i] = 0

        self.lidar = lidar_data  # Update the current LIDAR data.

        # Extract data from specific angular ranges: left (350°-360°) and right (0°-10°).
        self.left_lidar = self.lidar[350:360]
        self.right_lidar = self.lidar[0:10]

        # Log LIDAR information for debugging purposes.
        rospy.loginfo(f"Left Lidar: {self.left_lidar} ... (min: {min(self.left_lidar)})")
        rospy.loginfo(f"Right Lidar: {self.right_lidar} ... (min: {min(self.right_lidar)})")

        # Determine the closest object distance in the left and right regions.
        closest_distance_left = min(self.left_lidar) if self.left_lidar else float('inf')
        closest_distance_right = min(self.right_lidar) if self.right_lidar else float('inf')

        # Identify the overall closest object distance.
        closest_distance = min(closest_distance_left, closest_distance_right)

        if closest_distance == float('inf'):
            # If no valid distance readings, stop the robot.
            self.stop_movement()
            rospy.loginfo("Closest LIDAR distance is inf, stopping movement.")
            return

        if closest_distance < 0.5:
            # Stop the robot if an obstacle is detected within 0.5 meters.
            self.stop_movement()

    def move_towards_block(self, x, y, frame):
        """Move the robot towards the detected target block."""
        # Image center coordinates (assuming fixed dimensions of 640x480).
        center_x = 320
        center_y = 240

        # Distance thresholds for stopping and movement.
        stop_distance = 0.5  # Stop when within 0.5 meters of the target.
        threshold_distance = 1.0  # Allowable error threshold for adjustments.

        # Calculate error in the x and y directions.
        error_x = center_x - x
        error_y = center_y - y

        # Draw a visualization of the target location on the frame.
        cv2.circle(frame, (x, y), 10, (0, 255, 0), -1)
        cv2.imshow("Detected Frame", frame)
        cv2.waitKey(1)

        # Log error values for debugging purposes.
        rospy.loginfo(f"Error X: {error_x}, Error Y: {error_y}, Closest LIDAR Distance: {self.closest_distance}")

        if self.closest_distance < stop_distance:
            # Stop movement if the robot is too close to the target.
            self.stop_movement()
            rospy.loginfo("Block detected, stopping due to LIDAR distance.")
            return

        # Adjust linear velocity to move towards the target block.
        if abs(error_y) > threshold_distance:
            self.twist.linear.x = 0.1  # Move forward at a slow speed.
        else:
            self.twist.linear.x = 0.0  # Stop moving forward when close enough.

        # Adjust angular velocity based on horizontal alignment with the target.
        if abs(error_x) > threshold_distance:
            angular_velocity_factor = 0.005  # Factor to control rotation speed.
            self.twist.angular.z = angular_velocity_factor * error_x  # Rotate proportionally to error_x.
        else:
            self.twist.angular.z = 0.0  # Stop rotating when aligned.

        # Publish the updated velocity command to the robot.
        self.cmd_vel_pub.publish(self.twist)

    def stop_movement(self):
        """Stop all robot movements."""
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)


def main():
    rospy.init_node('object_detection', anonymous=True)  # Initialize the ROS node.

    # Create an instance of the ObjectDetection class.
    detector = ObjectDetection(target_color)

    # Keep the node running until manually terminated.
    rospy.spin()

if __name__ == '__main__':
    main()
