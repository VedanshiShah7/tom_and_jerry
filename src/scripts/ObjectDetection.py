#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Point  # For publishing the bounding box center

class ObjectDetection:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('object_detection', anonymous=True)

        # Create a publisher for the claw commands
        self.servo_pub = rospy.Publisher('/servo', String, queue_size=1)

        # Subscribe to the camera images to detect blocks
        self.image_sub = rospy.Subscriber('/cv_camera/image_raw', Image, self.image_callback)

        # Initialize CvBridge to convert ROS images to OpenCV format
        self.bridge = CvBridge()

        # Create a publisher for the robot's movement commands (Assumed topic)
        self.velocity_pub = rospy.Publisher('/cmd_vel', String, queue_size=1)

        # Publisher to send the bounding box coordinates (center)
        self.bounding_box_pub = rospy.Publisher('/bounding_box', Point, queue_size=1)

        self.bounding = None

    def image_callback(self, msg):
        # Convert the incoming ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Run object detection to find blocks
        detected_block = self.detect_block(cv_image)
        print(detected_block)

        if detected_block is not None:
            # Get the bounding box and display it
            self.move_to_block(cv_image, detected_block)

            rospy.sleep(1)  # Wait for the claw to close if applicable


    def detect_block(self, cv_image):
        # Simple color detection logic (this example is for red blocks)
        # Convert the image to HSV color space for better color segmentation
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the HSV color range for red detection (these values may need tweaking)
        lower_red = (74, 105, 129)
        upper_red = (180, 255, 255)

        # Create a mask that only includes the specified color range
        mask = cv2.inRange(hsv_image, lower_red, upper_red)

        # Find contours in the mask to detect objects
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # If any contours are found, return the largest one (assumed to be the block)
        if contours:
            return max(contours, key=cv2.contourArea)
        return None  # Return None if no blocks are detected

    def move_to_block(self, cv_image, block):
        # Get the bounding box of the largest contour (the detected block)
        x, y, w, h = cv2.boundingRect(block)
        self.bounding = (x, y, w, h)

        # Debugging: Check if bounding box coordinates are correct
        rospy.loginfo(f"Bounding box: x={x}, y={y}, w={w}, h={h}")

        # Ensure that bounding box coordinates are valid
        if w > 0 and h > 0:
            # Draw the bounding box on the image (green color with 2-pixel thickness)
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Publish the center of the bounding box to the new topic
            bounding_box_center = Point()
            bounding_box_center.x = x + w / 2
            bounding_box_center.y = y + h / 2
            bounding_box_center.z = 0  # You could use depth if you have it
            self.bounding_box_pub.publish(bounding_box_center)

        # Calculate the center of the block
        block_center_x = x + w / 2
        block_center_y = y + h / 2

        # Assume the robot’s camera is aligned with the robot's center
        rospy.loginfo(f"Block detected at (x, y): ({block_center_x}, {block_center_y})")

        # Simple movement logic to move towards the block
        move_command = String()
        if block_center_x < 320:  # Assuming 640x480 resolution and the block is left
            move_command.data = "move_left"
        elif block_center_x > 320:  # Assuming 640x480 resolution and the block is right
            move_command.data = "move_right"
        elif block_center_y < 240:  # Block is higher (closer)
            move_command.data = "move_forward"
        else:  # Block is lower (farther)
            move_command.data = "move_backward"

        # Publish the movement command (you'll need to implement robot movement logic)
        self.velocity_pub.publish(move_command)

        # Show the image with the bounding box
        cv2.imshow("Bounding Box", cv_image)  # Show the image with bounding box
        cv2.waitKey(10)  # Wait for 10ms for the window to update



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
