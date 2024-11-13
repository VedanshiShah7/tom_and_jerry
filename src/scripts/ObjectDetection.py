#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Point, Twist
import math

class PathPlanner:
    def __init__(self, grid_size=10):
        self.grid_size = grid_size
        self.grid = [[0] * grid_size for _ in range(grid_size)]
        
    def plan_path(self, start, goal):
        return [start, goal]  # Simple straight-line path (replace with A* for actual implementation)
    
    def update_grid(self, obstacles):
        for obs in obstacles:
            x, y = obs
            self.grid[x][y] = 1  # Mark obstacle in grid

class ObjectDetection:
    def __init__(self):
        rospy.init_node('object_detection', anonymous=True)
        self.target_color = rospy.get_param('~target_color', 'yellow')
        self.servo_pub = rospy.Publisher('/servo', String, queue_size=1)
        self.image_sub = rospy.Subscriber('/cv_camera/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.bounding_box_pub = rospy.Publisher('/bounding_box', Point, queue_size=1)
        self.bounding = None
        self.smoothed_center_x = 320
        self.smoothed_center_y = 240
        self.rate = rospy.Rate(10)
        self.path_planner = PathPlanner(grid_size=10)
        self.start_position = (1, 1)  # Initial position
        self.goal_position = None
        self.prev_position = self.start_position

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        detected_block = self.detect_block(cv_image)

        if detected_block is not None:
            self.move_to_block(cv_image, detected_block)

    def get_color_range(self):
        color_ranges = {
            "red": ((74, 105, 129), (180, 255, 255)),
            "blue": ((100, 150, 0), (140, 255, 255)),
            "green": ((35, 40, 40), (85, 255, 255)),
            "yellow": ((20, 100, 100), (30, 255, 255))
        }
        return color_ranges.get(self.target_color, ((0, 0, 0), (0, 0, 0)))

    def detect_block(self, cv_image):
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower, upper = self.get_color_range()
        mask = cv2.inRange(hsv_image, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            return max(contours, key=cv2.contourArea)
        return None

    def move_to_block(self, cv_image, block):
        x, y, w, h = cv2.boundingRect(block)
        self.bounding = (x, y, w, h)
        block_center_x = x + w / 2
        block_center_y = y + h / 2
        smoothed_block_center_x, smoothed_block_center_y = self.smooth_center(block_center_x, block_center_y)

        rospy.loginfo(f"Smoothed block detected at (x, y): ({smoothed_block_center_x}, {smoothed_block_center_y})")

        # Define the proximity threshold for stopping
        proximity_threshold = 5
        move_command = Twist()

        self.goal_position = (int(smoothed_block_center_x / 64), int(smoothed_block_center_y / 64))

        # Plan the path in real-time
        path = self.path_planner.plan_path(self.prev_position, self.goal_position)

        # Overlay path and goal on the image
        self.overlay_path_on_image(cv_image, path, smoothed_block_center_x, smoothed_block_center_y)

        # Execute movement
        self.follow_path(path, move_command)

        # Update previous position
        self.prev_position = self.goal_position

    def smooth_center(self, block_center_x, block_center_y):
        alpha = 0.5  # Smoothing factor
        self.smoothed_center_x = alpha * block_center_x + (1 - alpha) * self.smoothed_center_x
        self.smoothed_center_y = alpha * block_center_y + (1 - alpha) * self.smoothed_center_y
        return self.smoothed_center_x, self.smoothed_center_y

    def overlay_path_on_image(self, cv_image, path, block_center_x, block_center_y):
        x, y, w, h = self.bounding
        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        for i in range(len(path) - 1):
            start = (int(path[i][0] * 64), int(path[i][1] * 64))
            end = (int(path[i+1][0] * 64), int(path[i+1][1] * 64))
            cv2.line(cv_image, start, end, (255, 0, 0), 2)

        cv2.circle(cv_image, (int(block_center_x), int(block_center_y)), 5, (0, 0, 255), -1)
        cv2.imshow("Object Detection with Path", cv_image)
        cv2.waitKey(1)

    def follow_path(self, path, move_command):
        for step in path:
            # move_command.linear.x = 0.2  # Small step forward
            # move_command.angular.z = 0  # Straight path for now
            # self.velocity_pub.publish(move_command)
            rospy.sleep(0.5)  # Sleep for a shorter duration to prevent overshooting

if __name__ == '__main__':
    try:
        robot = ObjectDetection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
