#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
import math

# Wall-following state
class WallFollowingState:
    def __init__(self):
        self.following_wall = False  # Whether the robot is in wall-following mode
        self.wall_side = None  # The side of the wall being followed ('left' or 'right')
        self.in_corridor = False  # Whether the robot is in a corridor

class JerryRobot():
    def __init__(self, target_x, target_y):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.odom_cb)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        # Current heading of the robot.
        self.x = None
        self.y = None
        self.cur_yaw = None
        self.target_x = target_x
        self.target_y = target_y
        self.wall_following_state = WallFollowingState()
        self.v = 0
        self.w = 0
    
    def odom_cb(self, msg):
        """Callback function for `self.my_odom_sub`."""
        self.x = msg.y
        self.y = msg.x
        # msg.z is the angular z i.e. yaw of the robot
        self.cur_yaw = msg.z
    
    # Function to calculate Euclidean distance between current position and target
    def distance_to_target(self):
        return math.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2)

    # Function to calculate the angle between the robot's heading and the target
    def angle_to_target(self):
        return math.atan2(self.target_x - self.x, self.target_y - self.y)

    # Function to detect if a wall is present based on LIDAR scan
    def is_wall(self, lidar_scan, wall_threshold=0.05, min_points=5):
        """
        Detect if there is a wall by checking if a consecutive set of LIDAR values
        fall within a small range, indicating a flat surface.
        """
        consecutive_points = 0
        for i in range(len(lidar_scan) - 1):
            if abs(lidar_scan[i] - lidar_scan[i+1]) < wall_threshold:
                consecutive_points += 1
            else:
                consecutive_points = 0
            
            if consecutive_points >= min_points:
                return True
        return False

    # Function to calculate obstacle avoidance with wall following
    def obstacle_avoidance(self, lidar_scan, min_distance=1.0, wall_threshold=0.05):
        front_scan = lidar_scan[0:30] + lidar_scan[330:360]  # Approx front section (20 degrees on either side)
        left_scan = lidar_scan[60:180]  # Approx left region
        right_scan = lidar_scan[180:300]  # Approx right region
        min_front_distance = min(front_scan)
        min_left_distance = min(left_scan)
        min_right_distance = min(right_scan)

        # Check if we are in a corridor (walls on both sides)
        if min_left_distance < min_distance and min_right_distance < min_distance:
            self.wall_following_state.in_corridor = True
            self.wall_following_state.following_wall = False

            # Try to stay centered in the corridor
            if min_left_distance > min_right_distance:
                return 0.5  # Turn slightly right to center
            else:
                return -0.5  # Turn slightly left to center
        else:
            self.wall_following_state.in_corridor = False

        # If obstacle is too close in the front, avoid or follow a wall
        if min_front_distance < min_distance:
            if self.is_wall(lidar_scan):
                if self.wall_following_state.following_wall:
                    if self.wall_following_state.wall_side == 'left':
                        return 0.5  # Continue slight right turn to follow the wall on the left
                    else:
                        return -0.5  # Continue slight left turn to follow the wall on the right
                else:
                    # Determine which side to follow
                    if min_left_distance < min_right_distance:
                        self.wall_following_state.following_wall = True
                        self.wall_following_state.wall_side = 'left'
                        return 0.5  # Follow wall on the left
                    else:
                        self.wall_following_state.following_wall = True
                        self.wall_following_state.wall_side = 'right'
                        return -0.5  # Follow wall on the right
            else:
                # Isolated obstacle avoidance
                if min_left_distance < min_right_distance:
                    self.wall_following_state.following_wall = False
                    return 1.0  # Turn right (positive angular velocity)
                else:
                    self.wall_following_state.following_wall = False
                    return -1.0  # Turn left (negative angular velocity)

        # No obstacle in front, but still wall-following or corridor-following
        if self.wall_following_state.following_wall:
            side_scan = left_scan if self.wall_following_state.wall_side == 'left' else right_scan
            if min(side_scan) > min_distance:
                self.wall_following_state.following_wall = False  # Stop following if clear
            else:
                return 0.5 if self.wall_following_state.wall_side == 'left' else -0.5

        return 0  # No obstacle, no angular adjustment

    # Main function to compute velocities for target navigation and obstacle avoidance
    def compute_control(self, lidar_scan):
        # Parameters
        max_linear_velocity = 0.3  # Max forward speed
        max_angular_velocity = 0.4  # Max turn rate
        distance_threshold = 0.1  # Distance at which to stop near the target

        # Distance and angle to target
        distance = self.distance_to_target()
        target_angle = self.angle_to_target()
        
        # Compute the angle difference (heading error) between the robot's orientation and the target
        angle_diff = target_angle - self.cur_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # Normalize to [-pi, pi]

        # Initialize control signals
        linear_velocity = 0.0
        angular_velocity = 0.0

        # If far from target, move towards it
        if distance > distance_threshold:
            # Proportional control for target approach
            linear_velocity = max_linear_velocity * (distance / max(distance, 1.0))  # Adjust speed based on distance
            angular_velocity = max_angular_velocity * angle_diff  # Proportional control for orientation

            # Obstacle avoidance and wall-following logic
            avoidance_angular_velocity = self.obstacle_avoidance(lidar_scan)
            if avoidance_angular_velocity != 0:
                # Prioritize obstacle avoidance or wall following over target navigation
                angular_velocity = avoidance_angular_velocity
                linear_velocity = 0.2  # Slow down when avoiding obstacles or following walls

        return linear_velocity, angular_velocity
    
    def scan_cb(self, msg):
        self.v, self.w = self.compute_control(msg.ranges)

    def run(self):
        rate = rospy.Rate(10)
        twist = Twist()

        while self.x is None and self.y is None:
            pass

        while not rospy.is_shutdown():
            twist.linear.x = self.v
            twist.angular.z = self.w
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('jerry_robot', anonymous=True)
    JerryRobot(0, 3).run()
