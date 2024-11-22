import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt

class TomChaser(Node):
    def __init__(self):
        super().__init__('tom_chaser')
        
        # Velocity publisher
        self.velocity_publisher = self.create_publisher(Twist, '/tom/cmd_vel', 10)
        
        # Odometry subscriber for Tom
        self.tom_pose_subscriber = self.create_subscription(
            Odometry,
            '/tom/odom',
            self.update_tom_pose,
            10)
        
        # Odometry subscriber for Jerry
        self.jerry_pose_subscriber = self.create_subscription(
            Odometry,
            '/jerry/odom',
            self.update_jerry_pose,
            10)

        self.tom_pose = None
        self.jerry_pose = None
        self.chase_timer = self.create_timer(0.1, self.chase_jerry)
    
    def update_tom_pose(self, msg):
        # Update Tom's position from odometry data
        self.tom_pose = msg.pose.pose.position
    
    def update_jerry_pose(self, msg):
        # Update Jerry's position from odometry data
        self.jerry_pose = msg.pose.pose.position
    
    def chase_jerry(self):
        if not self.tom_pose or not self.jerry_pose:
            # Wait for both positions to be initialized
            return

        # Calculate the distance and angle to Jerry
        dx = self.jerry_pose.x - self.tom_pose.x
        dy = self.jerry_pose.y - self.tom_pose.y
        distance = sqrt(dx**2 + dy**2)
        angle_to_jerry = atan2(dy, dx)
        
        # Create a Twist message
        cmd_vel = Twist()
        
        if distance > 0.1:  # Threshold to stop when near Jerry
            # Linear velocity proportional to the distance
            cmd_vel.linear.x = 0.5 * distance
            
            # Angular velocity proportional to the angle difference
            cmd_vel.angular.z = 1.0 * angle_to_jerry
        else:
            # Stop if close enough
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.get_logger().info("Tom tagged Jerry!")
        
        # Publish velocity
        self.velocity_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    tom_chaser = TomChaser()
    rclpy.spin(tom_chaser)
    tom_chaser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
