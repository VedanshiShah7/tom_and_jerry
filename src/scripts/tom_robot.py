#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def tom_robot():
    rospy.init_node('tom_robot', anonymous=True)
    pub = rospy.Publisher('/tom/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    move_cmd = Twist()

    while not rospy.is_shutdown():
        move_cmd.linear.x = 0.5  # Move forward
        move_cmd.angular.z = 0.0  # No rotation
        pub.publish(move_cmd)
        rospy.loginfo("Tom is moving forward")
        rate.sleep()

if __name__ == '__main__':
    try:
        tom_robot()
    except rospy.ROSInterruptException:
        pass
