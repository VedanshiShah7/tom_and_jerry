#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def jerry_robot():
    rospy.init_node('jerry_robot', anonymous=True)
    pub = rospy.Publisher('/jerry/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    move_cmd = Twist()

    while not rospy.is_shutdown():
        move_cmd.linear.x = 0.0  # Stay still
        move_cmd.angular.z = 0.5  # Rotate in place
        pub.publish(move_cmd)
        rospy.loginfo("Jerry is rotating")
        rate.sleep()

if __name__ == '__main__':
    try:
        jerry_robot()
    except rospy.ROSInterruptException:
        pass
