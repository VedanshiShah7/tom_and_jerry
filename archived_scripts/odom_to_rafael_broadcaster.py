#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

def broadcast_transform():
    rospy.init_node('odom_to_rafael_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    transform = geometry_msgs.msg.TransformStamped()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "odom"  # Parent frame
        transform.child_frame_id = "rafael/base_footprint"  # Child frame

        # Example values for position and orientation
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        br.sendTransform(transform)
        rate.sleep()

if __name__ == "__main__":
    try:
        broadcast_transform()
    except rospy.ROSInterruptException:
        pass
