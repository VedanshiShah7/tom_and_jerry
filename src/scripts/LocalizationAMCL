import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class LocalizationAMCL:
    def __init__(self):
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_pose)
        self.current_pose = None

    def update_pose(self, msg):
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
