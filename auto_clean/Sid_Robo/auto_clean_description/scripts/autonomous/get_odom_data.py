import rospy
from nav_msgs.msg import Odometry

def call_back(msg):
    print(msg.pose.pose)

rospy.init_node('get_odom_data')
odom_sub = rospy.Subscriber('odom', Odometry, call_back)
rospy.spin()