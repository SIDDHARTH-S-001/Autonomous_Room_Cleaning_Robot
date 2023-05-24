#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def callback(data):
    ranges = data.ranges
    print(ranges)

def listener():
    rospy.init_node('lidar_listener', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
