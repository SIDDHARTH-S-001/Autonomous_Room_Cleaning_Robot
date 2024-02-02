#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped
import numpy as np

robot_x = 0.0
robot_y = 0.0

# Lidar Specifications
scan_min_range = 0.05
scan_max_range = 8.00

# Map Specifications
map_width = 20
map_height = 20
map_resolution = 0.05

def odom_data(pose_data):
    global robot_x, robot_y
    robot_x = pose_data.pose.pose.position.x
    robot_y = pose_data.pose.pose.position.y

def scan_data(data):
    origin_x = robot_x
    origin_y = robot_y

    ogm = OccupancyGrid()
    ogm.header.frame_id = "map"
    ogm.header.stamp = rospy.Time.now()
    ogm.info.resolution = map_resolution
    ogm.info.height = int(map_height/map_resolution)
    ogm.info.width = int(map_width/map_resolution)
    ogm.info.origin.position.x = origin_x - ((map_width)/2)
    ogm.info.origin.position.y = origin_y - ((map_height)/2)
    ogm.data = [-1] * ogm.info.width * ogm.info.height

    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "map" # Parent Frame
    transform.child_frame_id = "odom" # Child Frame
    transform.transform.translation.x = 0.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0    
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    transform.transform.rotation.w = 1.0

    tf_broadcaster.sendTransform(transform)

    scan_ranges_data = data.ranges
    min_angle  = data.angle_min
    angle_inc = data.angle_increment

    for i, scan_value in enumerate(scan_ranges_data):
        if ((scan_value > scan_min_range) and (scan_value < scan_max_range)):
            angle = min_angle + (i * angle_inc)

            x = int((scan_value*np.cos(angle) + map_width/2 - origin_x)/map_resolution)
            y = int((scan_value*np.sin(angle) + map_height/2 - origin_y)/map_resolution)

            ogm.data[x + (y * ogm.info.width)] = 100
    
    ogm_pub.publish(ogm)


if __name__ == "__main__":
    rospy.init_node("slam", anonymous=True)
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    ogm_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=10)
    rospy.Subscriber("/odom", Odometry, odom_data)
    rospy.Subscriber("/scan", LaserScan, scan_data)
    rospy.spin()
