#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from tf.transformations import euler_from_quaternion
import numpy as np

global occ_grid_pub, pose_x, pose_y, pose_yaw

def scan_data(scan_msg):    
    global occ_grid_pub, pose_x, pose_y, pose_yaw

    ranges = np.array(scan_msg.ranges)
    angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

    x = np.cos(angles)*ranges
    y = np.sin(angles)*ranges

    # compute x and y coordinates in map frame
    x_map = pose_x + np.cos(pose_yaw)*x - np.sin(pose_yaw)
    y_map = pose_y + np.sin(pose_yaw)*y + np.cos(pose_yaw)

    # convert x and y into occupancy grid indices
    map_width = 100 # meters
    map_height = 100 # meters
    map_resolution = 0.1 # meters
    x_indices = np.round((x_map + map_width/2)/map_resolution).astype(int)
    y_indices = np.round((y_map + map_height/2)/map_resolution).astype(int)

    occupancy_grid = OccupancyGrid
    occupancy_grid.header.stamp = rospy.Time.now()
    occupancy_grid.header.frame_id = 'map'
    occupancy_grid.info.resolution = map_resolution
    occupancy_grid.info.height = int(map_height/map_resolution)
    occupancy_grid.info.width = (map_width/map_resolution)
    occupancy_grid.info.origin.position.x = -map_width/2
    occupancy_grid.info.origin.position.y = -map_height/2
    occupancy_grid.info.origin.orientation.w = 1.0

    for i in range(len(x_indices)):
        if (x_indices[i] >= 0 and x_indices[i] < occupancy_grid.info.width and 
            y_indices[i] >= 0 and y_indices[i] < occupancy_grid.info.height):

            occupancy_grid.data[x_indices[i] + y_indices[i] * occupancy_grid.info.width] = 100
        
    occ_grid_pub.publish(occupancy_grid)




def pose_data(pose_msg):
    global occ_grid_pub, pose_x, pose_y, pose_yaw
    pose_msg = Odometry
    pose_x = pose_msg.pose.pose.position.x
    pose_y = pose_msg.pose.pose.position.y

    (roll, pitch, yaw) = euler_from_quaternion([pose_msg.pose.orientation.x,
                                                pose_msg.pose.orientation.y,
                                                pose_msg.pose.orientation.z,
                                                pose_msg.pose.orientation.w])
    pose_yaw = yaw

if __name__ == "__main__":
    rospy.init_node('Mapping')
    rospy.Subscriber('/scan', LaserScan, scan_data)
    rospy.Subscriber('/odom', Odometry, pose_data)

    occ_grid_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
