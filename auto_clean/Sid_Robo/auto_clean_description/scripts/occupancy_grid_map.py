#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf.transformations import euler_from_quaternion
from tf2_ros import TransformBroadcaster

map_width = 100 # meters
map_height = 100 # meters
map_resolution = 0.1 # meters
scan_range = 8 # meters


def scan_data(data):
    global occupancy_grid
    # initialise occupancy grid
    occupancy_grid = OccupancyGrid()
    pose_msg = PoseStamped()

    # Frame Transformations
    tf_transform.header.frame_id = 'map'
    tf_transform.child_frame_id = 'base_link'
    tf_transform.transform.translation.x = 0
    tf_transform.transform.translation.y = 0
    tf_transform.transform.translation.z = 0
    tf_transform.transform.rotation.x = 0
    tf_transform.transform.rotation.y = 0
    tf_transform.transform.rotation.z = 0
    tf_transform.transform.rotation.w = 1

    origin_x = pose_msg.pose.position.x
    origin_y = pose_msg.pose.position.y
    origin_orientation_w = pose_msg.pose.orientation.w

    occupancy_grid.header.frame_id = "odom"
    occupancy_grid.header.stamp = rospy.Time.now()
    occupancy_grid.info.resolution = map_resolution
    occupancy_grid.info.width = int(map_width / map_resolution)
    occupancy_grid.info.height = int(map_height / map_resolution)
    occupancy_grid.info.origin.position.x = origin_x-map_width/2
    occupancy_grid.info.origin.position.y = origin_y-map_height/2
    occupancy_grid.info.origin.orientation.w = origin_orientation_w
    occupancy_grid.data = [-1] * occupancy_grid.info.width * occupancy_grid.info.height

    occupancy_threshold = 50
    threshold_free = -1
    threshold_unknown = 0

    print("Map Origin | x, y:", occupancy_grid.info.origin.position.x,
         ", ", occupancy_grid.info.origin.position.y)
    


    # convert scan data to occupancy grid data
    for i, scan_value in enumerate(data.ranges):
        if scan_value < scan_range:
            # convert angle to index in the occupancy grid array
            angle = data.angle_min + i * data.angle_increment
            x = int((scan_value * np.cos(angle) + map_width/2) / map_resolution)
            y = int((scan_value * np.sin(angle) + map_height/2) / map_resolution)
            # set the corresponding cell in the occupancy grid to occupied
            occupancy_grid.data[x + y * occupancy_grid.info.width] = 100

            # for j in range(len(occupancy_grid.data)):
            #     if occupancy_grid.data[j] >= occupancy_threshold:
            #         # set the corresponding cell in the occupancy grid to occupied
            #         occupancy_grid.data[x + y * occupancy_grid.info.width] = 100
            #         # occupancy_grid.data.append(100) # occupied
            #     elif occupancy_grid.data[i] == threshold_free:
            #         occupancy_grid.data.append(0) # free
            #     else:
            #         occupancy_grid.data.append(-1) # unknown

    # publish the occupancy grid
    occupancy_grid_pub.publish(occupancy_grid)

if __name__ == '__main__':
    rospy.init_node('occupancy_grid_node', anonymous=True)
    rospy.Subscriber('/odom', PoseStamped)
    rospy.Subscriber("/scan", LaserScan, scan_data)
    occupancy_grid_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=10)
    

    tf_br = TransformBroadcaster()
    tf_transform = TransformStamped()
    tf_transform.header.stamp = rospy.Time.now()
    tf_br.sendTransform(tf_transform)
    
    rospy.spin()    