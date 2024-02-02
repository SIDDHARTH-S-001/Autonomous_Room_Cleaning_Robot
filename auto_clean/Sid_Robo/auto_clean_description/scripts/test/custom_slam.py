#!/usr/bin/env python3

## This code implements ICP-Point to Point and OGM creation from Lidar data.

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped
import numpy as np
from scipy.spatial import KDTree

robot_x = 0.0
robot_y = 0.0
current_odom = None

# Lidar Specifications
scan_min_range = 0.05
scan_max_range = 8.00

# Map Specifications
map_width = 20
map_height = 20
map_resolution = 0.05

def update_odom(odom_data):
    global current_odom, robot_x, robot_y
    current_odom = odom_data
    robot_x = current_odom.pose.pose.position.x
    robot_y = current_odom.pose.pose.position.y


def compute_odom_delta(prev_odom, current_odom):
    delta_x = prev_odom.pose.pose.position.x - current_odom.pose.pose.position.x
    delta_y = prev_odom.pose.pose.position.y - current_odom.pose.pose.position.y
    delta_z = prev_odom.pose.pose.position.z - current_odom.pose.pose.position.z
    delta_theta = compute_yaw_delta(prev_odom, current_odom)

    return delta_x, delta_y, delta_z, delta_theta

def compute_yaw_delta(prev_odom, current_odom):
    prev_orientation = prev_odom.pose.pose.orientation
    current_orientation = current_odom.pose.pose.orientation

    prev_euler = euler_from_quaternion([prev_orientation.x, prev_orientation.y, prev_orientation.z, prev_orientation.w])
    current_euler = euler_from_quaternion([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])

    delta_yaw = current_euler[2] - prev_euler[2] 
    return delta_yaw

def extract_points(scan):
    angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
    ranges = np.array(scan.ranges)

    points = []
    for i in range(len(ranges)):
        if ranges[i] < scan.range_max:
            x = ranges[i] * np.cos(angles[i])
            y = ranges[i] * np.sin(angles[i])
            points.append([x, y])

    return np.array(points)

def transform_points(points, transformation):
    translation = transformation[:2]
    rotation = transformation[2]

    rotation_matrix = np.array([[np.cos(rotation), -np.sin(rotation)],
                                [np.sin(rotation), np.cos(rotation)]])

    transformed_points = np.dot(points, rotation_matrix.T) + np.tile(translation, (points.shape[0], 1))

    return transformed_points

def compute_rigid_transformation(points_1, points_2):
    centroid_1 = np.mean(points_1, axis=0)
    centroid_2 = np.mean(points_2, axis=0)

    centered_points_1 = points_1 - centroid_1
    centered_points_2 = points_2 - centroid_2

    covariance_matrix = np.dot(centered_points_1.T, centered_points_2)

    U, _, Vt = np.linalg.svd(covariance_matrix)
    R = np.dot(Vt.T, U.T)

    if np.linalg.det(R) < 0:
        R[:, -1] *= -1

    t = centroid_1 - np.dot(R, centroid_2)

    transformation = np.zeros(3)
    transformation[:2] = t
    transformation[2] = np.arctan2(R[1, 0], R[0, 0])

    return transformation

def combine_transformations(transformation_1, transformation_2):
    translation_1 = transformation_1[:2]
    rotation_1 = transformation_1[2]
    translation_2 = transformation_2[:2]
    rotation_2 = transformation_2[2]

    rotation_matrix_1 = np.array([[np.cos(rotation_1), -np.sin(rotation_1)],
                                 [np.sin(rotation_1), np.cos(rotation_1)]])
    rotation_matrix_2 = np.array([[np.cos(rotation_2), -np.sin(rotation_2)],
                                 [np.sin(rotation_2), np.cos(rotation_2)]])

    combined_translation = np.dot(rotation_matrix_1.T, translation_2) + translation_1
    combined_rotation = rotation_1 + rotation_2

    combined_transformation = np.zeros(3)
    combined_transformation[:2] = combined_translation
    combined_transformation[2] = combined_rotation

    return combined_transformation

def compute_mean_squared_error(points_1, points_2):
    squared_errors = np.sum((points_1 - points_2) ** 2, axis=1)
    mean_squared_error = np.mean(squared_errors)

    return mean_squared_error

def perform_icp(points_1, points_2):
    max_iterations = 100
    convergence_threshold = 0.001
    transformation = np.zeros(3)

    for iteration in range(max_iterations):
        tree = KDTree(points_1)
        _, indices = tree.query(points_2)

        matched_points_1 = points_1[indices]
        matched_points_2 = points_2
        delta_transformation = compute_rigid_transformation(matched_points_1, matched_points_2)

        transformation = combine_transformations(delta_transformation, transformation)

        transformed_points_2 = transform_points(points_2, delta_transformation)

        mean_squared_error = compute_mean_squared_error(points_1, transformed_points_2)
        if mean_squared_error < convergence_threshold:
            break

        points2 = transformed_points_2

    return transformed_points_2

def icp_matching(scan_1, scan_2, delta_odom):
    points_1 = extract_points(scan_1)
    points_2 = extract_points(scan_2)

    transformed_points_2 = transform_points(points_2, delta_odom)
    transformed_points_2_aligned = perform_icp(points_1, transformed_points_2)
    aligned_points_2 = transform_points(transformed_points_2_aligned, (-delta_odom[0], -delta_odom[1], -delta_odom[2], -delta_odom[3]))

    transformed_scan = LaserScan()
    transformed_scan.header = scan_2.header
    transformed_scan.angle_min = scan_2.angle_min
    transformed_scan.angle_max = scan_2.angle_max
    transformed_scan.angle_increment = scan_2.angle_increment
    transformed_scan.time_increment = scan_2.time_increment
    transformed_scan.scan_time = scan_2.scan_time
    transformed_scan.range_min = scan_2.range_min
    transformed_scan.range_max = scan_2.range_max
    transformed_scan.ranges = aligned_points_2[:, 0].tolist()  # Convert to list
    transformed_scan.intensities = aligned_points_2[:, 1].tolist()  # Convert to list

    return transformed_scan

def icp(scan_data):
    global prev_scan, prev_odom

    if prev_scan is None or prev_odom is None:
        prev_scan = scan_data
        prev_odom = current_odom
        return

    delta_odom = compute_odom_delta(prev_odom, current_odom)
    transformed_scan = icp_matching(prev_scan, scan_data, delta_odom)

    prev_scan = scan_data
    prev_odom = current_odom

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

    scan_ranges_data = scan_data.ranges
    min_angle  = scan_data.angle_min
    angle_inc = scan_data.angle_increment

    for i, scan_value in enumerate(scan_ranges_data):
        if ((scan_value > scan_min_range) and (scan_value < scan_max_range)):
            angle = min_angle + (i * angle_inc)

            x = int((scan_value*np.cos(angle) + map_width/2 - origin_x)/map_resolution)
            y = int((scan_value*np.sin(angle) + map_height/2 - origin_y)/map_resolution)

            ogm.data[x + (y * ogm.info.width)] = 100
    
    ogm_pub.publish(ogm)
    transformed_scan_pub.publish(transformed_scan)


if __name__ == "__main__":
    rospy.init_node("icp_localization")
    global prev_scan, prev_odom
    prev_scan = None
    prev_odom = None
    
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    ogm_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=100)

    rospy.Subscriber("/scan", LaserScan, icp)
    rospy.Subscriber("/odom", Odometry, update_odom)

    transformed_scan_pub = rospy.Publisher("/transformed_scan", LaserScan, queue_size=10)

    rospy.spin()
