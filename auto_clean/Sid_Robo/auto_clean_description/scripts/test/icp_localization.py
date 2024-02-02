#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from scipy.spatial import KDTree

def update_odom(odom_data):
    global current_odom
    current_odom = odom_data

def compute_odom_delta(prev_odom, current_odom):
    delta_x = prev_odom.pose.pose.position.x - current_odom.pose.pose.position.x
    delta_y = prev_odom.pose.pose.position.y - current_odom.pose.pose.position.y
    delta_z = prev_odom.pose.pose.position.z - current_odom.pose.pose.position.z
    delta_theta = compute_yaw_delta(prev_odom, current_odom)

    return delta_x, delta_y, delta_z, delta_theta

def compute_yaw_delta(prev_odom, current_odom):
    prev_orientation = prev_odom.pose.pose.orientation
    curent_orientation = current_odom.pose.pose.orientation

    prev_euler = euler_from_quaternion([prev_orientation.x, prev_orientation.y, prev_orientation.z, prev_orientation.w])
    curent_euler = euler_from_quaternion([curent_orientation.x, curent_orientation.y, curent_orientation.z, curent_orientation.w])

    delta_yaw = curent_euler[2] - prev_euler[2] 
    return delta_yaw

def extract_points(scan):
    angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
    ranges = np.array(scan.ranges)

    points = []
    for i in range(len(ranges)):
        if ranges[i] < scan.range_max:
            x = ranges[i] * np.cos( angles[i])
            y = ranges[i] * np.sin( angles[i])
            points.append([x, y])
    # point_array = np.array(points)
    # print(point_array.shape) # (720, 2) Shape
    return np.array(points)

def transform_points(points, transformation): # transformations here is a list of 4 values x, y, z and theta.
    # Trans is a 4 valued tuple, translation is a 3 valued tuple, rotation is a single valued 

    translation = transformation[:3] 
    rotation = transformation[3]
    # print("Transform shape: ", transformation.shape, "trans shape: ", translation.shape, "rot shape: ", rotation.shape)    
    # print("Transform type: ", type(transformation), "trans type: ", type(translation), "rot type: ", type(rotation))  
    # print("Transform type: ", len(transformation), "trans type: ", len(translation), "rot type: ", rotation)     
    
    rotation_matrix = np.array([[np.cos(rotation), -np.sin(rotation)],
                                [np.sin(rotation), np.cos(rotation)]])
    
    print(rotation_matrix.shape)
    
    # Dot Prodict
    # transformed_points = np.dot(points, rotation_matrix.T) + translation[:2] # matrix.T represents its transpose.
    transformed_points = np.dot(points, rotation_matrix.T) + np.tile(translation[:2], (points.shape[0], 1))

    return transformed_points

def compute_rigid_transformation(points_1, points_2):
    # Compute the rigid transformation (translation and rotation) between two sets of points
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

    transformation = np.zeros(4)
    transformation[:2] = t
    transformation[3] = np.arctan2(R[1, 0], R[0, 0])

    return transformation


def combine_transformations(transformation_1, transformation_2):
    # Combine two rigid transformations
    translation_1 = transformation_1[:3]
    rotation_1 = transformation_1[3]
    translation_2 = transformation_2[:3]
    rotation_2 = transformation_2[3]

    rotation_matrix_1 = np.array([[np.cos(rotation_1), -np.sin(rotation_1)],
                                 [np.sin(rotation_1), np.cos(rotation_1)]])
    rotation_matrix_2 = np.array([[np.cos(rotation_2), -np.sin(rotation_2)],
                                 [np.sin(rotation_2), np.cos(rotation_2)]])

    combined_translation = np.dot(rotation_matrix_1.T, translation_2) + translation_1
    combined_rotation = rotation_1 + rotation_2

    combined_transformation = np.zeros(4)
    combined_transformation[:3] = combined_translation
    combined_transformation[3] = combined_rotation

    return combined_transformation


def compute_mean_squared_error(points_1, points_2):
    # Compute the mean squared error between two sets of points
    squared_errors = np.sum((points_1 - points_2) ** 2, axis=1)
    mean_squared_error = np.mean(squared_errors)

    return mean_squared_error

def perform_icp(points_1, points_2):
    # Perform the Iterative Closest Point (ICP) algorithm to align two sets of points
    max_iterations = 100
    convergence_threshold = 0.001
    transformation = np.zeros(4)

    for iteration in range(max_iterations):
        # Find the nearest neighbors between points1 and points2
        tree = KDTree(points_1)
        _, indices = tree.query(points_2)

        # Compute the transformation between matched points
        matched_points_1 = points_1[indices]
        matched_points_2 = points_2
        delta_transformation = compute_rigid_transformation(matched_points_1, matched_points_2)

        # Update the overall transformation
        transformation = combine_transformations(delta_transformation, transformation)

        # Apply the transformation to points2
        transformed_points_2 = transform_points(points_2, delta_transformation)

        # Check convergence
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

    return aligned_points_2

def icp(scan_data):
    global prev_scan, prev_odom

    if (prev_scan is None or prev_odom is None):
        prev_scan = scan_data
        prev_odom = current_odom
        return
    
    delta_odom = compute_odom_delta(prev_odom, current_odom)
    transformed_scan = icp_matching(prev_scan, scan_data, delta_odom)

    prev_scan = scan_data
    prev_odom = current_odom

    transformed_scan_pub.publish(transformed_scan)

    # More on ICP

if __name__ == "__main__":
    rospy.init_node("icp_localization")
    global prev_scan, prev_odom, current_odom
    prev_scan = None
    prev_odom = None

    rospy.Subscriber("/scan", LaserScan, icp)
    rospy.Subscriber("/odom", Odometry, update_odom)

    transformed_scan_pub = rospy.Publisher("/transformed_scan", LaserScan, queue_size=10)
    

    rospy.spin()
    
    


