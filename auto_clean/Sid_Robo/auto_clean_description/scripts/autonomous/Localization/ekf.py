#!/usr/bin/env python3

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64MultiArray

class EKFLocalization:
    def __init__(self):
        rospy.init_node('ekf_localization')

        # Robot pose estimation
        self.robot_pose_x = None
        self.robot_pose_y = None
        self.robot_yaw = None

        # Covariance matrices
        self.pose_covariance = np.eye(3)
        self.measurement_covariance = None  # Will be initialized in laser_scan_callback

        # Kalman filter variables
        self.state_estimate = np.zeros(3)
        self.covariance_estimate = np.eye(3)

        # Number of laser scan beams (initialize as 0, will be updated in laser_scan_callback)
        self.num_beams = 0

        # Subscriber for laser scan data
        rospy.Subscriber('/scan', LaserScan, self.laser_scan_callback)

        # Subscriber for odometry
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)

        # Publisher for estimated pose
        self.estimated_pose_pub = rospy.Publisher('/estimated_pose', PoseStamped, queue_size=10)

        rospy.spin()

    def laser_scan_callback(self, msg):
        # Perform localization using EKF

        # Extract laser scan data
        ranges = msg.ranges
        self.num_beams = len(ranges)

        # Initialize measurement_covariance based on num_beams
        if self.measurement_covariance is None:
            self.measurement_covariance = np.eye(self.num_beams)

        # Perform EKF prediction step
        self.ekf_prediction_step()

        # Perform EKF update step using laser scan data
        self.ekf_update_step(ranges)

        # Publish estimated pose
        self.publish_estimated_pose()

    def odometry_callback(self, msg):
        # Extract robot's current pose from odometry
        self.robot_pose_x = msg.pose.pose.position.x
        self.robot_pose_y = msg.pose.pose.position.y
        _, _, self.robot_yaw = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

    def ekf_prediction_step(self):
        # Perform EKF prediction step

        # Commanded velocity input
        linear_velocity = 0.1  # Adjust this value according to your robot
        angular_velocity = 0.0  # Adjust this value according to your robot

        # Compute Jacobian matrix
        jacobian = np.eye(3)
        jacobian[0, 2] = -linear_velocity * math.sin(self.robot_yaw)
        jacobian[1, 2] = linear_velocity * math.cos(self.robot_yaw)

        # Perform state prediction
        self.state_estimate += np.array([
            linear_velocity * math.cos(self.state_estimate[2]),
            linear_velocity * math.sin(self.state_estimate[2]),
            angular_velocity
        ])

        # Perform covariance prediction
        self.covariance_estimate = np.dot(np.dot(jacobian, self.covariance_estimate), jacobian.T) + self.pose_covariance

    def ekf_update_step(self, ranges):
        # Perform EKF update step using laser scan data

        # Perform measurement prediction
        predicted_ranges = self.compute_predicted_ranges()

        # Compute Jacobian matrix
        jacobian = self.compute_jacobian()

        # Compute Kalman gain
        kalman_gain = np.dot(np.dot(self.covariance_estimate, jacobian.T),
                            np.linalg.inv(np.dot(np.dot(jacobian, self.covariance_estimate), jacobian.T) +
                                        self.measurement_covariance))

        # Reshape matrices to align dimensions
        predicted_ranges = np.array(predicted_ranges).reshape(-1, 1)
        ranges = np.array(ranges).reshape(-1, 1)

        # Compute measurement residual
        measurement_residual = ranges - predicted_ranges

        # Update state estimate
        self.state_estimate += np.dot(kalman_gain, measurement_residual)

        # Update covariance estimate
        self.covariance_estimate = np.dot((np.eye(3) - np.dot(kalman_gain, jacobian)), self.covariance_estimate)


    # Rest of the code remains the same
    # ...

    def compute_predicted_ranges(self):
        # Compute predicted ranges using current state estimate

        # Example: Compute predicted ranges using simple model
        num_beams = 360  # Number of laser scan beams
        beam_angle_increment = 2 * math.pi / num_beams  # Angle increment between each beam
        predicted_ranges = []
        for beam_index in range(num_beams):
            beam_angle = self.state_estimate[2] + beam_index * beam_angle_increment
            predicted_range = 0.0  # TODO: Replace with your own prediction model
            predicted_ranges.append(predicted_range)

        return predicted_ranges

    def compute_jacobian(self, num_beams, num_dimensions):
        # Compute Jacobian matrix for the measurement prediction function

        # Example: Compute Jacobian matrix using simple model
        jacobian = np.zeros((num_beams * num_dimensions, len(self.state_estimate)))
        jacobian[0, 0] = 1.0
        jacobian[1, 1] = 1.0
        jacobian[2, 2] = 1.0

        return jacobian

    def publish_estimated_pose(self):
        # Publish estimated pose as PoseStamped

        estimated_pose_msg = PoseStamped()
        estimated_pose_msg.header.stamp = rospy.Time.now()
        estimated_pose_msg.header.frame_id = "odom"
        estimated_pose_msg.pose.position.x = self.state_estimate[0]
        estimated_pose_msg.pose.position.y = self.state_estimate[1]
        estimated_pose_msg.pose.orientation.z = math.sin(self.state_estimate[2] / 2.0)
        estimated_pose_msg.pose.orientation.w = math.cos(self.state_estimate[2] / 2.0)

        self.estimated_pose_pub.publish(estimated_pose_msg)


if __name__ == '__main__':
    try:
        localization = EKFLocalization()
    except rospy.ROSInterruptException:
        pass
