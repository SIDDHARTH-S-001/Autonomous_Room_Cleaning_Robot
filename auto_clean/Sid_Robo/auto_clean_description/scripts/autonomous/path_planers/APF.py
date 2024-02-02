#!/usr/bin/env python3

import rospy
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion

class APFPlanner:
    def __init__(self):
        rospy.init_node('apf_planner')

        # Map information
        self.map_data = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None

        # Robot information
        self.robot_pose_x = None
        self.robot_pose_y = None
        self.robot_yaw = None

        # Goal information
        self.goal_pose_x = None
        self.goal_pose_y = None

        # Publisher for path
        self.path_pub = rospy.Publisher('/apf_path', Path, queue_size=10)

        # Subscriber for map
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Subscriber for robot pose
        rospy.Subscriber('/odom', Odometry, self.robot_pose_callback)

        # Subscriber for goal pose
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_pose_callback)

        rospy.spin()

    def map_callback(self, msg):
        # Save the map information
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

    def robot_pose_callback(self, msg):
        # Extract robot's current pose from odometry
        self.robot_pose_x = msg.pose.pose.position.x
        self.robot_pose_y = msg.pose.pose.position.y
        _, _, self.robot_yaw = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

    def goal_pose_callback(self, msg):
        # Extract goal pose
        self.goal_pose_x = msg.pose.position.x
        self.goal_pose_y = msg.pose.position.y

        # Plan path and execute
        self.plan_and_execute()

    def plan_and_execute(self):
        if self.map_data is None or self.map_resolution is None or self.map_origin_x is None or self.map_origin_y is None:
            rospy.logwarn("Map information not available. Waiting for map...")
            return

        if self.robot_pose_x is None or self.robot_pose_y is None or self.robot_yaw is None:
            rospy.logwarn("Robot pose not available. Waiting for robot pose...")
            return

        if self.goal_pose_x is None or self.goal_pose_y is None:
            rospy.logwarn("Goal pose not available. Waiting for goal pose...")
            return

        # Convert goal pose to map coordinates
        goal_map_x = int((self.goal_pose_x - self.map_origin_x) / self.map_resolution)
        goal_map_y = int((self.goal_pose_y - self.map_origin_y) / self.map_resolution)

        # Convert robot pose to map coordinates
        robot_map_x = int((self.robot_pose_x - self.map_origin_x) / self.map_resolution)
        robot_map_y = int((self.robot_pose_y - self.map_origin_y) / self.map_resolution)

        # Run APF algorithm to find path
        path = self.apf_algorithm(robot_map_x, robot_map_y, goal_map_x, goal_map_y)

        if path is None:
            rospy.logwarn("No valid path found.")
            return

        # Publish path as Path message
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for waypoint in path:
            pose = Pose()
            pose.position.x = self.map_origin_x + (waypoint[0] * self.map_resolution)
            pose.position.y = self.map_origin_y + (waypoint[1] * self.map_resolution)
            pose.position.z = 0.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            pose_stamped = PoseStamped()
            pose_stamped.pose = pose
            pose_stamped.header = path_msg.header

            path_msg.poses.append(pose_stamped)

        self.path_pub.publish(path_msg)

    def apf_algorithm(self, start_x, start_y, goal_x, goal_y):
        # APF algorithm implementation
        # Return the path as a list of waypoints [(x1, y1), (x2, y2), ...]

        # Define APF parameters
        k_att = 0.5  # Attractive potential gain
        k_rep = 10.0  # Repulsive potential gain
        eta = 2.0  # Safety distance to obstacles

        # Create a grid to store the potential field
        potential_field = np.zeros_like(self.map_data, dtype=float)

        # Calculate the attractive potential
        for i in range(self.map_data.shape[0]):
            for j in range(self.map_data.shape[1]):
                potential_field[i, j] += k_att * self.euclidean_distance((i, j), (goal_x, goal_y))

        # Calculate the repulsive potential
        for i in range(self.map_data.shape[0]):
            for j in range(self.map_data.shape[1]):
                if self.map_data[i, j] == 0:  # Free space
                    continue
                for m in range(self.map_data.shape[0]):
                    for n in range(self.map_data.shape[1]):
                        if self.map_data[m, n] == 0:  # Obstacle-free space
                            continue
                        dist = self.euclidean_distance((i, j), (m, n))
                        if dist < eta:
                            potential_field[i, j] += k_rep * (1.0 / (dist + 0.001) - 1.0 / (eta + 0.001))**2

        # Define the gradient function for potential field
        def gradient(node):
            x, y = node
            x_grad = potential_field[x + 1, y] - potential_field[x - 1, y]
            y_grad = potential_field[x, y + 1] - potential_field[x, y - 1]
            return x_grad, y_grad

        # Perform gradient descent to find the path
        path = [(start_x, start_y)]
        current_node = (start_x, start_y)
        while current_node != (goal_x, goal_y):
            x_grad, y_grad = gradient(current_node)
            next_node = (current_node[0] - int(np.sign(x_grad)), current_node[1] - int(np.sign(y_grad)))
            path.append(next_node)
            current_node = next_node

        return path

    def euclidean_distance(self, node1, node2):
        return math.sqrt((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2)


if __name__ == '__main__':
    try:
        planner = APFPlanner()
    except rospy.ROSInterruptException:
        pass
