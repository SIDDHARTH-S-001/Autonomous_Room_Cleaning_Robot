#!/usr/bin/env python3

import rospy
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import tf2_ros
from random import uniform
from scipy.spatial import KDTree

class RRTStarPlanner:
    def __init__(self):
        rospy.init_node('rrt_star_planner')

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

        # Tree parameters
        self.nodes = []
        self.edges = []
        self.max_distance = 0.5

        self.path_publisher = rospy.Publisher('/rrt_star_plan', Path, queue_size=10)

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

        # Print debug information
        rospy.loginfo("Goal pose received: x={}, y={}".format(self.goal_pose_x, self.goal_pose_y))

        # Plan path and execute
        self.plan_and_execute()

    def plan_and_execute(self):
        # Get the current robot position from the TF transform
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to get robot position from TF.")
            return

        robot_x = trans.transform.translation.x
        robot_y = trans.transform.translation.y

        # Convert the goal pose to the map frame
        goal_map = self.tf_buffer.transform(self.goal_pose, "map")

        # Get the position components of the goal in the map frame
        goal_x = goal_map.pose.position.x
        goal_y = goal_map.pose.position.y

        # Convert robot and goal positions to map indices
        robot_map_x = int((robot_x - self.map_origin_x) / self.map_resolution)
        robot_map_y = int((robot_y - self.map_origin_y) / self.map_resolution)
        goal_map_x = int((goal_x - self.map_origin_x) / self.map_resolution)
        goal_map_y = int((goal_y - self.map_origin_y) / self.map_resolution)

        # Run RRT* algorithm to find path
        path = self.rrt_star_algorithm(robot_map_x, robot_map_y, goal_map_x, goal_map_y)

        if path is None:
            rospy.logwarn("No valid path found.")
            return

        # Create a new Path message
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        # Convert the path waypoints to PoseStamped messages and add them to the Path
        for waypoint in path:
            pose = PoseStamped()
            pose.pose.position.x = self.map_origin_x + (waypoint[0] * self.map_resolution)
            pose.pose.position.y = self.map_origin_y + (waypoint[1] * self.map_resolution)
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)


    def rrt_star_algorithm(self, start_x, start_y, goal_x, goal_y):
        # RRT* algorithm implementation
        # Return the path as a list of waypoints [(x1, y1), (x2, y2), ...]

        # Define the cost of moving from one cell to another
        cost_map = np.ones_like(self.map_data, dtype=int)
        obstacle_cells = np.where(self.map_data > 0)
        cost_map[obstacle_cells] = 1000000  # Set a large integer value for obstacles

        # Initialize the nodes list with the start node
        self.nodes = [(start_x, start_y)]

        # Perform the RRT* search
        for _ in range(1000):
            # Generate a random node in the map
            random_node = (uniform(0, self.map_data.shape[0]), uniform(0, self.map_data.shape[1]))

            # Find the nearest node in the tree to the random node
            nearest_node_idx = self.nearest_node(random_node)

            # Steer towards the random node from the nearest node
            new_node = self.steer(random_node, nearest_node_idx)

            # Check if the new node is valid (not in an obstacle)
            if cost_map[new_node] != 1000000:
                # Find nearby nodes within a certain radius
                nearby_nodes = self.nearby_nodes(new_node)

                # Find the parent node that minimizes the cost to reach the new node
                min_cost_node = self.choose_parent(nearby_nodes, new_node)

                # Add the new node to the tree
                self.nodes.append(new_node)
                self.edges.append((min_cost_node, new_node))

                # Rewire the nearby nodes to update their parents if the new node is a better parent
                self.rewire(nearby_nodes, new_node)

        # Find the node in the tree that is closest to the goal
        goal_node_idx = self.nearest_node((goal_x, goal_y))
        goal_node = self.nodes[goal_node_idx]

        # Reconstruct the path from the goal node to the start node
        path = [goal_node]
        current_node = goal_node
        while current_node != (start_x, start_y):
            parent_node = self.get_parent(current_node)
            path.append(parent_node)
            current_node = parent_node
        path.reverse()

        return path

    def nearest_node(self, query_node):
        # Use KDTree to efficiently find the nearest node in the tree to the query node
        kdtree = KDTree(self.nodes)
        nearest_node_idx = kdtree.query(query_node)[1]
        return nearest_node_idx

    def steer(self, random_node, nearest_node_idx):
        # Steer towards the random node from the nearest node
        nearest_node = self.nodes[nearest_node_idx]
        direction = (random_node[0] - nearest_node[0], random_node[1] - nearest_node[1])
        direction_norm = math.sqrt(direction[0] ** 2 + direction[1] ** 2)
        steer_distance = min(self.max_distance, direction_norm)
        steer_node = (
            int(nearest_node[0] + (direction[0] / direction_norm) * steer_distance),
            int(nearest_node[1] + (direction[1] / direction_norm) * steer_distance)
        )
        return steer_node

    def nearby_nodes(self, new_node):
        # Find nearby nodes within a certain radius
        radius = 2 * math.sqrt(math.log(len(self.nodes)) / len(self.nodes))
        kdtree = KDTree(self.nodes)
        nearby_node_indices = kdtree.query_ball_point(new_node, radius)
        nearby_nodes = [self.nodes[i] for i in nearby_node_indices]
        return nearby_nodes

    def choose_parent(self, nearby_nodes, new_node):
        # Find the parent node that minimizes the cost to reach the new node
        min_cost = float('inf')
        min_cost_node = None
        for node in nearby_nodes:
            cost = self.get_cost(node) + self.cost(node, new_node)
            if cost < min_cost:
                min_cost = cost
                min_cost_node = node
        return min_cost_node

    def rewire(self, nearby_nodes, new_node):
        # Rewire the nearby nodes to update their parents if the new node is a better parent
        for node in nearby_nodes:
            new_cost = self.get_cost(new_node) + self.cost(new_node, node)
            if new_cost < self.get_cost(node):
                self.set_parent(node, new_node)

    def get_parent(self, node):
        # Get the parent node of a given node
        for edge in self.edges:
            if edge[1] == node:
                return edge[0]
        return None

    def set_parent(self, node, parent):
        # Set the parent node of a given node
        for i, edge in enumerate(self.edges):
            if edge[1] == node:
                self.edges[i] = (parent, node)

    def get_cost(self, node):
        # Get the cost to reach a given node
        for edge in self.edges:
            if edge[1] == node:
                return self.cost(edge[0], edge[1])
        return float('inf')

    def cost(self, node1, node2):
        # Check if either node1 or node2 is None
        if node1 is None or node2 is None:
            return float('inf')

        # Calculate the cost between two nodes (Euclidean distance)
        return math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)


if __name__ == '__main__':
    try:
        planner = RRTStarPlanner()
    except rospy.ROSInterruptException:
        pass
