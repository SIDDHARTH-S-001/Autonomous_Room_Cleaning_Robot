#!/usr/bin/env python3

import rospy
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

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

        # Subscriber for map
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Subscriber for robot pose
        rospy.Subscriber('/odom', Odometry, self.robot_pose_callback)

        # Subscriber for goal pose
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_pose_callback)

        # Publisher for path
        self.path_publisher = rospy.Publisher('/rrt_star_path', Path, queue_size=1)

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

        # Run RRT* algorithm to find path
        path = self.rrt_star_algorithm(robot_map_x, robot_map_y, goal_map_x, goal_map_y)

        if path is None:
            rospy.logwarn("No valid path found.")
            return

        # Publish the path
        self.publish_path(path)

    def rrt_star_algorithm(self, start_x, start_y, goal_x, goal_y):
        # RRT* algorithm implementation
        # Return the path as a list of waypoints [(x1, y1), (x2, y2), ...]

        # Define the step size for exploring new nodes
        step_size = 5

        # Define the search radius for nearby nodes
        search_radius = 15

        # Define the maximum number of iterations
        max_iterations = 5000

        # Initialize the tree with the start node
        tree = { (start_x, start_y): None }

        # Perform the RRT* search
        for _ in range(max_iterations):
            random_point = self.generate_random_point()
            nearest_node = self.find_nearest_node(tree, random_point)
            new_node = self.steer(nearest_node, random_point, step_size)

            if self.is_valid_node(new_node):
                nearby_nodes = self.find_nearby_nodes(tree, new_node, search_radius)
                min_cost_node = self.find_min_cost_node(nearby_nodes, nearest_node, new_node)
                tree[new_node] = min_cost_node
                self.rewire_nearby_nodes(tree, nearby_nodes, new_node)

                if self.is_goal_reachable(new_node, (goal_x, goal_y)):
                    return self.build_path(tree, (start_x, start_y), new_node)

        return None

    def generate_random_point(self):
        # Generate a random point within the map bounds
        x = np.random.randint(0, self.map_data.shape[0])
        y = np.random.randint(0, self.map_data.shape[1])
        return (x, y)

    def find_nearest_node(self, tree, point):
        # Find the nearest node in the tree to the given point
        min_distance = float('inf')
        nearest_node = None

        for node in tree:
            distance = self.euclidean_distance(node, point)
            if distance < min_distance:
                min_distance = distance
                nearest_node = node

        return nearest_node

    def steer(self, nearest_node, random_point, step_size):
        # Steer towards the random point from the nearest node within the step size
        dist = self.euclidean_distance(nearest_node, random_point)

        if dist < step_size:
            return random_point
        elif dist > 0:  # Add this check to handle the case when dist is 0
            unit_vector = (
                (random_point[0] - nearest_node[0]) / dist,
                (random_point[1] - nearest_node[1]) / dist
            )
            return (
                int(nearest_node[0] + step_size * unit_vector[0]),
                int(nearest_node[1] + step_size * unit_vector[1])
            )
        else:
            return nearest_node  # Return nearest_node when dist is 0

    def is_valid_node(self, node):
        # Check if the node is within the map bounds and not in collision
        if node is None:  # Add this check to handle None nodes
            return False
        if node[0] < 0 or node[0] >= self.map_data.shape[0] or node[1] < 0 or node[1] >= self.map_data.shape[1]:
            return False
        if self.map_data[node[0], node[1]] > 0:
            return False
        return True

    def find_nearby_nodes(self, tree, node, search_radius):
        # Find the nearby nodes within the search radius
        nearby_nodes = []

        for other_node in tree:
            if self.euclidean_distance(node, other_node) <= search_radius:
                nearby_nodes.append(other_node)

        return nearby_nodes

    def find_min_cost_node(self, nearby_nodes, nearest_node, new_node):
        # Find the nearby node with the minimum cost to reach from the nearest node via the new node
        min_cost = float('inf')
        min_cost_node = None

        for node in nearby_nodes:
            cost = self.cost(node, nearest_node) + self.euclidean_distance(node, new_node)
            if cost < min_cost:
                min_cost = cost
                min_cost_node = node

        return min_cost_node

    def rewire_nearby_nodes(self, tree, nearby_nodes, new_node):
        for node in nearby_nodes:
            new_cost = self.cost(new_node, node) + self.cost(tree[node], node)
            if new_cost < self.cost(tree[node], node):
                tree[node] = new_node

    def is_goal_reachable(self, node, goal_node):
        # Check if the goal is reachable from the given node
        distance = self.euclidean_distance(node, goal_node)
        return distance <= 5

    def build_path(self, tree, start_node, goal_node):
        # Build the path from the goal node back to the start node
        path = [goal_node]
        current_node = goal_node

        while current_node != start_node:
            current_node = tree[current_node]
            path.append(current_node)

        path.reverse()
        return path

    def cost(self, node1, node2):
        if node1 is None or node2 is None:
            return float('inf')  # Return a large value when either node is None
        return self.euclidean_distance(node1, node2)

    def euclidean_distance(self, node1, node2):
        # Compute the Euclidean distance between two nodes
        return math.sqrt((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2)

    def publish_path(self, path):
        # Create a Path message and publish it
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'map'

        for node in path:
            pose = PoseStamped()
            pose.pose.position.x = self.map_origin_x + (node[0] * self.map_resolution)
            pose.pose.position.y = self.map_origin_y + (node[1] * self.map_resolution)
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        rospy.loginfo("Path published.")

if __name__ == '__main__':
    try:
        planner = RRTStarPlanner()
    except rospy.ROSInterruptException:
        pass
