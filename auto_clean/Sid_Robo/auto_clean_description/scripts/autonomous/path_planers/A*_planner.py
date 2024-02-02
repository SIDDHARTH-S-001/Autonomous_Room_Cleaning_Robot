#!/usr/bin/env python3

import rospy
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class AStarPlanner:
    def __init__(self):
        rospy.init_node('a_star_planner')

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

        # Path publisher
        self.path_publisher = rospy.Publisher('/a_star_plan', Path, queue_size=10)

        # Subscriber for map
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Subscriber for robot pose
        rospy.Subscriber('/odom', Odometry, self.robot_pose_callback)

        # Subscriber for goal pose
        rospy.Subscriber('/move_base/current_goal', PoseStamped, self.goal_pose_callback)

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

        # Run A* algorithm to find path
        path = self.a_star_algorithm(robot_map_x, robot_map_y, goal_map_x, goal_map_y)

        if path is None:
            rospy.logwarn("No valid path found.")
            return

        # Publish the path
        self.publish_path(path)

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'map'

        for waypoint in path:
            pose = PoseStamped()
            pose.pose.position.x = self.map_origin_x + (waypoint[0] * self.map_resolution)
            pose.pose.position.y = self.map_origin_y + (waypoint[1] * self.map_resolution)
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)

    def a_star_algorithm(self, start_x, start_y, goal_x, goal_y):
        # A* algorithm implementation
        # Return the path as a list of waypoints [(x1, y1), (x2, y2), ...]

        # Define the cost of moving from one cell to another
        cost_map = np.ones_like(self.map_data, dtype=int)
        obstacle_cells = np.where(self.map_data > 0)
        cost_map[obstacle_cells] = 1000000  # Set a large integer value for obstacles

        # Define the heuristic function
        def heuristic(x, y):
            return math.sqrt((x - goal_x) ** 2 + (y - goal_y) ** 2)

        # Create the open and closed sets
        open_set = {(start_x, start_y)}
        closed_set = set()

        # Initialize the g-score and f-score dictionaries
        g_score = {cell: np.inf for cell in open_set}
        g_score[(start_x, start_y)] = 0
        f_score = {cell: np.inf for cell in open_set}
        f_score[(start_x, start_y)] = heuristic(start_x, start_y)

        # Create a dictionary to store the parent of each cell
        parent = {}

        # Perform the A* search
        while open_set:
            current = min(open_set, key=lambda cell: f_score[cell])

            if current == (goal_x, goal_y):
                # Reconstruct the path
                path = [current]
                while current in parent:
                    current = parent[current]
                    path.append(current)
                path.reverse()
                return path

            open_set.remove(current)
            closed_set.add(current)

            # Generate the neighboring cells
            x, y = current
            neighbors = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]

            for neighbor in neighbors:
                if neighbor in closed_set:
                    continue

                if neighbor[0] < 0 or neighbor[0] >= self.map_data.shape[0] or \
                        neighbor[1] < 0 or neighbor[1] >= self.map_data.shape[1]:
                    continue

                if cost_map[neighbor[0], neighbor[1]] == 1000000:  # Check cost of neighbor cell
                    continue

                tentative_g_score = g_score[current] + 1

                if neighbor not in open_set or tentative_g_score < g_score[neighbor]:
                    parent[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor[0], neighbor[1])

                    if neighbor not in open_set:
                        open_set.add(neighbor)


if __name__ == '__main__':
    try:
        planner = AStarPlanner()
    except rospy.ROSInterruptException:
        pass
