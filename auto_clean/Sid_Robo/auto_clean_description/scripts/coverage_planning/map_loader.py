#!/usr/bin/env python3

import cv2
import yaml
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult

class MapLoader:
    def __init__(self, map_file, yaml_file):
        rospy.init_node('Coverage_Planner_Node')
        self.map_file = map_file
        self.yaml_file = yaml_file
        self.map_data = None
        self.map_image = None
        self.x_offset = 0.15
        self.y_offset = 0.7

    def load_map(self):
        # Load map parameters from YAML file
        with open(self.yaml_file, 'r') as file:
            self.map_data = yaml.safe_load(file)

        # Load map image using OpenCV
        self.map_image = cv2.imread(self.map_file, cv2.IMREAD_GRAYSCALE)

    def display_map_parameters(self):
        if self.map_data is not None:
            rospy.loginfo("Map Parameters:")
            for key, value in self.map_data.items():
                rospy.loginfo(f"{key}: {value}")
        else:
            rospy.loginfo("Map parameters not available. Call load_map() first.")

    def pixel_to_map_coordinates(self, x_pixel, y_pixel):
        # Convert pixel coordinates to map coordinates
        resolution = self.map_data['resolution']
        origin = self.map_data['origin']

        y_map = -(origin[0] + (y_pixel * resolution) + self.y_offset)
        x_map = origin[1] + (x_pixel * resolution) + self.x_offset

        return x_map, y_map

    def distance_to_nearest_black(self, y, x, radius=7):
        # Check distance to the nearest black pixel
        black_mask = self.map_image < 250
        y_range, x_range = self.map_image.shape

        for i in range(-radius, radius + 1):
            for j in range(-radius, radius + 1):
                if 0 <= y + i < y_range and 0 <= x + j < x_range and black_mask[y + i, x + j]:
                    return False  # Too close to a black pixel

        return True

    def display_modified_map(self):
        if self.map_image is not None:
            # Convert the grayscale image to RGB
            color_map = cv2.cvtColor(self.map_image, cv2.COLOR_GRAY2RGB)

            # Create mask for pixels below 250
            black_mask = self.map_image < 250

            # Set pixels to black based on the mask
            color_map[black_mask] = [0, 0, 0]  # Set black

            # Create mask for pixels above 250
            white_mask = self.map_image >= 250

            # Lists to store coordinates and goal indexes
            goal_positions = []
            goal_pixel_positions = []

            # Draw red points on the white part and store goal positions
            goal_index = 1
            max_index = len(range(0, self.map_image.shape[0], 10)) * len(range(0, self.map_image.shape[1], 10))
            for x in range(0, self.map_image.shape[1], 10):  # Adjust the step size as needed
                # Determine whether to iterate from top to bottom or bottom to top based on x-coordinate
                y_range = range(0, self.map_image.shape[0], 10) if x % 20 == 0 else reversed(range(0, self.map_image.shape[0], 10))
                
                for y in y_range:  # Adjust the step size as needed
                    if white_mask[y, x] and self.distance_to_nearest_black(y, x):
                        if goal_index == 1:
                            color_map[y, x] = [255, 0, 0]  # Blue for the first red dot
                        elif goal_index == max_index:
                            color_map[y, x] = [0, 255, 0]  # Green for the last red dot
                        else:
                            color_map[y, x] = [0, 0, 255]  # Red for other red dots

                        goal_pixel_positions.append(((x, y)))
                        x_map, y_map = self.pixel_to_map_coordinates(x, y)
                        goal_positions.append(((round(x_map, 2), round(y_map, 2)), goal_index))  # Store coordinates and goal index
                        goal_index += 1

            # Draw green lines between consecutive red dots
            for i in range(1, len(goal_positions)):
                cv2.line(color_map, (int(goal_pixel_positions[i-1][0]), int(goal_pixel_positions[i-1][1])),
                         (int(goal_pixel_positions[i][0]), int(goal_pixel_positions[i][1])), (0, 255, 0), 1)

            cv2.imshow('Modified Map', color_map)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            rospy.loginfo("Map image not available. Call load_map() first.")

if __name__ == "__main__":
    # Replace these paths with your actual paths
    map_file_path = '/home/siddharth/catkin_ws/src/auto_clean/Sid_Robo/auto_clean_description/world/test_map.pgm'
    yaml_file_path = '/home/siddharth/catkin_ws/src/auto_clean/Sid_Robo/auto_clean_description/world/test_map.yaml'

    # Create MapLoader instance
    map_loader = MapLoader(map_file_path, yaml_file_path)

    # Load and display map parameters
    map_loader.load_map()
    map_loader.display_map_parameters()

    # Display modified map
    map_loader.display_modified_map()
