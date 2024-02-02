import math
import matplotlib.pyplot as plt
from map_loader import MapLoader

class CoveragePlanner:
    def __init__(self, map_loader):
        self.map_loader = map_loader
        self.goal_positions = []

    def plan_coverage(self, grid_spacing=5):
        if self.map_loader.map_image is not None:
            # Get map dimensions
            height, width = self.map_loader.map_image.shape

            # Define robot diameter and calculate radius
            robot_diameter = 0.2  # Replace with your actual robot diameter
            robot_radius = robot_diameter / 2.0

            # Define the spiral parameters
            a = 1.0  # Spacing between spiral arms
            b = 0.1  # Rate of rotation

            # Generate goal positions in a spiral pattern
            for t in range(0, 1000):
                theta = a * t
                x = int(width / 2 + (robot_radius + grid_spacing * theta) * (1 + b * theta) * math.cos(theta))
                y = int(height / 2 + (robot_radius + grid_spacing * theta) * (1 + b * theta) * math.sin(theta))

                # Check if the goal position is within the map boundaries
                if 0 <= x < width and 0 <= y < height:
                    self.goal_positions.append((x, y))

            # Display the map and planned grid
            self.display_coverage_plan()

    def display_coverage_plan(self):
        if self.map_loader.map_image is not None and self.goal_positions:
            # Plot the map
            plt.imshow(self.map_loader.map_image, cmap='gray', origin='upper')

            # Plot the planned grid
            goals = [list(pos) for pos in zip(*self.goal_positions)]
            plt.scatter(goals[0], goals[1], c='red', marker='x', label='Planned Goals')

            plt.title('Coverage Planner')
            plt.legend()
            plt.show()
        else:
            print("Map or goal positions not available. Call plan_coverage() first.")


if __name__ == "__main__":
    # Replace these paths with your actual paths
    map_file_path = '/home/siddharth/catkin_ws/src/auto_clean/Sid_Robo/auto_clean_description/world/map.pgm'
    yaml_file_path = '/home/siddharth/catkin_ws/src/auto_clean/Sid_Robo/auto_clean_description/world/map.yaml'

    # Create MapLoader instance
    map_loader = MapLoader(map_file_path, yaml_file_path)

    # Create CoveragePlanner instance
    coverage_planner = CoveragePlanner(map_loader)

    # Load map and plan coverage
    map_loader.load_map()
    coverage_planner.plan_coverage()
