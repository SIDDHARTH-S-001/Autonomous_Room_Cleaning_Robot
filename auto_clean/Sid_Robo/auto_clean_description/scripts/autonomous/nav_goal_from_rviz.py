# Import required libraries
import rospy
import tf
from geometry_msgs.msg import PoseStamped

# Define a callback function to receive nav goal data
def nav_goal_callback(data):
    # Extract goal position
    data = PoseStamped()
    goal_position = data.pose.position
    goal_orientation = data.pose.orientation

    # Print the received goal position
    position = (goal_position.x, goal_position.y, goal_position.z)
    orientation = (goal_orientation.x, goal_orientation.y, goal_orientation.z, goal_orientation.w)
    print("Received nav goal position:", position)
    print("Received nav goal orientation:", orientation)
    # print(type(goal_orientation))
# Initialize the node
rospy.init_node("nav_goal_listener")

# Subscribe to the /move_base_simple/goal topic to receive nav goal data
rospy.Subscriber("/move_base_simple/goal", PoseStamped, nav_goal_callback)

# Spin the node to keep it running
rospy.spin()
