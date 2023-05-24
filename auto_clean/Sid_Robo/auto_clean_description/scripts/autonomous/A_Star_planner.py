#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

def a_star_planner(start, goal):
    try:
        get_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        resp = get_plan(start, goal, 0)
        if resp.plan.poses:
            return resp.plan
        else:
            return None
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return None

def goal_callback(msg):
    global goal
    goal = msg

def main():
    rospy.init_node('a_star_planner', anonymous=True)
    start = PoseStamped()
    odom = Odometry()
    start.header.frame_id = "map"
    start.pose.position.x = odom.pose.pose.position.x
    start.pose.position.y = odom.pose.pose.position.y
    start.pose.orientation.w = odom.pose.pose.orientation.w
    goal = None
    goal_sub = rospy.Subscriber("/move_base/current_goal", PoseStamped, goal_callback)
    # goal = Marker.
    while goal is None:
        rospy.loginfo("Waiting for goal from RViz...")
        rospy.sleep(0.5)
    plan = a_star_planner(start, goal)
    if plan:
        rospy.loginfo("Found a plan")
        for pose in plan.poses:
            rospy.loginfo("x: %f, y: %f", pose.pose.position.x, pose.pose.position.y)
    else:
        rospy.logwarn("No plan found")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
