#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist

class Turtlebot3Follower:
    def __init__(self):
        rospy.init_node('turtlebot3_follower')

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.goal_pub = rospy.Publisher('/auto_clean/move_base_simple/goal', PoseStamped, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/auto_clean/cmd_vel', Twist, queue_size=10)
        
        self.current_pose = None
        
    def odom_callback(self, odom_msg):
        # odom_msg = Odometry
        self.current_pose = odom_msg.pose.pose
        
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_pose is not None:
                goal_pose = PoseStamped()
                goal_pose.header.stamp = rospy.Time.now()
                goal_pose.header.frame_id = 'map'
                goal_pose.pose.position.x = self.current_pose.position.x 
                goal_pose.pose.position.y = self.current_pose.position.y
                goal_pose.pose.orientation = self.current_pose.orientation
                
                self.goal_pub.publish(goal_pose)
                
                # implement your robot's navigation logic here
                
                cmd_vel = Twist()
                # set linear and angular velocities based on the navigation logic
                self.cmd_vel_pub.publish(cmd_vel)
                
            rate.sleep()

if __name__ == '__main__':
    follower = Turtlebot3Follower()
    follower.run()
