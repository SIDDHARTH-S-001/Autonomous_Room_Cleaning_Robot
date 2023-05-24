#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def keyboard_control():
    pub = rospy.Publisher('/auto_clean/cmd_vel', Twist, queue_size=10)
    rospy.init_node('keyboard_teleop')
    twist = Twist()
    rate = rospy.Rate(1)
    lin_vel = 0
    ang_vel = 0
    while not rospy.is_shutdown():
        cmd = input("Enter a command (q/z or e/c and either of w/a/s/d): ")
        if cmd == 'q':
            lin_vel += 0.05
        elif cmd == 'z':
            lin_vel -= 0.05
        elif cmd == 'e':
            ang_vel += 0.05
        elif cmd == 'c':
            ang_vel -= 0.05      
        elif cmd == 'w':
            twist.linear.x = lin_vel
            print('moving forward')
        elif cmd == 'a':
            twist.angular.z = ang_vel
            print('turning left')
        elif cmd == 's':
            twist.linear.x = -lin_vel
            print('moving reverse')
        elif cmd == 'd':
            twist.angular.z = -ang_vel
            print('turning right')
        else:
            twist = Twist()
            print('Stop')
        print('lin_vel: ', lin_vel)
        print('ang_vel: ', ang_vel)        
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        keyboard_control()
    except rospy.ROSInterruptException:
        pass
