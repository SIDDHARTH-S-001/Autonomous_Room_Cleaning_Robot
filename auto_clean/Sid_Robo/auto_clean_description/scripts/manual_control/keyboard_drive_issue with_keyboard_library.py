#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import keyboard
import time

def key_drive():
    # print('twist_msg')
    print('Press any Key:')
    print('w is forward')
    print('a is left')
    print('d is right')
    print('x is reverse')     

    print('use arrow keys to increase or decrease magnitude')

    rospy.init_node('keyboard_drive')
    # cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, key_drive, queue_size=1)
    pub = rospy.Publisher('key_drive', Twist, '/cmd_vel', queue_size=1)

    t = Twist()

    x_vel = t.linear.x
    y_vel = t.linear.y
    z_vel = t.linear.z

    x_ang = t.angular.x
    y_ang = t.angular.y
    z_ang = t.angular.z

    lin_vel = 0
    ang_vel = 0

    while not rospy.is_shutdown():
        x_vel = 0
        y_vel = 0
        z_vel = 0
        x_ang = 0
        x_ang = 0
        y_ang = 0     
        z_ang = 0

        if (lin_vel > 2.5):
            lin_vel = 2.5

        if (ang_vel > 2.5):
            ang_vel = 2.5

        if (lin_vel < 0):
            lin_vel = 0

        if (ang_vel < 0):
            ang_vel = 0


        if keyboard.is_pressed('up'):
            lin_vel = lin_vel + 0.25
        
        if keyboard.is_pressed('down'):
            lin_vel = lin_vel - 0.25

        if keyboard.is_pressed('left'):
            lin_vel = ang_vel + 0.125

        if keyboard.is_pressed('right'):
            lin_vel = ang_vel - 0.125

        if keyboard.is_pressed('w'):
            x_vel = lin_vel

        if keyboard.is_pressed('s'):
            x_vel = 0
            z_ang = 0
        
        if keyboard.is_pressed('a'):
            z_ang = ang_vel
            # print('a')

        if keyboard.is_pressed('d'):
            z_ang = 0
            time.sleep(0.25)
            z_ang = -ang_vel

        if keyboard.is_pressed('x'):
            # print('a')
            x_vel = 0
            time.sleep(0.25)
            x_vel = -lin_vel

        pub.publish(t)
        rospy.spin()

    
if __name__ == '__main__':
    try:
        key_drive()
    except rospy.ROSInterruptException:
        pass


    
