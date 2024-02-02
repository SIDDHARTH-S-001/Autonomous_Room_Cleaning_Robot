import rospy
from nav_msgs.msg import Odometry
import tf
from math import sin, cos, pi
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3

rospy.init_node("Odom_Publisher")
odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
odom_broadcaster = tf.TransformBroadcaster()

current_time = rospy.Time.now()
last_time = rospy.Time.now()

x = int(input())
y = int(input())
th = int(input())

vx = int(input())
vy = int(input())
vth = int(input())

r = rospy.Rate(1.0)

while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    dt =  (current_time - last_time).to_sec()
    delta_x = (vx*cos(th) - vy*sin(th))*dt
    delta_y = (vx*sin(th) + vy*cos(th))*dt
    delta_th = vth*dt

    x = x + delta_x
    y = y + delta_y
    th = th + delta_th

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    odom_broadcaster.sendTransform((x, y, 0), odom_quat, current_time, "base_link", "odom")

    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose = Pose(Point(x, y, 0), Quaternion(*odom_quat))
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    odom_pub.publish(odom)
    last_time = current_time
    r.sleep()

