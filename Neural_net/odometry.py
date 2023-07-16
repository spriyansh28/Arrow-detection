import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from odompackage.msg import EncoderTick




Left_Ticks=0
Right_Ticks=0

def tickcallback(msg):
    global Left_Ticks
    global Right_Ticks
    Left_Ticks = msg.left
    Right_Ticks = msg.right
    rospy.loginfo("I heard")

ex_Left_Ticks = 0
ex_Right_Ticks = 0




rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
tick_sub =rospy.Subscriber("TicksPublisher",EncoderTick,tickcallback)
odom_broadcaster = tf.TransformBroadcaster()




current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(10.0)




R = 0.0325        #//Wheel Radius
N = 40            #// Number of Ticks Per revolution
L = 0.125         #//Distance between left and right wheels

x = 0.0
y = 0.0
th = 0.0

vx =  0.0
vy =  0.0
vth =  0.0

dc = 0.0
dr = 0.0
dl = 0.0
dt = 0.0
dx = 0.0
dy = 0.0
dtheta = 0.0




while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    L_delta_Tick = Left_Ticks - ex_Left_Ticks
    R_delta_Tick = Right_Ticks - ex_Right_Ticks
    dt = (current_time - last_time).to_sec()
    #rospy.loginfo(dt)
    #rospy.loginfo(Right_Ticks)
    #rospy.loginfo(Left_Ticks)

    dl = 2 * pi * R * L_delta_Tick / N
    dr = 2 * pi * R * R_delta_Tick / N
    dc = (dl + dr) / 2
    dtheta = (dr - dl) / L
    if dr==dl:
        dx=dr*cos(th)
        dy=dr*sin(th)
    else:
        radius=dc/dtheta
        iccX=x-radius*sin(th)
        iccY=y+radius*cos(th)
        dx=cos(dtheta)*(x-iccX)-sin(dtheta)*(y-iccY)+iccX-x
        dy=sin(dtheta)*(x-iccX)-cos(dtheta)*(y-iccY)+iccY-y
    x += dx  # delta_x
    y += dy #delta_y
    th =(th+dtheta)% (2 * pi) #delta_th
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
       (x, y, 0.),
       odom_quat,
       current_time,
       "base_link",
       "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
    vx=dx/dt
    vy=dy/dt
    vth=dtheta/dt
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    odom_pub.publish(odom)

    ex_Left_Ticks=Left_Ticks
    ex_Right_Ticks = Right_Ticks
    last_time = current_time
    r.sleep()