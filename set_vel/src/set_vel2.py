#!/usr/bin/env python3
#rotation drive
import rospy
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

path = Path()

def time_vel(start_time):
    
    current_time =  rospy.Time.now()
    dt = current_time - start_time

    if dt.to_sec() < 10:
        control_linear_vel = 0
        control_angular_vel = 0

    elif dt.to_sec() >= 10 and dt.to_sec() < 80:
        control_linear_vel = 0.0448798950513  # radius*2*pi/time
        control_angular_vel = 0.0942 # 2*pi/time

    elif dt.to_sec() >= 80 and dt.to_sec() < 95:
        control_linear_vel = 0
        control_angular_vel = 0

    elif dt.to_sec() >= 95 and dt.to_sec() < 165:
        control_linear_vel = 0.0448798950513  # radius*2*pi/time
        control_angular_vel = -0.0942 # 2*pi/time

    elif dt.to_sec() >= 165 and dt.to_sec() <170:
        control_linear_vel = 0
        control_angular_vel = 0

    elif dt.to_sec() >= 170 and dt.to_sec() < 240:
        control_linear_vel = 0.0448798950513  # radius*2*pi/time
        control_angular_vel = 0.0942 # 2*pi/time

    elif dt.to_sec() >= 240 and dt.to_sec() < 245:
        control_linear_vel = 0
        control_angular_vel = 0

    elif dt.to_sec() >= 245 and dt.to_sec() < 315:
        control_linear_vel = 0.0448798950513  # radius*2*pi/time
        control_angular_vel = -0.0942 # 2*pi/time

    else :
        control_linear_vel = 0
        control_angular_vel = 0

    return control_linear_vel, control_angular_vel



def pose_cb(data):
    global path
    path.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose
    path.poses.append(pose)
    path_pub.publish(path)

if __name__=="__main__":

    rospy.init_node('set_vel')

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=3)
    path_pub = rospy.Publisher('/path', Path, queue_size=1)
    sub = rospy.Subscriber('/pose', PoseStamped, pose_cb)

    twist = Twist()
    pose = PoseStamped()
    path = Path()

    start_time =  rospy.Time.now()
    rate = rospy.Rate(20) # 10hz

    while not rospy.is_shutdown():
        linear_vel, angular_vel = time_vel(start_time)

        twist.linear.x = linear_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_vel
        pub.publish(twist)

        rate.sleep()
