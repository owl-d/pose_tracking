#!/usr/bin/env python3
#linear drive(translation)
import rospy
import time
from geometry_msgs.msg import Twist


def time_vel(start_time):

    current_time =  rospy.Time.now()
    dt = current_time - start_time

    if dt.to_sec() < 10:
        control_linear_vel = 0
        control_angular_vel = 0

    elif dt.to_sec() >= 10 and dt.to_sec() < 40:
        control_linear_vel = 0.03
        control_angular_vel = 0

    elif dt.to_sec() >= 40 and dt.to_sec() < 100:
        control_linear_vel = -0.03
        control_angular_vel = 0

    elif dt.to_sec() >= 100 and dt.to_sec() < 130:
        control_linear_vel = 0.03
        control_angular_vel = 0

    elif dt.to_sec() >= 130 and dt.to_sec() < 135:
        control_linear_vel = 0
        control_angular_vel = 0
    
    elif dt.to_sec() >= 135 and dt.to_sec() < 145:
        control_linear_vel = 0
        control_angular_vel = 0.15708

    elif dt.to_sec() >= 145 and dt.to_sec() < 150:
        control_linear_vel = 0
        control_angular_vel = 0

    elif dt.to_sec() >= 150 and dt.to_sec() < 180:
        control_linear_vel = 0.03
        control_angular_vel = 0

    elif dt.to_sec() >= 180 and dt.to_sec() < 240:
        control_linear_vel = -0.03
        control_angular_vel = 0
        
    elif dt.to_sec() >= 240 and dt.to_sec() < 270:
        control_linear_vel = 0.03
        control_angular_vel = 0
    
    elif dt.to_sec() >= 270 and dt.to_sec() < 275:
        control_linear_vel = 0
        control_angular_vel = 0

    elif dt.to_sec() >= 275 and dt.to_sec() < 285:
        control_linear_vel = 0
        control_angular_vel = -0.15708

    else :
        control_linear_vel = 0
        control_angular_vel = 0

    return control_linear_vel, control_angular_vel



if __name__=="__main__":

    rospy.init_node('set_vel')

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=3)

    twist = Twist()

    start_time =  rospy.Time.now()
    rate = rospy.Rate(50) # 10hz

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
