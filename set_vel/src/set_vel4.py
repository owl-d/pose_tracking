#!/usr/bin/env python3
#rotation drive
import rospy
import time
from geometry_msgs.msg import Twist


def time_vel(start_time):
    
    current_time =  rospy.Time.now()
    dt = current_time - start_time

    if dt.to_sec() < 10:
        control_linear_vel = 0
        control_angular_vel = 0

    elif dt.to_sec() >= 10 and dt.to_sec() < 27.5:
        control_linear_vel = 0.0448798950513  # radius*2*pi/time
        control_angular_vel = 0.089579790102566 # 2*pi/time
    
    elif dt.to_sec() >= 27.5 and dt.to_sec() < 32.5:
        control_linear_vel = 0
        control_angular_vel = 0 #90
    
    elif dt.to_sec() >= 32.5 and dt.to_sec() < 50:
        control_linear_vel = 0.0448798950513  # radius*2*pi/time
        control_angular_vel = 0.089579790102566 # 2*pi/time
    
    elif dt.to_sec() >= 50 and dt.to_sec() < 55:
        control_linear_vel = 0
        control_angular_vel = 0 #180
    
    elif dt.to_sec() >= 55 and dt.to_sec() < 72.5:
        control_linear_vel = 0.0448798950513  # radius*2*pi/time
        control_angular_vel = 0.089579790102566 # 2*pi/time
    
    elif dt.to_sec() >= 72.5 and dt.to_sec() < 77.5:
        control_linear_vel = 0
        control_angular_vel = 0 #270
    
    elif dt.to_sec() >= 77.5 and dt.to_sec() < 95:
        control_linear_vel = 0.0448798950513  # radius*2*pi/time
        control_angular_vel = 0.089579790102566 # 2*pi/time
    
    elif dt.to_sec() >= 95 and dt.to_sec() < 100:
        control_linear_vel = 0
        control_angular_vel = 0  #1 cycle
    
   
    elif dt.to_sec() >= 100 and dt.to_sec() < 117.5:
        control_linear_vel = 0.0448798950513  # radius*2*pi/time
        control_angular_vel = -0.089579790102566 # 2*pi/time

    elif dt.to_sec() >= 117.5 and dt.to_sec() < 122.5:
        control_linear_vel = 0
        control_angular_vel = 0 #90
    
    elif dt.to_sec() >= 122.5 and dt.to_sec() < 140:
        control_linear_vel = 0.0448798950513  # radius*2*pi/time
        control_angular_vel = -0.089579790102566 # 2*pi/time

    elif dt.to_sec() >= 140 and dt.to_sec() < 145:
        control_linear_vel = 0
        control_angular_vel = 0 #180
    
    elif dt.to_sec() >= 145 and dt.to_sec() < 162.5:
        control_linear_vel = 0.0448798950513  # radius*2*pi/time
        control_angular_vel = -0.089579790102566 # 2*pi/time

    elif dt.to_sec() >= 162.5 and dt.to_sec() < 167.5:
        control_linear_vel = 0
        control_angular_vel = 0 #270
    
    elif dt.to_sec() >= 167.5 and dt.to_sec() < 185:
        control_linear_vel = 0.0448798950513  # radius*2*pi/time
        control_angular_vel = -0.089579790102566 # 2*pi/time

    elif dt.to_sec() >= 185 and dt.to_sec() < 190:
        control_linear_vel = 0
        control_angular_vel = 0 #2 cycle
    

    elif dt.to_sec() >= 190 and dt.to_sec() < 207.5:
        control_linear_vel = 0.0448798950513  # radius*2*pi/time
        control_angular_vel = 0.089579790102566 # 2*pi/time

    elif dt.to_sec() >= 207.5 and dt.to_sec() < 212.5:
        control_linear_vel = 0
        control_angular_vel = 0 #90
    
    elif dt.to_sec() >= 212.5 and dt.to_sec() < 230:
        control_linear_vel = 0.0448798950513  # radius*2*pi/time
        control_angular_vel = 0.089579790102566 # 2*pi/time

    elif dt.to_sec() >= 230 and dt.to_sec() < 235:
        control_linear_vel = 0
        control_angular_vel = 0 #180
    
    elif dt.to_sec() >= 235 and dt.to_sec() < 252.5:
        control_linear_vel = 0.0448798950513  # radius*2*pi/time
        control_angular_vel = 0.089579790102566 # 2*pi/time

    elif dt.to_sec() >= 252.5 and dt.to_sec() < 257.5:
        control_linear_vel = 0
        control_angular_vel = 0 #270

    elif dt.to_sec() >= 257.5 and dt.to_sec() < 275:
        control_linear_vel = 0.0448798950513  # radius*2*pi/time
        control_angular_vel = 0.089579790102566 # 2*pi/time

    elif dt.to_sec() >= 275 and dt.to_sec() < 280:
        control_linear_vel = 0
        control_angular_vel = 0 #3 cycle
    

    elif dt.to_sec() >= 280 and dt.to_sec() < 297.5:
        control_linear_vel = 0.0448798950513  # radius*2*pi/time
        control_angular_vel = -0.089579790102566 # 2*pi/time

    elif dt.to_sec() >= 297.5 and dt.to_sec() < 302.5:
        control_linear_vel = 0
        control_angular_vel = 0 #90
    
    elif dt.to_sec() >= 302.5 and dt.to_sec() < 320:
        control_linear_vel = 0.0448798950513  # radius*2*pi/time
        control_angular_vel = -0.089579790102566 # 2*pi/time

    elif dt.to_sec() >= 320 and dt.to_sec() < 325:
        control_linear_vel = 0
        control_angular_vel = 0 #180
    
    elif dt.to_sec() >= 325 and dt.to_sec() < 342.5:
        control_linear_vel = 0.0448798950513  # radius*2*pi/time
        control_angular_vel = -0.089579790102566 # 2*pi/time

    elif dt.to_sec() >= 342.5 and dt.to_sec() < 347.5:
        control_linear_vel = 0
        control_angular_vel = 0 #270
    
    elif dt.to_sec() >= 347.5 and dt.to_sec() < 365:
        control_linear_vel = 0.0448798950513  # radius*2*pi/time
        control_angular_vel = -0.089579790102566 # 2*pi/time

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