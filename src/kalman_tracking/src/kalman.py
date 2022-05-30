#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, PoseStamped, Twist, TransformStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
import numpy as np
import numpy.linalg as lin
import math
import sys
import rospy
import tf_conversions
import tf2_ros

class Algorithm(object):

    def __init__(self):
        self._pose_sub = rospy.Subscriber("/pose", PoseStamped, self.encoder_pose_callback) # pose from encoder
        self._cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist) # value scenario require
        self._lin_vel_sub = rospy.Subscriber("/lin_vel", Vector3, self.lin_vel_callback) # value scenario require
        self._yaw_sub = rospy.Subscriber("/yaw", Float64, self.encoder_yaw_callback) # yaw from encoder(degree)
        self._encoder_orientation_sub = rospy.Subscriber("/encoder_orientation", Imu, self.imu_callback) # IMU value
        self._mag_sub = rospy.Subscriber("/magnetic_field", MagneticField, self.mag_callback) # mag value
        self.pose_pub = rospy.Publisher("pose_kalman", PoseStamped, queue_size = 10)
        self.pose_xp_pub = rospy.Publisher("pose_xp_kalman", PoseStamped, queue_size = 10)
        self.velocity_pub = rospy.Publisher("velocity", Twist, queue_size = 10)

        self.br = tf2_ros.TransformBroadcaster()
        self.tf = TransformStamped()
        self._pose = PoseStamped()
        self._pose_xp = PoseStamped()
        self._velocity = Twist()
        self._gyro = Vector3()
        self._acc = Vector3()
        self._mag = Vector3()
        self._lin_vel = Vector3()
        self._encoder_x = 0
        self._encoder_y = 0
        self._encoder_yaw = 0
        # self._encoder_orientation = [0, 0, 0, 0]

        self.body_v_x = 0
        self.body_v_y = 0 #velocity
        self.local_v_x = 0
        self.local_v_y = 0
        self.local_p_x = 0
        self.local_p_y = 0 #position
        self._dt = 0.02
        self._H = np.zeros((3,4))
        self._Q = 1e-8*np.eye(4)
        self._R = 2*np.eye(3)
        self._V = np.eye(3)
        self._P = 0.125*np.eye(4)
        self._A = np.zeros((4,4))
        self._K = np.zeros((4,3))
        self._x = np.array([[1],[0],[0],[0]]) #quarternion, angular position : output
        self._q1 = np.array([[1],[0],[0],[0]])
        self._xp = np.zeros((4,1))
        self._Pp = np.zeros((4,4))
        self._z1 = np.zeros((3,1))
        self._z2 = np.zeros((3,1))
        self._h = np.zeros((3,1))

        #position var
        self.pos_x = np.zeros((3,1))
        self.pos_y = np.zeros((3,1))
        self.pos_xp = np.zeros((3,1))
        self.pos_yp = np.zeros((3,1))
        self.pos_A = np.array([[1, self._dt, self._dt*self._dt/2],[0, 1, self._dt],[0, 0, 1]])
        self._z3 = np.zeros((3,1))
        self.posx_K = np.zeros((3,3))
        self.posy_K = np.zeros((3,3))
        self.posx_P = np.zeros((3,3))
        self.posy_P = np.zeros((3,3))
        self.posx_Pp = np.zeros((3,3))
        self.posy_Pp = np.zeros((3,3))
        self.pos_R = np.eye(3)
        self.pos_V = np.eye(3)
        self.pos_H = np.eye(3)
        self.pos_Q = 1e-10*np.eye(3)

    def kalman_filter(self):
        I = np.eye(4)
        # self.normal_acc()

        #Priori System Estimate : gyro
        self.create_A()
        self._xp = self._A.dot(self._x)
        self._Pp = self._A.dot(self._P).dot(self._A.T) + self._Q

        #Correction Stage1 : with acc
        self.create_H1()
        self.GetKalmanGain()
        qe1 = self._K.dot(self._z1 - self._h)
        qe1[3][0] = 0
        self._q1 = self._xp + qe1
        temp = I - self._K.dot(self._H)
        P1 = temp.dot(self._Pp)

        #Correction Stage2 : with encoder yaw
        self.create_H2()
        self.GetKalmanGain()
        qe2 = self._K.dot(self._z2 - self._h)
        qe2[1][0] = 0
        qe2[2][0] = 0
        self._x = self._q1 + qe2
        temp = I - self._K.dot(self._H)
        self._P = temp.dot(P1)

        #output(normalization)
        Q_abs = math.sqrt(self._x[0][0]**2 + self._x[1][0]**2 + self._x[2][0]**2 + self._x[3][0]**2)
        self._x = self._x/Q_abs
        self.getposition()

        #position X Estimate
        self.pos_xp = self.pos_A.dot(self.pos_x)
        self.posx_Pp = self.pos_A.dot(self.posx_P).dot(self.pos_A.T) + self.pos_Q

        #position Y Estimate
        self.pos_yp = self.pos_A.dot(self.pos_y)
        self.posy_Pp = self.pos_A.dot(self.posy_P).dot(self.pos_A.T) + self.pos_Q

        #position X Correction
        I = np.eye(3)
        self._z3 = [[self._encoder_x], [self._lin_vel.x*math.cos(self._encoder_yaw)], [self._acc.x]]

        temp = self.pos_H.dot(self.posx_Pp).dot(self.pos_H.T) + self.pos_V.dot(self.pos_R).dot(self.pos_V.T)
        inverse_temp = lin.inv(temp)
        self.posx_K = self.posx_Pp.dot(self.pos_H.T).dot(inverse_temp)

        self.pos_x = self.pos_xp + self.posx_K.dot(self._z3)
        self.posx_P = (I - self.posx_K.dot(self.pos_H)).dot(self.posx_Pp)

        #position Y Correction
        self._z3 = [[self._encoder_y], [self._lin_vel.x*math.cos(self._encoder_yaw)], [self._acc.y]]
        
        temp = self.pos_H.dot(self.posy_Pp).dot(self.pos_H.T) + self.pos_V.dot(self.pos_R).dot(self.pos_V.T)
        inverse_temp = lin.inv(temp)
        self.posy_K = self.posy_Pp.dot(self.pos_H.T).dot(inverse_temp)

        self.pos_y = self.pos_yp + self.posy_K.dot(self._z3)
        self.posy_P = (I - self.posy_K.dot(self.pos_H)).dot(self.posy_Pp)

    def lin_vel_callback(self, msg):
        self._lin_vel.x = msg.x
        self._lin_vel.y = msg.y
        self._lin_vel.z = msg.z

    def encoder_pose_callback(self, msg):
        self._encoder_x = msg.pose.position.x
        self._encoder_y = msg.pose.position.y

    def encoder_yaw_callback(self, msg):
        self._encoder_yaw = -np.deg2rad(msg.data)
        #self._encoder_orientation[0], self._encoder_orientation[1], self._encoder_orientation[2], self._encoder_orientation[3] = quaternion_from_euler(0, 0, self._encoder_yaw)

    def normal_acc(self):
        #normalize the accel value
        self._acc.x = self._acc.x / math.sqrt(self._acc.x**2 +self._acc.y**2 + self._acc.z**2)
        self._acc.y = self._acc.y / math.sqrt(self._acc.x**2 +self._acc.y**2 + self._acc.z**2)
        self._acc.z = self._acc.z / math.sqrt(self._acc.x**2 +self._acc.y**2 + self._acc.z**2)

        # if self._acc.z >= 0:
        # self.q_acc = np.matrix([math.sqrt(0.5*(self._acc.z + 1)), -self._acc.y/(2*math.sqrt(0.5*(self._acc.z+1))), self._acc.x/(2*math.sqrt(0.5*(self._acc.z+1))), 0])
        # else :
        # self.q_acc_const = math.sqrt((1.0-self._acc.z) * 0.5)
        # self.q_acc = np.matrix([-self._acc.y/(2.0*self.q_acc_const), self.q_acc_const, 0.0, self._acc.x/(2.0*self.q_acc_const)])

    def imu_callback(self, msg):

        self._gyro.x = msg.angular_velocity.x
        self._gyro.y = msg.angular_velocity.y
        self._gyro.z = msg.angular_velocity.z

        self._acc.x = msg.linear_acceleration.x
        self._acc.y = msg.linear_acceleration.y
        self._acc.z = msg.linear_acceleration.z

        self.kalman_filter()
        self.publish()

    def mag_callback(self, msg):
        self._mag.x = msg.magnetic_field.x
        self._mag.y = msg.magnetic_field.y
        self._mag.z = msg.magnetic_field.z

    def getposition(self):
        quat = [self._x[0][0], self._x[1][0], self._x[2][0], self._x[3][0]]
        _, _, yaw = euler_from_quaternion(quat)

        local_acc_x = math.cos(yaw)*self._acc.x - math.sin(yaw)*self._acc.y
        local_acc_y = math.sin(yaw)*self._acc.x + math.cos(yaw)*self._acc.y
        self.body_v_x += self._acc.x*self._dt
        self.body_v_y += self._acc.y*self._dt #velocity
        self.local_v_x += local_acc_x*self._dt
        self.local_v_y += local_acc_y*self._dt
        # self.local_p_x += self.local_v_x*self._dt/2
        # self.local_p_y += self.local_v_y*self._dt/2 #position
        # self.local_p_x += local_acc_x*self._dt*self._dt/2
        # self.local_p_y += local_acc_y*self._dt*self._dt/2

    def create_A(self) :
        #A
        ohm = np.zeros((4,4))
        ohm[0][1] = -self._gyro.x
        ohm[0][2] = -self._gyro.y
        ohm[0][3] = -self._gyro.z
        ohm[1][0] = self._gyro.x
        ohm[1][2] = self._gyro.z
        ohm[1][3] = -self._gyro.y
        ohm[2][0] = self._gyro.y
        ohm[2][1] = -self._gyro.z
        ohm[2][3] = self._gyro.x
        ohm[3][0] = self._gyro.z
        ohm[3][1] = self._gyro.y
        ohm[3][2] = -self._gyro.x
        I = np.eye(4)
        self._A = I + self._dt/2*ohm

    def create_H1(self):
        #H_k1
        self._H[0][0] = -2*self._xp[2][0]
        self._H[0][1] = 2*self._xp[3][0]
        self._H[0][2] = -2*self._xp[0][0]
        self._H[0][3] = 2*self._xp[1][0]
        self._H[1][0] = 2*self._xp[1][0]
        self._H[1][1] = 2*self._xp[0][0]
        self._H[1][2] = 2*self._xp[3][0]
        self._H[1][3] = 2*self._xp[2][0]
        self._H[2][0] = 2*self._xp[0][0]
        self._H[2][1] = -2*self._xp[1][0]
        self._H[2][2] = -2*self._xp[2][0]
        self._H[2][3] = 2*self._xp[3][0]

        #h_1(qp)
        self._h[0][0] = 2*self._xp[1][0]*self._xp[3][0] - 2*self._xp[0][0]*self._xp[2][0]
        self._h[1][0] = 2*self._xp[0][0]*self._xp[1][0] + 2*self._xp[2][0]*self._xp[3][0]
        self._h[2][0] = self._xp[0][0]**2 - self._xp[1][0]**2 - self._xp[2][0]**2 + self._xp[3][0]**2
        self._h = 9.81 * self._h

        #z1
        self._z1 = [[self._acc.x], [self._acc.y], [self._acc.z]]

        #R1
        self._R = 0.5*np.eye(3)

    def create_H2(self):
        #H_k2
        self._H[0][0] = 2*self._xp[3][0]
        self._H[0][1] = 2*self._xp[2][0]
        self._H[0][2] = 2*self._xp[1][0]
        self._H[0][3] = 2*self._xp[0][0]
        self._H[1][0] = 2*self._xp[0][0]
        self._H[1][1] = -2*self._xp[1][0]
        self._H[1][2] = -2*self._xp[2][0]
        self._H[1][3] = -2*self._xp[3][0]
        self._H[2][0] = -2*self._xp[1][0]
        self._H[2][1] = -2*self._xp[0][0]
        self._H[2][2] = 2*self._xp[3][0]
        self._H[2][3] = 2*self._xp[2][0]

        #h_2(qp)
        self._h[0][0] = 2*self._xp[1][0]*self._xp[2][0] + 2*self._xp[0][0]*self._xp[3][0]
        self._h[1][0] = self._xp[0][0]**2 - self._xp[1][0]**2 - self._xp[2][0]**2 - self._xp[3][0]**2
        self._h[2][0] = 2*self._xp[2][0]*self._xp[3][0] - 2*self._xp[0][0]*self._xp[1][0]

        #z2
        self._z2 = [[0], [0], [self._encoder_yaw]]
        # self._z2 = [[self._mag.x], [self._mag.y], [self._mag.z]]


        #R2
        self._R = np.eye(3)

    def GetKalmanGain(self) :
        temp = self._H.dot(self._Pp).dot(self._H.T) + self._V.dot(self._R).dot(self._V.T)
        inverse_temp = lin.inv(temp)
        self._K = self._Pp.dot(self._H.T).dot(inverse_temp)

    def publish(self):
        self._pose.pose.position.x = self._encoder_x #self.pos_x[0][0] #self._encoder_x
        self._pose.pose.position.y = self._encoder_y #self.pos_y[0][0] #self._encoder_y
        self._pose.pose.position.z = 0
        self._pose.pose.orientation.x = self._x[1][0]
        self._pose.pose.orientation.y = self._x[2][0]
        self._pose.pose.orientation.z = self._x[3][0]
        self._pose.pose.orientation.w = self._x[0][0]
        self._pose.header.stamp = rospy.Time.now()
        self._pose.header.frame_id = "map"
        self.pose_pub.publish(self._pose)

        self._pose_xp.pose.position.x = self._encoder_x
        self._pose_xp.pose.position.y = self._encoder_y
        self._pose_xp.pose.position.z = 0
        self._pose_xp.pose.orientation.x = self._xp[1][0]
        self._pose_xp.pose.orientation.y = self._xp[2][0]
        self._pose_xp.pose.orientation.z = self._xp[3][0]
        self._pose_xp.pose.orientation.w = self._xp[0][0]
        self._pose_xp.header.stamp = rospy.Time.now()
        self._pose_xp.header.frame_id = "map"
        self.pose_xp_pub.publish(self._pose_xp)

        self.tf.header.stamp = rospy.Time.now()
        self.tf.header.frame_id = "map"
        self.tf.child_frame_id = "box"
        self.tf.transform.rotation.x = self._pose.pose.orientation.x
        self.tf.transform.rotation.y = self._pose.pose.orientation.y
        self.tf.transform.rotation.z = self._pose.pose.orientation.z
        self.tf.transform.rotation.w = self._pose.pose.orientation.w
        self.tf.transform.translation.x = self._pose.pose.position.x
        self.tf.transform.translation.y = self._pose.pose.position.y
        self.tf.transform.translation.z = self._pose.pose.position.z
        self.br.sendTransform(self.tf)

        self._velocity.linear.x = self.body_v_x
        self._velocity.linear.y = self.body_v_y
        self._velocity.linear.z = 0
        self._velocity.angular.x = self._gyro.x
        self._velocity.angular.y = self._gyro.y
        self._velocity.angular.z = self._gyro.z
        self.velocity_pub.publish(self._velocity)

if __name__ == '__main__' :
    rospy.init_node("kalman_IMU")
    Algorithm()
    rospy.spin()