#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 02:02:03 2019
Added manipulator on Tue Feb 14 01:20:03 2023
@author: mason
"""

''' import libraries '''
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos, sin
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy
from mavros_msgs.srv import SetMode, CommandBool

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def rpy_saturation(angle):
    if angle>np.pi:
        angle=angle-2*np.pi
    if angle<-np.pi:
        angle=angle+2*np.pi
    return angle

global joy_check
joy_check=0
global mav_check
mav_check=0

global d2r
global r2d
global max_rate_x
global max_rate_y
global max_vel_x
global max_vel_y
global max_vel_z
global yaw_rate
yaw_rate = 2

r2d = 180/np.pi
d2r = np.pi/180
max_rate_x = 360 * d2r
max_rate_y = 360 * d2r
max_vel_x = 10
max_vel_y = 10
max_vel_z = 4

def even_odd_to_sign(val):
    if val%2==0:
        return 1
    else:
        return -1

''' class '''
class robot():
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.model_name = rospy.get_param("/model_name", 'typhoon_h480_kepco')
        self.arm_pub = rospy.Publisher('/'+self.model_name+'/joint_group_position_controller/command', JointTrajectory, queue_size=10)
        #self.pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.offboarding = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.rate = rospy.Rate(40)
        self.hold = 0 #to stop sending input shortly
        self.arm_sign = 0
        self.j1_d = 0
        self.j2_d = 0
        self.j3_d = 0
        self.j4_d = 0

    def pose_callback(self, msg):
        global mav_check
        self.truth=msg.pose.position
        orientation_list = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        mav_check=1

    def joy_callback(self, msg):
        self.joy = msg
        global joy_check
        if len(self.joy.axes)>0 or len(self.joy.buttons)>0 :
            joy_check=1
            if self.joy.buttons[4]==1:
                self.arm_sign = self.arm_sign+1
            if self.joy.axes[6]==-1:
                self.j1_d = self.j1_d + 0.1 * even_odd_to_sign(self.arm_sign)
            if self.joy.axes[6]==1:
                self.j2_d = self.j2_d + 0.1 * even_odd_to_sign(self.arm_sign)
            if self.joy.axes[7]==-1:
                self.j3_d = self.j3_d + 0.1 * even_odd_to_sign(self.arm_sign)
            if self.joy.axes[7]==1:
                self.j4_d = self.j4_d + 0.1 * even_odd_to_sign(self.arm_sign)
            if self.joy.buttons[2]==1:
                print("-------------arming!-----------------")
                self.arming(True)
            if self.joy.buttons[3]==1:
                print("-------------offboard!---------------")
                self.offboarding(base_mode=0, custom_mode="OFFBOARD")
            if self.joy.buttons[0]==1:
                self.hold = self.hold+1            

def input(rbt):
    global d2r
    global r2d
    global max_rate_x
    global max_rate_y
    global max_vel_x
    global max_vel_y
    global max_vel_z
    global yaw_rate

    if rbt.hold%2==0:
        pose_input=PoseStamped()
        #joy_axes: {pitch: 4, roll: 3, yaw: 0, vertical: 1}
        pose_input.pose.position.x= rbt.truth.x + ( rbt.joy.axes[4]*max_vel_x)*cos(rbt.yaw) - ( rbt.joy.axes[3]*max_vel_y)*sin(rbt.yaw)
        pose_input.pose.position.y= rbt.truth.y + ( rbt.joy.axes[3]*max_vel_y)*cos(rbt.yaw) + ( rbt.joy.axes[4]*max_vel_x)*sin(rbt.yaw)
        pose_input.pose.position.z= rbt.truth.z + ( rbt.joy.axes[1]*max_vel_z)
        yaw_input = rpy_saturation(rbt.yaw + yaw_rate*(rbt.joy.axes[0]))
        qq = quaternion_from_euler(0,0,yaw_input)
        pose_input.pose.orientation.x = qq[0]
        pose_input.pose.orientation.y = qq[1]
        pose_input.pose.orientation.z = qq[2]
        pose_input.pose.orientation.w = qq[3]

        print("Input : X: %.2f  Y: %.2f  Z: %.2f  Yaw: %.2f "%(pose_input.pose.position.x, pose_input.pose.position.y, pose_input.pose.position.z, yaw_input))
        print("Position(Meter): X: %.2f Y: %.2f Z: %.2f "%(rbt.truth.x, rbt.truth.y, rbt.truth.z))
        print("Angle(Degree): roll: %.2f pitch: %.2f yaw: %.2f \n"%(rbt.roll/np.pi*180, rbt.pitch/np.pi*180, rbt.yaw/np.pi*180)) #radian : +-pi

        pose_input.header.stamp = rospy.Time.now()
        rbt.position_pub.publish(pose_input)

        data=JointTrajectory()
        da=JointTrajectoryPoint()
        da.positions = [rbt.j1_d, rbt.j2_d, rbt.j3_d, rbt.j4_d]
        da.time_from_start = rospy.Duration(0.1)
        data.joint_names=["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint"]
        data.header.stamp=rospy.Time.now()
        data.points.append(da)
        rbt.arm_pub.publish(data)

    else:
        print("Hold now, press Button[0] to control again")
        print("Position(Meter): X: %.2f Y: %.2f Z: %.2f "%(rbt.truth.x, rbt.truth.y, rbt.truth.z))
        print("Angle(Degree): roll: %.2f pitch: %.2f yaw: %.2f \n"%(rbt.roll/np.pi*180, rbt.pitch/np.pi*180, rbt.yaw/np.pi*180)) #radian : +-pi

##############################################################################################

mav_ctr = robot()
time.sleep(1) #wait 1 second to assure that all data comes in

''' main '''
if __name__ == '__main__':
    while 1:
        try:
            if joy_check==1 and mav_check==1:
                input(mav_ctr)
                mav_ctr.rate.sleep()
            else: 
                mav_ctr.rate.sleep()
                pass
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        #except:
        #    print("exception")
        #    pass
