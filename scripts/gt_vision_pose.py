#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 19 00:28:30 2020

@author: mason
"""

''' import libraries '''
import time
import numpy as np

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from gazebo_msgs.msg import ModelStates

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


''' class '''

class gtpub():
    def __init__(self):
        rospy.init_node('gtgt', anonymous=True)
        self.parent_frame_id = rospy.get_param("/parent_frame_id", '/map')
        self.drone_name = rospy.get_param("/drone_name", 'typhoon_h480')
        self.my_pose = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        self.gtpubb = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)

        self.rate = rospy.Rate(30)
        self.pose = Pose()
        self.pose_offset = Pose()
        self.check = 0

    def gazebo_callback(self, msg):
        for i in range(len(msg.name)):
            if msg.name[i] == self.drone_name:
                self.pose = msg.pose[i]
                if self.check == 0 :
                    self.pose_offset = self.pose
                self.check = 1

''' main '''
pub_class = gtpub()
time.sleep(1.0) #wait 1 second to assure that all data comes in

if __name__ == '__main__':
    while 1:
        try:
            if pub_class.check == 1:
                pose = PoseStamped()
                pose.pose.position.x = pub_class.pose.position.x - pub_class.pose_offset.position.x
                pose.pose.position.y = pub_class.pose.position.y - pub_class.pose_offset.position.y
                pose.pose.position.z = pub_class.pose.position.z - pub_class.pose_offset.position.z
                pose.pose.orientation.x = pub_class.pose.orientation.x
                pose.pose.orientation.y = pub_class.pose.orientation.y
                pose.pose.orientation.z = pub_class.pose.orientation.z
                pose.pose.orientation.w = pub_class.pose.orientation.w
                pose.header.frame_id = pub_class.parent_frame_id
                pose.header.stamp = rospy.Time.now()
                
                pub_class.gtpubb.publish(pose)

                pub_class.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        # except:
        #     print("exception")
        #     pass
