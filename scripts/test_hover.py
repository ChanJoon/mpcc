#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TwistStamped

def hover():

    rospy.init_node('traj_circle_node')
    pub = rospy.Publisher('/target_pos', Odometry, queue_size=10)
    pub_path = rospy.Publisher('/target_path', Path, queue_size=10)
    pub_traj = rospy.Publisher('/target_traj', Path, queue_size=10)
    f = 10
    rate = rospy.Rate(f) 
    t=0.0
    v = 1.0
    dt = v/f/4
    t_off=5.0
    hov_offset=t_off*f*dt

    msg = Odometry()
    msg.header.frame_id = "map"

    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = 'map'

    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now() 
        msg.pose.pose.position.z = 5.0  
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0  
        traj_msg = Path()
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.header.frame_id = 'map'
        for i in range(0,25):
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.z = 5.0
            pose_msg.pose.orientation.w = 1.0
            traj_msg.poses.append(pose_msg)
        # print("Hovering")

        t=t+dt
        pub.publish(msg)  
        pub_path.publish(path_msg)
        pub_traj.publish(traj_msg)
        rate.sleep()  

if __name__ == '__main__':
    try:
        hover()
    except rospy.ROSInterruptException:
        pass
