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
    pub_pose = rospy.Publisher('/target_pose', PoseStamped, queue_size=10)
    f = 10
    rate = rospy.Rate(f) 
    t=0.0
    v = 1.0
    dt = v/f/4
    t_off=7.0
    hov_offset=t_off*f*dt

    msg = Odometry()
    msg.header.frame_id = "map"

    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = 'map'

    i=0.0
    while i<30:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = 4.0*math.cos(i/3)-4.0
        pose_msg.pose.position.y = 4.0*math.sin(i/3)
        pose_msg.pose.position.z = 5.0
        pose_msg.pose.orientation.w = 1.0
        path_msg.poses.append(pose_msg)
        i=i+0.5

    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now() 
        if(t<hov_offset):
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
            print("Hovering")
        else:
            t2=t-hov_offset
            msg.pose.pose.position.x = 4.0*math.cos(t2)-4.0
            msg.pose.pose.position.y = 4.0*math.sin(t2)
            msg.pose.pose.position.z = 5.0  
            msg.twist.twist.linear.x = -4.0*math.sin(t2)*dt*f
            msg.twist.twist.linear.y = 4.0*math.cos(t2)*dt*f
            msg.twist.twist.linear.z = 0
            msg.pose.pose.orientation.z = 0.0
            msg.pose.pose.orientation.w = 1.0  
        
            traj_msg = Path()
            traj_msg.header.stamp = rospy.Time.now()
            traj_msg.header.frame_id = 'map'
            for i in range(0,25):
                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = 'map'
                pose_msg.pose.position.x = 4.0*math.cos(t2+i/100)-4.0
                pose_msg.pose.position.y = 4.0*math.sin(t2+i/100)
                pose_msg.pose.position.z = 5.0
                pose_msg.pose.orientation.w = 1.0
                traj_msg.poses.append(pose_msg)

        t=t+dt
        pub.publish(msg)  
        pub_path.publish(path_msg)
        pub_traj.publish(traj_msg)
        pub_pose.publish(traj_msg.poses[0])
        rate.sleep()  

if __name__ == '__main__':
    try:
        hover()
    except rospy.ROSInterruptException:
        pass
