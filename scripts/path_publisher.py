#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path


class PathVisualizer:
    def __init__(self):
        rospy.init_node('path_visualizer')
        self.path_pub = rospy.Publisher('/uav_path', Path, queue_size=10)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.path = Path()
        self.path.header.frame_id = "map"
        self.path.header.stamp = rospy.Time.now()
        self.rate = rospy.Rate(10)  # publishing rate in Hz

    def pose_callback(self, msg):
        # Create a new pose message for the path
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        pose.pose.orientation = Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)

        # Append the new pose message to the path
        self.path.poses.append(pose)

        # Publish the updated path
        self.path_pub.publish(self.path)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':
    visualizer = PathVisualizer()
    visualizer.run()
