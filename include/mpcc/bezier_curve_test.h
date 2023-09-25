#ifndef BEZIER_CURVE_TEST_H
#define BEZIER_CURVE_TEST_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <mpcc/bezier_curve.h>

class BEZIER_TEST {
	public:
		BEZIER bezier;
		Eigen::Matrix<double, 3, 6> points;

		ros::NodeHandle nh;
		ros::Publisher bezier_path_pub;
		ros::Subscriber path_sub;

		void pathCallback(const nav_msgs::Path::ConstPtr& msg);
		void pubBezierPath(const Eigen::Ref<const Eigen::Matrix<double, 3, N>> bezier_points);

		BEZIER_TEST(const ros::NodeHandle& n_private) : nh(n_private) {
			ROS_INFO("Bezier node initialized");

    	bezier_path_pub = nh.advertise<nav_msgs::Path>("bezier_path", 10);
    	path_sub = nh.subscribe<nav_msgs::Path>("target_traj", 10, &BEZIER_TEST::pathCallback, this);
		}

};

void BEZIER_TEST::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
	std::vector<geometry_msgs::PoseStamped> poses = msg->poses;
	int total_poses = poses.size();
	int s = (total_poses - 1) / bezier.traj_order; // interval

	for(int i = 0; i < bezier.traj_order+1; ++i) {
		points(0, i) = poses[i * s].pose.position.x;
		points(1, i) = poses[i * s].pose.position.y;
		points(2, i) = poses[i * s].pose.position.z;
	}

	bezier.calculateBezierCurve(points);

	Eigen::Matrix<double, 3, N> bezier_points;
	bezier.getPos(bezier_points);

	pubBezierPath(bezier_points);
}

void BEZIER_TEST::pubBezierPath(const Eigen::Ref<const Eigen::Matrix<double, 3, N>> bezier_points) {
	nav_msgs::Path bezier_path;
	bezier_path.header.stamp = ros::Time::now();
	bezier_path.header.frame_id = "map";

	for (int i = 0; i < bezier_points.cols(); ++i) {
		geometry_msgs::PoseStamped pose;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "map";
		
		pose.pose.position.x = bezier_points(0, i);
		pose.pose.position.y = bezier_points(1, i);
		pose.pose.position.z = bezier_points(2, i);
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;

		bezier_path.poses.push_back(pose);
	}
	bezier_path_pub.publish(bezier_path);
}

#endif