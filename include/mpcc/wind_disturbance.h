#ifndef INCLUDE_MPCC_WIND_DISTURBANCE_H_
#define INCLUDE_MPCC_WIND_DISTURBANCE_H_

///// common headers
#include <math.h>  // pow

#include <random>
#include <string>
#include <vector>

#include <ros/ros.h>

///// ROS headers
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>

#include "utils/log++.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////
class wind_disturbance {
 public:
  // no meaning for public, just separate ROS and main variables
  geometry_msgs::Pose m_uav_pose;
  gazebo_msgs::ApplyBodyWrench m_model_force_srv;

  bool m_state_check = false;
  std::string m_robot_name;

  // wind points
  vector<double> m_wind_spec;

  ///// ros and tf
  ros::NodeHandle nh;
  ros::Subscriber states_sub;
  ros::ServiceClient m_model_pusher;
  ros::Timer main_timer;

  void if_wind_disturbance(const geometry_msgs::Pose &pose);
  void main_timer_func(const ros::TimerEvent &event);
  void states_callback(const gazebo_msgs::ModelStates::ConstPtr &msg);
  double gaussian_distribution(const double &mean, const double &stddev);

  explicit wind_disturbance(ros::NodeHandle &n) : nh(n) {
    ///// params
    nh.param<std::string>("/m_robot_name", m_robot_name, "iris");
    nh.getParam("/windpoints", m_wind_spec);

    ///// Init
    m_model_force_srv.request.body_name = "iris::base_link";
    m_model_force_srv.request.duration = ros::Duration(0.1);

    ///// sub pub
    states_sub = nh.subscribe<gazebo_msgs::ModelStates>(
        "/gazebo/model_states", 3, &wind_disturbance::states_callback, this);
    m_model_pusher = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

    //// timers
    main_timer = nh.createTimer(ros::Duration(1 / 24.4), &wind_disturbance::main_timer_func, this);

    ROS_WARN("wind node...");
  }
  ~wind_disturbance() {}
};

////////////////////// definitions, can be separated to .cpp files
/////// timer functions
/////// callbacks
void wind_disturbance::states_callback(const gazebo_msgs::ModelStates::ConstPtr &msg) {
  for (int i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == m_robot_name) {
      m_uav_pose = msg->pose[i];
      m_state_check = true;
    }
  }
}
void wind_disturbance::main_timer_func(const ros::TimerEvent &event) {
  if (m_state_check) {
    if_wind_disturbance(m_uav_pose);
  }
}
void wind_disturbance::if_wind_disturbance(const geometry_msgs::Pose &pose) {
  double x = pose.position.x;
  double y = pose.position.y;
  double z = pose.position.z;

  for (int i = 0; i < m_wind_spec.size() / 9; ++i) {
    if (m_wind_spec[i * 9] < x && m_wind_spec[i * 9 + 1] < y && m_wind_spec[i * 9 + 2] < z &&
        x < m_wind_spec[i * 9 + 3] && y < m_wind_spec[i * 9 + 4] && z < m_wind_spec[i * 9 + 5]) {
      if (m_wind_spec[i * 9 + 6] == 0.0) {  // +x direction
        m_model_force_srv.request.wrench.force.x =
            1.0 * gaussian_distribution(m_wind_spec[i * 9 + 7], m_wind_spec[i * 9 + 8]);
        m_model_force_srv.request.wrench.force.y = 0.0;
        m_model_force_srv.request.wrench.force.z = 0.0;
      } else if (m_wind_spec[i * 9 + 6] == 1.0) {  // -y direction
        m_model_force_srv.request.wrench.force.x = 0.0;
        m_model_force_srv.request.wrench.force.y =
            -1.0 * gaussian_distribution(m_wind_spec[i * 9 + 7], m_wind_spec[i * 9 + 8]);
      } else if (m_wind_spec[i * 9 + 6] == 2.0) {  // +y direction
        m_model_force_srv.request.wrench.force.x = 0.0;
        m_model_force_srv.request.wrench.force.y =
            1.0 * gaussian_distribution(m_wind_spec[i * 9 + 7], m_wind_spec[i * 9 + 8]);
        m_model_force_srv.request.wrench.force.z = 0.0;
      } else if (m_wind_spec[i * 9 + 6] == 3.0) {  // -x direction
        m_model_force_srv.request.wrench.force.x =
            -1.0 * gaussian_distribution(m_wind_spec[i * 9 + 7], m_wind_spec[i * 9 + 8]);
        m_model_force_srv.request.wrench.force.y = 0.0;
        m_model_force_srv.request.wrench.force.z = 0.0;
      }
      m_model_pusher.call(m_model_force_srv);
      break;  // no need to inspect more
    } else {
      m_model_force_srv.request.wrench.force.x = 0.0;
      m_model_force_srv.request.wrench.force.y = 0.0;
      m_model_force_srv.request.wrench.force.z = 0.0;
    }
  }
}
double wind_disturbance::gaussian_distribution(const double &mean, const double &stddev) {
  default_random_engine generator;
  normal_distribution<double> dist(mean, stddev);
  return dist(generator);
}

#endif  // INCLUDE_MPCC_WIND_DISTURBANCE_H_
