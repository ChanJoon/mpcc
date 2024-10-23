#ifndef INCLUDE_MPCC_TEST_H_
#define INCLUDE_MPCC_TEST_H_

///// C++ common headers
#include <math.h>
#include <time.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

///// Eigen
#include <Eigen/Eigen>  // whole Eigen library: Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

//// Conversions
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>

///// ROS
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>

// MAVROS
#include <log++.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>  // arming
#include <mavros_msgs/CommandLong.h>  // disarming (kill service)
#include <mavros_msgs/SetMode.h>      // offboarding
#include <mavros_msgs/State.h>
// MPC
#include "mpcc/bezier_curve.h"
#include "mpcc/mpcc_wrapper.h"

using namespace std;
using namespace std::chrono;
using namespace Eigen;
using namespace mpcc;

enum STATE {
  kPosX = 0,
  kPosY = 1,
  kPosZ = 2,
  kOriW = 3,
  kOriX = 4,
  kOriY = 5,
  kOriZ = 6,
  kVelX = 7,
  kVelY = 8,
  kVelZ = 9,
  kTime = 10,
  kVelT = 11,
  kAccT = 12
};

enum INPUT { kThrust = 0, kRateX = 1, kRateY = 2, kRateZ = 3, kJerkT = 4 };

////////////////////////////////////////////////////////////////////////////////////////////////////
class MPCC {
 public:
  double ctrl_hz, m_cutoff_hz, um_cutoff_hz;
  double thrust_const, thrust_offset, max_thrust, min_thrust, K_adaacc;
  double As_1, As_2, As_3, As_4, As_5, As_6;
  double rho;

  bool state_check = false, target_check = false, L1_start = false;
  bool ctrl_init = false, L1_on = false, debug = false;

  Vector3d gravity{Vector3d(0.0, 0.0, -9.81)}, Kp, Kv;

  mavros_msgs::State curr_state;
  mavros_msgs::AttitudeTarget att_msg;

  Vector3d pos, vel, rpy, t_pos, t_vel, t_vel_prev, t_acc, t_rpy;
  Matrix3d rot, t_rot;
  Vector4d att, t_att;

  nav_msgs::Path target_traj;

  // Bezier
  BEZIER bezier;
  Matrix<double, 3, 6> points;
  Matrix<double, 3, kSamples> bezier_points;
  Matrix<double, 3, kSamples> bezier_vel;

  // L1 adaptive control
  Vector3d L1_vel_error, L1_ang_error;
  Matrix3d L1_rot_error;

  Vector3d predict_pos, predict_vel;
  Matrix3d predict_rot;

  Vector4d ctbr_cmd, ada_cmd, m_sigma;
  Vector2d um_sigma;
  Vector3d ada_acc;

  // MPC
  MpcWrapper<double> mpc_wrapper_;
  thread preparation_thread_;

  clock_t timing_preparation_, timing_feedback_;

  bool solve_from_scratch_ = true, preparation_ = false;
  bool changed_ = true;

  Matrix<double, kStateSize, 1> est_state_;
  Matrix<double, kStateSize, 1> est_state_prev;
  Matrix<double, kStateSize, kSamples + 1> reference_states_;
  Matrix<double, kInputSize, kSamples + 1> reference_inputs_;
  Matrix<double, kStateSize, kSamples + 1> predicted_states_;
  Matrix<double, kInputSize, kSamples> predicted_inputs_;
  Matrix<double, kStateSize, 1> predicted_state_i;
  // Matrix<double, kStateSize, kStateSize> Q_;
  Matrix<double, kRefSize, kRefSize * kSamples> Q_;
  Matrix<double, kInputSize, kInputSize> R_;
  // Matrix<double, kStateSize, 1> q_;
  Matrix<double, kStateSize*(kSamples + 1), 1, ColMajor> q_;
  Matrix<double, kStateSize, 1> q_tmp;
  Matrix<double, kStateSize, 1> grad_x;
  Matrix<double, kStateSize, 1> grad_y;
  Matrix<double, kStateSize, 1> grad_z;

  double max_bodyrate_xy_, max_bodyrate_z_, max_throttle_, min_throttle_, max_jerk_, Q_pos_xy_,
      Q_pos_z_, Q_attitude_, Q_velocity_, Q_vt_, Q_at_, R_thrust_, R_pitchroll_, R_yaw_, R_jt_,
      state_cost_exponential_, input_cost_exponential_;

  // ROS
  ros::NodeHandle nh;
  ros::Subscriber m_state_sub, m_odom_sub, m_target_sub, m_target_traj_sub;
  ros::Publisher m_pos_ctrl_pub, m_ctbr_pub, m_pub_MPC_traj;
  ros::ServiceClient m_arming_client, m_set_mode_client, m_kill_client;
  ros::Timer m_control_timer;

  ros::Time ctrl_start_time, t_time, t_time_prev, t_odom, t_odom_prev, t_state;
  double dt_odom;

  ///// functions
  void state_cb(const mavros_msgs::State::ConstPtr& msg);
  void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
  void target_cb(const nav_msgs::Odometry::ConstPtr& msg);
  void target_traj_cb(const nav_msgs::Path::ConstPtr& msg);
  void control_timer_func(const ros::TimerEvent& event);
  void pos_ctrl(Vector3d target_pos, Vector4d target_att);
  void pub_cmd();
  void solve_mpc();
  void preparationThread();
  bool set_params();
  bool set_state_est();
  bool pub_predict(const Ref<const Matrix<double, kStateSize, kSamples + 1>> states,
                   const Ref<const Matrix<double, kInputSize, kSamples>> inputs,
                   ros::Time& time);
  void compute_L1adaptive();
  double throttle_mapping(double thrust);
  double throttle_mapping_inverse(double throttle);
  Matrix3d hat_operator(Vector3d v);
  Vector3d vee_operator(Matrix3d R);
  Matrix3d exp_operator(Vector3d v);

  explicit MPCC(const ros::NodeHandle& n_private) : nh(n_private) {
    ////////// ROS things
    // params
    nh.param("/control_hz", ctrl_hz, 10.0);
    nh.param("/norm_thrust_const", thrust_const, 0.06);
    nh.param("/norm_thrust_offset", thrust_offset, 0.1);
    nh.param("/max_thrust", max_thrust, 0.9);
    nh.param("/min_thrust", min_thrust, 0.0);
    nh.param("/max_throttle", max_throttle_, 0.5);
    nh.param("/min_throttle", min_throttle_, 0.0);
    nh.param("/max_bodyrate_xy", max_bodyrate_xy_, 0.5);
    nh.param("/max_bodyrate_z", max_bodyrate_z_, 0.5);
    nh.param("/max_jerk", max_jerk_, 15.0);
    nh.param("/Q_pos_xy", Q_pos_xy_, 0.1);
    nh.param("/Q_pos_z", Q_pos_z_, 0.1);
    nh.param("/Q_attitude", Q_attitude_, 0.1);
    nh.param("/Q_velocity", Q_velocity_, 0.1);
    nh.param("/Q_vt", Q_vt_, 0.1);
    nh.param("/Q_at", Q_at_, 0.1);
    nh.param("/R_thrust", R_thrust_, 0.1);
    nh.param("/R_pitchroll", R_pitchroll_, 0.1);
    nh.param("/R_yaw", R_yaw_, 0.1);
    nh.param("/R_jt", R_jt_, 0.1);
    nh.param("/state_cost_exponential", state_cost_exponential_, 0.1);
    nh.param("/input_cost_exponential", input_cost_exponential_, 0.1);
    nh.param("/cutoff_freq_m", m_cutoff_hz, 1.0);
    nh.param("/cutoff_freq_um", um_cutoff_hz, 1.0);
    nh.param("/As_1", As_1, 2.0);
    nh.param("/As_2", As_2, 2.0);
    nh.param("/As_3", As_3, 2.0);
    nh.param("/As_4", As_4, 2.0);
    nh.param("/As_5", As_5, 2.0);
    nh.param("/As_6", As_6, 2.0);
    nh.param("/K_adaacc", K_adaacc, 1.0);
    nh.param("/L1_on", L1_on, true);
    nh.param("/debug", debug, true);
    nh.param("/rho", rho, 0.005);  // a weight of aggressiveness

    // publishers
    m_pos_ctrl_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 3);
    m_ctbr_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    m_pub_MPC_traj = nh.advertise<nav_msgs::Path>("/predicted_path", 1);

    // subscribers
    m_state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &MPCC::state_cb, this);
    m_odom_sub =
        nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &MPCC::odom_cb, this);
    m_target_sub = nh.subscribe<nav_msgs::Odometry>("/target_pos", 10, &MPCC::target_cb, this);
    m_target_traj_sub =
        nh.subscribe<nav_msgs::Path>("/target_traj", 10, &MPCC::target_traj_cb, this);

    // service clients
    m_arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    m_kill_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    m_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // timers
    ctrl_start_time = ros::Time::now();
    m_control_timer = nh.createTimer(ros::Duration(1 / ctrl_hz), &MPCC::control_timer_func, this);

    ROS_WARN("Controller heritated, starting node...");
  }
};

//////////////////////// callbacks
void MPCC::state_cb(const mavros_msgs::State::ConstPtr& msg) {
  curr_state = *msg;
  state_check = true;
}

void MPCC::odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
  dt_odom = (msg->header.stamp - t_odom).toSec();
  t_odom = msg->header.stamp;
  pos << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  att << msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z;
  if (msg->pose.pose.orientation.w < 0) {
    att = -att;
  }

  tf::Quaternion m_q(att(1), att(2), att(3), att(0));
  tf::Matrix3x3 m(m_q);
  m.getRPY(rpy(0), rpy(1), rpy(2));
  Quaterniond q(m_q.w(), m_q.x(), m_q.y(), m_q.z());  // tf::Quaternion -> Eigen::Quaterniond
  rot = q.normalized().toRotationMatrix();

  // local velocity -> global velocity
  Vector3d local_vel;
  local_vel << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
  vel = rot * local_vel;
}

void MPCC::target_cb(const nav_msgs::Odometry::ConstPtr& msg) {
  t_time = ros::Time::now();
  t_pos << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  t_vel << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
  t_att << msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z;

  tf::Quaternion t_q(t_att(1), t_att(2), t_att(3), t_att(0));
  tf::Matrix3x3 t(t_q);
  Quaterniond qq(t_q.w(), t_q.x(), t_q.y(), t_q.z());  // tf::Quaternion -> Eigen::Quaterniond
  t_rot = qq.normalized().toRotationMatrix();
  t.getRPY(t_rpy(0), t_rpy(1), t_rpy(2));

  // target acceleration
  if (!target_check) {
    t_acc = Vector3d::Zero();
    target_check = true;
  } else {
    t_acc = (t_vel - t_vel_prev) / (t_time - t_time_prev).toSec();
  }
  t_time_prev = t_time;
  t_vel_prev = t_vel;
}

void MPCC::target_traj_cb(const nav_msgs::Path::ConstPtr& msg) {
  target_traj = *msg;
  int total_poses = target_traj.poses.size();
  int s = (total_poses - 1) / bezier.traj_order;  // interval

  for (int i = 0; i < bezier.traj_order + 1; ++i) {
    points(0, i) = target_traj.poses[i * s].pose.position.x;
    points(1, i) = target_traj.poses[i * s].pose.position.y;
    points(2, i) = target_traj.poses[i * s].pose.position.z;
  }

  bezier.calculateBezierCurve(points);
}

void MPCC::control_timer_func(const ros::TimerEvent& event) {
  if (state_check) {
    if (!ctrl_init) {  // intially taking off and go forward
      if (!curr_state.armed) {
        mavros_msgs::CommandBool arming_command;
        arming_command.request.value = true;
        m_arming_client.call(arming_command);
        ROS_WARN("Arming...");
        return;
      } else if (curr_state.mode != "OFFBOARD") {
        Vector3d pos0{Vector3d(0.0, 0.0, 0.0)};
        Vector4d att0{Vector4d(1.0, 0.0, 0.0, 0.0)};
        pos_ctrl(pos0, att0);

        mavros_msgs::SetMode offboarding_command;
        offboarding_command.request.custom_mode = "OFFBOARD";
        m_set_mode_client.call(offboarding_command);
        ROS_WARN("Offboarding...");

        ctrl_start_time = ros::Time::now();
        est_state_ << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, ctrl_start_time.toSec(), 0, 0;
        return;
      } else {
        if (target_check) {
          if (!preparation_) {
            preparation_thread_ = std::thread(&MpcWrapper<double>::prepare, mpc_wrapper_);
            preparation_ = true;
          }
          pub_cmd();
        }
        return;
      }
    }
  }
}

void MPCC::pos_ctrl(Vector3d target_pos, Vector4d target_att) {
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.pose.position.x = target_pos(0);
  goal_pose.pose.position.y = target_pos(1);
  goal_pose.pose.position.z = target_pos(2);
  goal_pose.pose.orientation.x = target_att(1);
  goal_pose.pose.orientation.y = target_att(2);
  goal_pose.pose.orientation.z = target_att(3);
  goal_pose.pose.orientation.w = target_att(0);
  m_pos_ctrl_pub.publish(goal_pose);
}

void MPCC::pub_cmd() {
  // Baseline
  solve_mpc();

  if (L1_on) {
    compute_L1adaptive();
  } else {
    ada_cmd.setZero();
  }
  // Pub commands
  att_msg.header.stamp = ros::Time::now();
  att_msg.header.frame_id = "map";
  att_msg.type_mask = 128;  // Ignore orientation messages

  att_msg.body_rate.x = ctbr_cmd(INPUT::kRateX) + ada_cmd(0);
  att_msg.body_rate.y = ctbr_cmd(INPUT::kRateY) + ada_cmd(1);
  att_msg.body_rate.z = ctbr_cmd(INPUT::kRateZ) + ada_cmd(2);

  att_msg.thrust = throttle_mapping(ctbr_cmd(INPUT::kThrust) + ada_cmd(3));

  ROS_INFO("r: %.2f p: %.2f y: %.2f Thrust: %.2f",
           att_msg.body_rate.x,
           att_msg.body_rate.y,
           att_msg.body_rate.z,
           att_msg.thrust);
  // //dummy values
  // att_msg.orientation.w = 1.0;
  // att_msg.orientation.x = 0.0;
  // att_msg.orientation.y = 0.0;
  // att_msg.orientation.z = 0.0;
  m_ctbr_pub.publish(att_msg);
}

////////////////////// MPC //////////////////////
void MPCC::solve_mpc() {
  ros::Time call_time = ros::Time::now();  // == tMPC(CMPCC)
  const clock_t start = clock();

  // Set cost weight matrices dynamically (remove boolean `changed_`)
  set_params();
  // Setup
  preparation_thread_.join();
  // Convert everything into Eigen format.
  set_state_est();

  // Approximate est_state_
  // TODO(ChanJoon): CMPCC에서는 근사하고 last horizon에서 update해야 하는 부분은 새로 업데이트
  // 한다. set_state_est()에서 어느 부분까지 가져올지 결정해야 함. t_state = ros::Time::now();
  // est_state_ = (est_state_ - est_state_prev)/dt_odom*(t_state-t_odom).toSec() + est_state_;

  // setReference(reference_trajectory);
  reference_inputs_.setZero();
  reference_states_.setZero();
  clock_t dt = mpc_wrapper_.getTimestep();
  for (int i = 0; i < kSamples + 1; i++) {
    reference_states_(0, i) = target_traj.poses[i].pose.position.x;
    reference_states_(1, i) = target_traj.poses[i].pose.position.y;
    reference_states_(2, i) = target_traj.poses[i].pose.position.z;
    reference_states_(3, i) = target_traj.poses[i].pose.orientation.w;
    reference_states_(4, i) = target_traj.poses[i].pose.orientation.x;
    reference_states_(5, i) = target_traj.poses[i].pose.orientation.y;
    reference_states_(6, i) = target_traj.poses[i].pose.orientation.z;
    reference_states_(7, i) = t_vel(0);
    reference_states_(8, i) = t_vel(1);
    reference_states_(9, i) = t_vel(2);
    reference_states_(10, i) = 0;  // t
    reference_states_(11, i) = 0;  // vt
    reference_states_(12, i) = 0;  // at

    reference_inputs_(0, i) = 9.8066;  // T
    reference_inputs_(1, i) = 0;       // w_x
    reference_inputs_(2, i) = 0;       // w_y
    reference_inputs_(3, i) = 0;       // w_z
    reference_inputs_(4, i) = 0;       // jt
  }
  // Get the feedback from MPC.
  mpc_wrapper_.setTrajectory(reference_states_, reference_inputs_);

  static const bool do_preparation_step(false);

  // SOLVE MPC
  if (solve_from_scratch_) {
    ROS_INFO("Solving MPC with hover as initial guess.");
    mpc_wrapper_.solve(est_state_);
    solve_from_scratch_ = false;
  } else {
    est_state_(STATE::kVelT) = predicted_states_(STATE::kVelT, 1);
    est_state_(STATE::kAccT) = predicted_states_(STATE::kAccT, 1);
    mpc_wrapper_.update(est_state_, do_preparation_step);
  }
  mpc_wrapper_.getStates(predicted_states_);
  mpc_wrapper_.getInputs(predicted_inputs_);

  if (debug) {
    ROS_INFO(
        "State [ %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f ]",
        predicted_states_(0, 0),
        predicted_states_(1, 0),
        predicted_states_(2, 0),
        predicted_states_(3, 0),
        predicted_states_(4, 0),
        predicted_states_(5, 0),
        predicted_states_(6, 0),
        predicted_states_(7, 0),
        predicted_states_(8, 0),
        predicted_states_(9, 0),
        predicted_states_(10, 0),
        predicted_states_(11, 0),
        predicted_states_(12, 0));
    ROS_INFO("Input [ %.2f, %.2f, %.2f, %.2f, %.2f ]",
             predicted_inputs_(0, 0),
             predicted_inputs_(1, 0),
             predicted_inputs_(2, 0),
             predicted_inputs_(3, 0),
             predicted_inputs_(4, 0));
    pub_predict(predicted_states_, predicted_inputs_, call_time);
  }

  preparation_thread_ = std::thread(&MPCC::preparationThread, this);

  // Timing
  const clock_t end = clock();
  timing_feedback_ =
      0.9 * timing_feedback_ + 0.1 * static_cast<double>(end - start) / CLOCKS_PER_SEC;
  if (debug)
    ROS_INFO_THROTTLE(1.0,
                      "MPC Timing: Latency: %1.1ld ms  |  Total: %1.1ld ms",
                      timing_feedback_ * 1000,
                      (timing_feedback_ + timing_preparation_) * 1000);

  // Calculate input
  Matrix<double, kInputSize, 1> input_bounded = predicted_inputs_.col(0).template cast<double>();

  // Bound inputs for sanity.
  ctbr_cmd(INPUT::kThrust) =
      std::max(min_throttle_, std::min(max_throttle_, input_bounded(INPUT::kThrust)));
  ctbr_cmd(INPUT::kRateX) =
      std::max(-max_bodyrate_xy_, std::min(max_bodyrate_xy_, input_bounded(INPUT::kRateX)));
  ctbr_cmd(INPUT::kRateY) =
      std::max(-max_bodyrate_xy_, std::min(max_bodyrate_xy_, input_bounded(INPUT::kRateY)));
  ctbr_cmd(INPUT::kRateZ) =
      std::max(-max_bodyrate_z_, std::min(max_bodyrate_z_, input_bounded(INPUT::kRateZ)));
}

void MPCC::preparationThread() {
  const clock_t start = clock();
  mpc_wrapper_.prepare();
  // Timing
  const clock_t end = clock();
  timing_preparation_ =
      0.9 * timing_preparation_ + 0.1 * static_cast<double>(end - start) / CLOCKS_PER_SEC;
}

bool MPCC::set_params() {
  Q_.setZero();
  q_.setZero();
  grad_x.setZero();
  grad_y.setZero();
  grad_z.setZero();

  bezier.getPos(bezier_points);
  bezier.getVel(bezier_vel);

  double theta = 0.0;
  for (int i = 0; i < kSamples; i++) {
    if (solve_from_scratch_) {
      theta = ctrl_start_time.toSec();
    } else {
      mpc_wrapper_.getState(i, predicted_state_i);
      theta = predicted_state_i(STATE::kTime, 0);
    }
    double x_v = bezier_points(0, i);
    double y_v = bezier_points(1, i);
    double z_v = bezier_points(2, i);
    double dx_v__dtheta = bezier_vel(0, i);
    double dy_v__dtheta = bezier_vel(1, i);
    double dz_v__dtheta = bezier_vel(2, i);
    double r_x = x_v - dx_v__dtheta * theta;
    double r_y = y_v - dy_v__dtheta * theta;
    double r_z = z_v - dz_v__dtheta * theta;

    grad_x.coeffRef(0, 0) = 1;
    grad_y.coeffRef(1, 0) = 1;
    grad_z.coeffRef(2, 0) = 1;
    grad_x.coeffRef(10, 0) = -dx_v__dtheta;
    grad_y.coeffRef(10, 0) = -dy_v__dtheta;
    grad_z.coeffRef(10, 0) = -dz_v__dtheta;

    Matrix<double, kStateSize, kStateSize> Q_tmp =
        grad_x * grad_x.transpose() + grad_y * grad_y.transpose() + grad_z * grad_z.transpose();
    Q_tmp.block(3, 3, 7, 7) = (Matrix<double, 7, 1>() << Q_attitude_,
                               Q_attitude_,
                               Q_attitude_,
                               Q_attitude_,
                               Q_velocity_,
                               Q_velocity_,
                               Q_velocity_)
                                  .finished()
                                  .asDiagonal();  // Assign weights from q to v
    Q_tmp.block(11, 11, 2, 2) =
        (Matrix<double, 2, 1>() << Q_vt_, Q_at_).finished().asDiagonal();  // Assign weights vt, at

    Q_.block(0, i * kRefSize, kStateSize, kStateSize) = Q_tmp.block(0, 0, kStateSize, kStateSize);

    q_tmp = -r_x * grad_x - r_y * grad_y - r_z * grad_z;
    q_tmp.coeffRef(11, 0) = -rho;

    q_.block(i * kStateSize, 0, kStateSize, 1) = q_tmp;
  }
  // Q_.block(3, 3, 7, 7) = (Matrix<double, 7, 1>() <<
  // Q_attitude_, Q_attitude_, Q_attitude_, Q_attitude_,
  // Q_velocity_, Q_velocity_, Q_velocity_
  // ).finished().asDiagonal();

  // R_.setIdentity();  // Assign weights to T and w
  R_ = (Matrix<double, kInputSize, 1>() << R_thrust_, R_pitchroll_, R_pitchroll_, R_yaw_, R_jt_)
           .finished()
           .asDiagonal();

  mpc_wrapper_.setCosts(Q_, R_, q_, state_cost_exponential_, input_cost_exponential_);
  mpc_wrapper_.setLimits(
      min_throttle_, max_throttle_, max_bodyrate_xy_, max_bodyrate_z_, max_jerk_);

  changed_ = false;
  return true;
}

bool MPCC::set_state_est() {
  est_state_prev = est_state_;
  est_state_(STATE::kPosX) = pos(0);
  est_state_(STATE::kPosY) = pos(1);
  est_state_(STATE::kPosZ) = pos(2);
  est_state_(STATE::kOriW) = att(0);
  est_state_(STATE::kOriX) = att(1);
  est_state_(STATE::kOriY) = att(2);
  est_state_(STATE::kOriZ) = att(3);
  est_state_(STATE::kVelX) = vel(0);
  est_state_(STATE::kVelY) = vel(1);
  est_state_(STATE::kVelZ) = vel(2);
  est_state_(STATE::kTime) = t_odom.toSec();
  est_state_(STATE::kVelT) = 0;
  est_state_(STATE::kAccT) = 0;
  return true;
}

bool MPCC::pub_predict(const Ref<const Matrix<double, kStateSize, kSamples + 1>> states,
                       const Ref<const Matrix<double, kInputSize, kSamples>> inputs,
                       ros::Time& time) {
  nav_msgs::Path path_msg;
  path_msg.header.stamp = time;
  path_msg.header.frame_id = "map";
  geometry_msgs::PoseStamped pose;
  clock_t dt = mpc_wrapper_.getTimestep();
  for (int i = 0; i < kSamples; i++) {
    pose.header.stamp = time + ros::Duration(i * dt);
    pose.header.seq = i;
    pose.pose.position.x = states(kPosX, i);
    pose.pose.position.y = states(kPosY, i);
    pose.pose.position.z = states(kPosZ, i);
    pose.pose.orientation.w = states(kOriW, i);
    pose.pose.orientation.x = states(kOriX, i);
    pose.pose.orientation.y = states(kOriY, i);
    pose.pose.orientation.z = states(kOriZ, i);
    path_msg.poses.push_back(pose);
  }

  m_pub_MPC_traj.publish(path_msg);
  return true;
}

////////////////////// L1 //////////////////////
void MPCC::compute_L1adaptive() {
  if (!L1_start) {
    L1_start = true;
    predict_vel = vel;
    predict_rot = rot;
    predict_pos = pos;
    return;
  } else {
    L1_vel_error = predict_vel - vel;
    L1_rot_error = 0.5 * (predict_rot.transpose() * rot - rot.transpose() * predict_rot);
    L1_ang_error = vee_operator(L1_rot_error);

    VectorXd As_vec(6), z_error(6);
    As_vec << As_1, As_2, As_3, As_4, As_5, As_6;
    z_error << L1_vel_error(0), L1_vel_error(1), L1_vel_error(2), L1_ang_error(0), L1_ang_error(1),
        L1_ang_error(2);

    MatrixXd As(6, 6), exp_AsTs(6, 6);
    As = (-As_vec).asDiagonal();
    exp_AsTs = (As / ctrl_hz).exp();

    MatrixXd B_bar = MatrixXd::Zero(6, 6);
    B_bar.block<3, 1>(0, 3) = rot.col(2);
    B_bar.block<3, 1>(0, 4) = rot.col(0);
    B_bar.block<3, 1>(0, 5) = rot.col(1);
    B_bar.block<3, 3>(3, 0) = MatrixXd::Identity(3, 3);

    VectorXd sigma = -B_bar.inverse() * (exp_AsTs - MatrixXd::Identity(6, 6)).inverse() * As *
                     exp_AsTs * z_error;

    m_sigma << sigma(0), sigma(1), sigma(2), sigma(3);
    um_sigma << sigma(4), sigma(5);

    double m_wTs = m_cutoff_hz / ctrl_hz;
    double um_wTs = um_cutoff_hz / ctrl_hz;

    // LPF
    ada_cmd = exp(-m_wTs) * ada_cmd - (1 - exp(-m_wTs)) * (m_sigma);
    ada_acc(0) = exp(-um_wTs) * ada_acc(0) - (1 - exp(-um_wTs)) * (um_sigma(0));
    ada_acc(1) = exp(-um_wTs) * ada_acc(1) - (1 - exp(-um_wTs)) * (um_sigma(1));

    // Unmatched disturbance
    Vector3d um_ada_cmd = -K_adaacc * rot.block<3, 3>(0, 0) * ada_acc;
    mpc_wrapper_.setadaptiveacc(um_ada_cmd);

    if (debug) {
      ROS_INFO("Sigma: %.2f %.2f %.2f %.2f %.2f %.2f",
               sigma(0),
               sigma(1),
               sigma(2),
               sigma(3),
               sigma(4),
               sigma(5));
      ROS_INFO(
          "Adaptive command: %.2f %.2f %.2f %.2f", ada_cmd(0), ada_cmd(1), ada_cmd(2), ada_cmd(3));
      ROS_INFO("Adaptive acc: %.2f %.2f %.2f", um_ada_cmd(0), um_ada_cmd(1), um_ada_cmd(2));
      // ROS_INFO("Ada mag: %.2f", um_ada_cmd.norm());
    }

    // Prediction step
    predict_vel = predict_vel +
                  (gravity + rot.col(2) * (throttle_mapping_inverse(att_msg.thrust) + m_sigma(3)) +
                   rot.col(0) * um_sigma(0) + rot.col(1) * um_sigma(1) +
                   As.block<3, 3>(0, 0) * L1_vel_error) /
                      ctrl_hz;

    predict_rot =
        predict_rot * exp_operator((ctbr_cmd.block<3, 1>(1, 0) + ada_cmd.head(3) + m_sigma.head(3) +
                                    As.block<3, 3>(3, 3) * L1_ang_error) /
                                   ctrl_hz);
  }
}

// Utility functions
double MPCC::throttle_mapping(double thrust) {
  return max(min_thrust, min(max_thrust, thrust_const * thrust + thrust_offset));
}

double MPCC::throttle_mapping_inverse(double throttle) {
  return (throttle - thrust_offset) / thrust_const;
}

Matrix3d MPCC::hat_operator(Vector3d v) {
  Matrix3d R;
  R << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
  return R;
}

Vector3d MPCC::vee_operator(Matrix3d R) {
  Vector3d v;
  v << R(7), R(2), R(3);
  return v;
}

Matrix3d MPCC::exp_operator(Vector3d v) {
  Matrix3d v_hat;
  v_hat = hat_operator(v);
  double v_norm = v.norm();
  if (v_norm < 1e-10) {
    return Matrix3d::Identity(3, 3);
  }
  Matrix3d omega = v_hat / v_norm;
  return MatrixXd::Identity(3, 3) + sin(v_norm) * omega + (1 - cos(v_norm)) * omega * omega;
}
#endif  // INCLUDE_MPCC_TEST_H_
