/*    rpg_quadrotor_mpc
 *    A model predictive control implementation for quadrotors.
 *    Copyright (C) 2017-2018 Philipp Foehn, 
 *    Robotics and Perception Group, University of Zurich
 * 
 *    Intended to be used with rpg_quadrotor_control and rpg_quadrotor_common.
 *    https://github.com/uzh-rpg/rpg_quadrotor_control
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


#include "mpcc/mpc_wrapper.h"


namespace mpcc {

// Default Constructor.
template <typename T>
MpcWrapper<T>::MpcWrapper()
{
  // Clear solver memory.
  memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
  memset(&acadoVariables, 0, sizeof( acadoVariables ));
  
  // Initialize the solver.
  acado_initializeSolver();

  // Initialize the states and controls.
  const Eigen::Matrix<T, kStateSize-3, 1> hover_state =
    (Eigen::Matrix<T, kStateSize-3, 1>() << 0.0, 0.0, 0.0,
                                          1.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0).finished();

  // Initialize states x and xN and input u.
  acado_initial_state_.block(0, 0, kStateSize-3, 1) = hover_state.template cast<float>();

  acado_initial_state_.block(kStateSize-3, 0, 3, 1) = 
    (Eigen::Matrix<float, 3, 1>() << 0.0, 0.0, 0.0).finished();

  acado_states_.block(0, 0, kStateSize-3, kSamples+1) = 
    hover_state.replicate(1, kSamples+1).template cast<float>();
  
  for(int i = 0; i < kSamples+1; ++i){
    acado_states_(kStateSize-3, i) = i * dt_;
    acado_states_(kStateSize-2, i) = 0.5;
    acado_states_(kStateSize-1, i) = 0;
  }

  acado_inputs_ = kHoverInput_.replicate(1, kSamples).template cast<float>();

  // Initialize references y and yN.
  acado_reference_states_.setZero();
  // acado_reference_states_.block(0, 0, kStateSize-3, kSamples) =
  //   hover_state.replicate(1, kSamples).template cast<float>();

  // //TODO t에 대한 initial 값을 어떻게 주면 될지? N = 20 dt = 0.1, t = 2
  // for(int i = 0; i < kSamples; ++i){
  //   acado_reference_states_(kStateSize-3, i) = i * dt_;
  //   acado_reference_states_(kStateSize-2, i) = 0.5;
  //   acado_reference_states_(kStateSize-1, i) = 0;
  // }

  // acado_reference_states_.block(kStateSize, 0, kInputSize, kSamples) = 
  //   kHoverInput_.replicate(1, kSamples);

  
  acado_reference_end_state_.segment(0, kEndRefSize) =
    Eigen::Matrix<float, kEndRefSize, 1>::Zero();

  // Initialize Cost matrix W and WN.
  if(!(acado_W_.trace()>0.0))
  {
    acado_W_ = W_.replicate(1, kSamples).template cast<float>();
    acado_W_end_ = WN_.template cast<float>();
  }

  // Initialize solver.
  acado_initializeNodesByForwardSimulation();
  preparation_status = acado_preparationStep();
  acado_is_prepared_ = true;
}

// Constructor with cost matrices as arguments.
template <typename T>
MpcWrapper<T>::MpcWrapper(
  const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kStateSize>> Q,
  const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
  const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> q)
{
  setCosts(Q, R, q);
  MpcWrapper();
}

// Set cost matrices with optional scaling.
template <typename T>
bool MpcWrapper<T>::setCosts(
  const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kStateSize>> Q,
  const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
  const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> q,
  const T state_cost_scaling, const T input_cost_scaling)
{
  if(state_cost_scaling < 0.0 || input_cost_scaling < 0.0 )
  {
    ROS_ERROR("MPC: Cost scaling is wrong, must be non-negative!");
    return false;
  }
  W_.block(0, 0, kStateSize, kStateSize) = Q;
  W_.block(kStateSize, kStateSize, kInputSize, kInputSize) = R;

  Wlx_.block(0, 0, kStateSize, 1) = q;

  WN_.setZero();

  float state_scale{1.0};
  float input_scale{1.0};
  // Set cost matrix acadoVariables.W from k=0,1,...,N with scaled W_
  for(int i=0; i<kSamples; i++)
  { 
    state_scale = exp(- float(i)/float(kSamples)
      * float(state_cost_scaling));
    input_scale = exp(- float(i)/float(kSamples)
      * float(input_cost_scaling));
    acado_W_.block(0, i * kRefSize, kStateSize, kStateSize) =
      W_.block(0, 0, kStateSize, kStateSize).template cast<float>()
      * state_scale;
    acado_W_.block(kStateSize, i*kRefSize+kStateSize, kInputSize, kInputSize) =
      W_.block(kStateSize, kStateSize, kInputSize, kInputSize).template cast<float>()
     * input_scale;

    acado_Wlx_.block(i * kStateSize, 0, kStateSize, 1) =
      Wlx_.template cast<float>() * state_scale;
  }
  state_scale = exp(- 1.0 * float(state_cost_scaling));
  acado_Wlx_.block(kSamples * kStateSize, 0, kStateSize, 1) = 
    Wlx_.template cast<float>() * state_scale;

  acado_W_end_ = WN_.template cast<float>();

  std::cout << "acadoVariable.W [" << std::endl << acado_W_.block(0, 0, kRefSize, kRefSize) << "]" << std::endl;
  // std::cout << "acadoVariable.Wlx [" << std::endl << acado_Wlx_.block(0, 0, kStateSize, 1) << "]" << std::endl;
  // std::cout << "acadoVariable.WN [" << std::endl << acado_W_end_.block(0, 0, kEndRefSize, kEndRefSize) << "]" << std::endl;

  return true;
}

// Set the input limits.
template <typename T>
bool MpcWrapper<T>::setLimits(T min_thrust, T max_thrust,
    T max_rollpitchrate, T max_yawrate, T max_jerk)
{
  if(min_thrust <= 0.0 || min_thrust > max_thrust)
  {
    ROS_ERROR("MPC: Minimal thrust is not set properly, not changed.");
    return false;
  }

  if(max_thrust <= 0.0 || min_thrust > max_thrust)
  {
    ROS_ERROR("MPC: Maximal thrust is not set properly, not changed.");
    return false;
  }

  if(max_rollpitchrate <= 0.0)
  {
    ROS_ERROR("MPC: Maximal xy-rate is not set properly, not changed.");
    return false;
  }

  if(max_yawrate <= 0.0)
  {
    ROS_ERROR("MPC: Maximal yaw-rate is not set properly, not changed.");
    return false;
  }

  // Set input boundaries.
  Eigen::Matrix<T, kInputSize, 1> lower_bounds = Eigen::Matrix<T, kInputSize, 1>::Zero();
  Eigen::Matrix<T, kInputSize, 1> upper_bounds = Eigen::Matrix<T, kInputSize, 1>::Zero();
  lower_bounds << min_thrust,
    -max_rollpitchrate, -max_rollpitchrate, -max_yawrate, -max_jerk;
  upper_bounds << max_thrust,
    max_rollpitchrate, max_rollpitchrate, max_yawrate, max_jerk;

  acado_lower_bounds_ =
    lower_bounds.replicate(1, kSamples).template cast<float>();

  acado_upper_bounds_ =
    upper_bounds.replicate(1, kSamples).template cast<float>();
  return true;
}

// Set a reference pose.
template <typename T>
bool MpcWrapper<T>::setReferencePose(
  const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state)
{
// This function is not used in mpc_test.h. If used, should be modified.
  acado_reference_states_.block(0, 0, kStateSize, kSamples) =
    state.replicate(1, kSamples).template cast<float>();

  acado_reference_states_.block(kStateSize, 0, kInputSize, kSamples) =
    kHoverInput_.replicate(1, kSamples);

// We don't use acado_reference_end_state.
  // acado_reference_end_state_.segment(0, kStateSize) =
    // state.template cast<float>();

  // acado_reference_end_state_.segment(kStateSize, 0) =
    // Eigen::Matrix<float, 0, 1>::Zero();

  acado_initializeNodesByForwardSimulation();
  return true;
}

// Set a reference trajectory.
template <typename T>
bool MpcWrapper<T>::setTrajectory(
  const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples+1>> states,
  const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples+1>> inputs)
{
  Eigen::Map<Eigen::Matrix<float, kRefSize, kSamples, Eigen::ColMajor>>
    y(const_cast<float*>(acadoVariables.y));

  acado_reference_states_.setZero();
  // q_w, q_x, q_y, q_z, v_x, v_y, v_z
  acado_reference_states_.block(3, 0, kStateSize-6, kSamples) =
    states.block(3, 0, kStateSize-6, kSamples).template cast<float>();
  // at
  acado_reference_states_.block(kStateSize-1, 0, 1, kSamples) =
    states.block(kStateSize-1, 0, 1, kSamples).template cast<float>();
  acado_reference_states_.block(kStateSize, 0, kInputSize, kSamples) =
    inputs.block(0, 0, kInputSize, kSamples).template cast<float>();

// Store reference_states_ to use quaternion
  // reference_states_.block(0, 0, kStateSize, kSamples) =
    // states.block(0, 0, kStateSize, kSamples).template cast<float>();
  // reference_states_.block(kStateSize, 0, kInputSize, kSamples) =
    // inputs.block(0, 0, kInputSize, kSamples).template cast<float>();

// We don't use acado_reference_end_state.
  // acado_reference_end_state_.segment(0, kStateSize) =
    // states.col(kSamples).template cast<float>();
  // acado_reference_end_state_.segment(kStateSize, 0) =
    // Eigen::Matrix<float, 0, 1>::Zero();

  return true;
}

// Update dynamic using adaptive control module
template <typename T>
bool MpcWrapper<T>::setadaptiveacc(
  const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& ada_acc)
{
  acado_online_data_.block(0, 0, 3, ACADO_N+1)
    = ada_acc.replicate(1, ACADO_N+1).template cast<float>();
  return true;
}


// Reset states and inputs and calculate new solution.
template <typename T>
bool MpcWrapper<T>::solve(
  const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state)
{
  acado_states_ = state.replicate(1, kSamples+1).template cast<float>();

  acado_inputs_ = kHoverInput_.replicate(1, kSamples);

  return update(state);
}


// Calculate new solution from last known solution.
template <typename T>
bool MpcWrapper<T>::update(
  const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
  bool do_preparation)
{
  if(!acado_is_prepared_)
  {
    ROS_WARN("MPC: Solver was triggered without preparation, abort!");
    return false;
  }
  std::cout << "acado_states_ [" << std::endl << acado_states_.col(0) << "]" << std::endl;
  std::cout << "acado_reference_states_ [" << std::endl << acado_reference_states_.col(0) << "]" << std::endl;


  // Check if estimated and reference quaternion live in sthe same hemisphere.
  acado_initial_state_ = state.template cast<float>();
  if(acado_initial_state_.segment(3,4).dot(
    Eigen::Vector4f(acado_reference_states_.block(3,0,4,1)))<(T)0.0)
    // Eigen::Vector4f(reference_states_.block(3,0,4,1)))<(T)0.0)
  {
    acado_initial_state_.segment(3,4) = -acado_initial_state_.segment(3,4);
  }

  // Perform feedback step and reset preparation check.
  int solve_status = acado_feedbackStep();
  acado_is_prepared_ = false;

  // std::cout << "preparation_status " << preparation_status << std::endl;
  std::cout << "QPOASES status " << solve_status << std::endl;
  std::cout << "Objective value " << acado_getObjective() << std::endl;

  // Prepare if the solver if wanted
  if(do_preparation)
  {
    acado_preparationStep();
    acado_is_prepared_ = true;
  }

  return true;
}

// Prepare the solver.
// Must be triggered between iterations if not done in the update function.
template <typename T>
bool MpcWrapper<T>::prepare()
{
  acado_preparationStep();
  acado_is_prepared_ = true;

  return true;
}

// Get a specific state.
template <typename T>
void MpcWrapper<T>::getState(const int node_index,
    Eigen::Ref<Eigen::Matrix<T, kStateSize, 1>> return_state)
{
  return_state = acado_states_.col(node_index).cast<T>();
}

// Get all states.
template <typename T>
void MpcWrapper<T>::getStates(
    Eigen::Ref<Eigen::Matrix<T, kStateSize, kSamples+1>> return_states)
{
  return_states = acado_states_.cast<T>();
}

// Get a specific input.
template <typename T>
void MpcWrapper<T>::getInput(const int node_index,
    Eigen::Ref<Eigen::Matrix<T, kInputSize, 1>> return_input)
{
  return_input = acado_inputs_.col(node_index).cast<T>();
}

// Get all inputs.
template <typename T>
void MpcWrapper<T>::getInputs(
    Eigen::Ref<Eigen::Matrix<T, kInputSize, kSamples>> return_inputs)
{
  return_inputs = acado_inputs_.cast<T>();
}

template <typename T>
void MpcWrapper<T>::getVerbose()
{
  acado_printDifferentialVariables();
	acado_printControlVariables();
}

template class MpcWrapper<float>;
template class MpcWrapper<double>;

} // namespace rpg_mpc

