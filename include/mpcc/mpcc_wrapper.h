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

#pragma once

#include <Eigen/Eigen>
#include <log++.h>
#include <ros/ros.h>

namespace mpcc {

#include "acado_auxiliary_functions.h"
#include "acado_common.h"

static constexpr int kSamples = ACADO_N;  // number of samples
// 20
static constexpr int kStateSize = ACADO_NX;  // number of states(=differential state variables)
// 13
static constexpr int kInputSize = ACADO_NU;  // number of inputs
// 5
static constexpr int kRefSize = ACADO_NY;  // number of reference states
// 18 (13+5)
static constexpr int kEndRefSize = ACADO_NYN;  // number of end reference states
// 13, Code generation을 위해 임의로 두었고 MPCC에서는 사용 X
static constexpr int kOdSize = ACADO_NOD;  // number of online data
// 0 일단 불필요함

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

template <typename T>
class MpcWrapper {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MpcWrapper();
  MpcWrapper(
      // const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kStateSize>> Q,
      // const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
      // const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> q);
      const Eigen::Ref<const Eigen::Matrix<T, kRefSize, kRefSize * kSamples>> Q,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize*(kSamples + 1), 1, Eigen::ColMajor>> q);

  bool setCosts(
      // const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kStateSize>> Q,
      const Eigen::Ref<const Eigen::Matrix<T, kRefSize, kRefSize * kSamples>> Q,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
      // const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> q,
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize*(kSamples + 1), 1, Eigen::ColMajor>> q,
      const T state_cost_scaling = 0.0,
      const T input_cost_scaling = 0.0);

  bool setLimits(T min_thrust, T max_thrust, T max_rollpitchrate, T max_yawrate, T max_jerk);

  bool setReferencePose(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state);
  bool setTrajectory(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
                     const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples + 1>> inputs);

  bool solve(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state);
  bool update(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
              bool do_preparation = true);
  bool prepare();

  bool setadaptiveacc(const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& ada_acc);

  void getState(const int node_index, Eigen::Ref<Eigen::Matrix<T, kStateSize, 1>> return_state);
  void getStates(Eigen::Ref<Eigen::Matrix<T, kStateSize, kSamples + 1>> return_states);
  void getInput(const int node_index, Eigen::Ref<Eigen::Matrix<T, kInputSize, 1>> return_input);
  void getInputs(Eigen::Ref<Eigen::Matrix<T, kInputSize, kSamples>> return_input);
  T getTimestep() { return dt_; }

 private:
  Eigen::Map<Eigen::Matrix<float, kRefSize, kSamples, Eigen::ColMajor>> acado_reference_states_{
      acadoVariables.y};
  Eigen::Matrix<float, kRefSize, kSamples> reference_states_;

  Eigen::Map<Eigen::Matrix<float, kEndRefSize, 1, Eigen::ColMajor>> acado_reference_end_state_{
      acadoVariables.yN};

  Eigen::Map<Eigen::Matrix<float, kStateSize, 1, Eigen::ColMajor>> acado_initial_state_{
      acadoVariables.x0};

  Eigen::Map<Eigen::Matrix<float, kStateSize, kSamples + 1, Eigen::ColMajor>> acado_states_{
      acadoVariables.x};

  Eigen::Map<Eigen::Matrix<float, kInputSize, kSamples, Eigen::ColMajor>> acado_inputs_{
      acadoVariables.u};

  Eigen::Map<Eigen::Matrix<float, kOdSize, kSamples + 1, Eigen::ColMajor>> acado_online_data_{
      acadoVariables.od};

  Eigen::Map<Eigen::Matrix<float, kRefSize, kRefSize * kSamples>> acado_W_{acadoVariables.W};

  Eigen::Map<Eigen::Matrix<float, kEndRefSize, kEndRefSize>> acado_W_end_{acadoVariables.WN};

  // Linear term weighting vector
  Eigen::Map<Eigen::Matrix<float, kStateSize*(kSamples + 1), 1, Eigen::ColMajor>> acado_Wlx_{
      acadoVariables.Wlx};

  Eigen::Map<Eigen::Matrix<float, kInputSize * kSamples, 1, Eigen::ColMajor>> acado_Wlu_{
      acadoVariables.Wlu};

  Eigen::Map<Eigen::Matrix<float, 5, kSamples, Eigen::ColMajor>> acado_lower_bounds_{
      acadoVariables.lbValues};

  Eigen::Map<Eigen::Matrix<float, 5, kSamples, Eigen::ColMajor>> acado_upper_bounds_{
      acadoVariables.ubValues};

  Eigen::Matrix<T, kRefSize, kRefSize> W_ =
      (Eigen::Matrix<T, kRefSize, 1>() << 10 * Eigen::Matrix<T, 3, 1>::Ones(),
       100 * Eigen::Matrix<T, 4, 1>::Ones(),
       10 * Eigen::Matrix<T, 3, 1>::Ones(),
       1 * Eigen::Matrix<T, 3, 1>::Ones(),
       1 * Eigen::Matrix<T, 4, 1>::Ones(),
       1.0)
          .finished()
          .asDiagonal();

  Eigen::Matrix<T, kEndRefSize, kEndRefSize> WN_ = W_.block(0, 0, kEndRefSize, kEndRefSize);

  Eigen::Matrix<T, kStateSize, 1> Wlx_ = Eigen::Matrix<T, kStateSize, 1>::Zero();

  Eigen::Matrix<T, kInputSize, 1> Wlu_ = Eigen::Matrix<T, kInputSize, 1>::Zero();

  bool acado_is_prepared_{false};
  const T dt_{0.1};
  const Eigen::Matrix<real_t, kInputSize, 1> kHoverInput_ =
      (Eigen::Matrix<real_t, kInputSize, 1>() << 9.81, 0.0, 0.0, 0.0, 0.0).finished();
  int preparation_status;
};

}  // namespace mpcc
