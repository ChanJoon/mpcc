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

#include <math.h>
#include <memory>
#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>

// Standalone code generation for a parameter-free quadrotor model
// with thrust and rates input. 

int main( ){
  // Use Acado
  USING_NAMESPACE_ACADO

  /// System variables
  /// The state vectors of the system x = [p, v, q]^T
  DifferentialState     p_x, p_y, p_z;      // the position of the body frame w.r.t the world frame
  DifferentialState     q_w, q_x, q_y, q_z; // the orientation of the body frame w.r.t the world frame
  DifferentialState     v_x, v_y, v_z;      // the linear velocity of the body frame w.r.t the body frame
  DifferentialState     t, vt, at;

  // the mass normalized thrust vector c = (0, 0, c)^T = (0, 0, (f1+f2+f3+f4)/m)^T
  // The input vectors of the system u = [c, \omega]^T
  Control               T, w_x, w_y, w_z;
  Control               jt;

  DifferentialEquation  f;
  Function              h, hN;

  // L1 Control accelerations
  OnlineData            ada_acc_x, ada_acc_y, ada_acc_z;


  // Parameters with exemplary values. These are set/overwritten at runtime.
  const double t_start = 0.0;     // Initial time [s]
  const double t_end = 2.0;       // Time horizon [s]
  const double dt = 0.1;          // Discretization time [s]
  const int N = round(t_end/dt);  // Number of nodes
  const double g_z = 9.8066;      // Gravity is everywhere [m/s^2]
  const double w_max_yaw = 1;     // Maximal yaw rate [rad/s]
  const double w_max_xy = 3;      // Maximal pitch and roll rate [rad/s]
  const double T_min = 2;         // Minimal thrust [N]
  const double T_max = 20;        // Maximal thrust [N]
  const double a_max = 20;
  const double at_max = 3;

  // Bias to prevent division by zero.
  const double epsilon = 0.1;     // Camera projection recover bias [m]


  // System Dynamics
  // M. W. Mueller, M. Hehn and R. D'Andrea, "A Computationally Efficient Motion Primitive for Quadrocopter Trajectory Generation," in IEEE Transactions on Robotics, vol. 31, no. 6, pp. 1294-1310, Dec. 2015, doi: 10.1109/TRO.2015.2479878.
  f << dot(p_x) ==  v_x;
  f << dot(p_y) ==  v_y;
  f << dot(p_z) ==  v_z;
  f << dot(q_w) ==  0.5 * ( - w_x * q_x - w_y * q_y - w_z * q_z);
  f << dot(q_x) ==  0.5 * ( w_x * q_w + w_z * q_y - w_y * q_z);
  f << dot(q_y) ==  0.5 * ( w_y * q_w - w_z * q_x + w_x * q_z);
  f << dot(q_z) ==  0.5 * ( w_z * q_w + w_y * q_x - w_x * q_y);
  f << dot(v_x) ==  2 * ( q_w * q_y + q_x * q_z ) * T + ada_acc_x;
  f << dot(v_y) ==  2 * ( q_y * q_z - q_w * q_x ) * T + ada_acc_y;
  f << dot(v_z) ==  ( 1 - 2 * q_x * q_x - 2 * q_y * q_y ) * T - g_z + ada_acc_z;
  f << dot(t) == vt;
  f << dot(vt) == at;
  f << dot(at) == jt;


  // Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
  // Running cost vector consists of position, time, v_time.
  h << p_x << p_y << p_z
    << t << vt;

  // End cost vector consists of all states (no inputs at last state).
  hN << p_x << p_y << p_z;



  // DEFINE AN OPTIMAL CONTROL PROBLEM:
  // ----------------------------------
  OCP ocp( t_start, t_end, N );

  // For code generation, references are set during run time.
  // Running cost weight matrix & End cost weight matrix are set during run time.
  BMatrix Q_sparse(h.getDim(), h.getDim());
  Q_sparse.setIdentity();
  BMatrix QN_sparse(hN.getDim(), hN.getDim());
  QN_sparse.setIdentity();
  ocp.minimizeLSQ( Q_sparse, h);
  ocp.minimizeLSQEndTerm( QN_sparse, hN );
  // Running cost vector of linear term Sum(i=0, ..., N-1){qT * x} (maybe)
  BVector Slx, Slu;
  Slx.setZero();
  Slu.setZero();
  ocp.minimizeLSQLinearTerms( Slx, Slu );

  // Add system dynamics
  ocp.subjectTo( f );
  // Add constraints
  ocp.subjectTo(-w_max_xy <= w_x <= w_max_xy);
  ocp.subjectTo(-w_max_xy <= w_y <= w_max_xy);
  ocp.subjectTo(-w_max_yaw <= w_z <= w_max_yaw);
  ocp.subjectTo( T_min <= T <= T_max);

  ocp.setNOD(10);


  // For code generation, we can set some properties.
  // The main reason for a setting is given as comment.
  OCPexport mpc(ocp);

  mpc.set(HESSIAN_APPROXIMATION,  GAUSS_NEWTON);        // is robust, stable
  mpc.set(DISCRETIZATION_TYPE,    MULTIPLE_SHOOTING);   // good convergence
  mpc.set(SPARSE_QP_SOLUTION,     FULL_CONDENSING_N2);  // due to qpOASES
  mpc.set(INTEGRATOR_TYPE,        INT_IRK_GL4);         // accurate
  mpc.set(NUM_INTEGRATOR_STEPS,   N);
  mpc.set(QP_SOLVER,              QP_QPOASES);          // free, source code
  mpc.set(HOTSTART_QP,            YES);
  mpc.set(CG_USE_OPENMP,                    YES);       // paralellization
  mpc.set(CG_HARDCODE_CONSTRAINT_VALUES,    NO);        // set on runtime
  mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);       // time-varying costs
  mpc.set( USE_SINGLE_PRECISION,        YES);           // Single precision

  // Do not generate tests, makes or matlab-related interfaces.
  mpc.set( GENERATE_TEST_FILE,          YES);
  mpc.set( GENERATE_MAKE_FILE,          NO);
  mpc.set( GENERATE_MATLAB_INTERFACE,   NO);
  mpc.set( GENERATE_SIMULINK_INTERFACE, NO);

  // Finally, export everything.
  if(mpc.exportCode("quadrotor_mpcc") != SUCCESSFUL_RETURN)
    exit( EXIT_FAILURE );
  mpc.printDimensionsQP( );

  return EXIT_SUCCESS;
}