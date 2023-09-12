# Model Predicive Contouring Control

### Updates
- 23-09-12
  - 기존 Cost function을 quadratic form 에서 $l_2$-norm으로 바꿔줌. OSQP에서는 Reference에 해당하는 부분을 $x^TQx+q^Tx$의 $Q, q$에 넣어주었지만, ACADO에서는 별도로 reference state가 주어진다. 따라서 *acadoVariables.y*에는 0을 넣어주고 weight matrix를 그대로 사용하고자 함.
  - Gazebo 테스트 중 비정상적으로 비행하여서, 출력문을 통해 디버깅 중.
  - acado_inputs에 값이 정상적으로 변하지 않아 acado_feedbackStep() returnValue로 원인 파악 중.
  - *acadoVariables.y*를 기존 MPC처럼 state+input으로 변경해주어야 할 것으로 보임.
- 23-09-07
  - `acado_Wlx`가 $`273*1`$의 column vector, `mpc_wrapper`의 `Wlx_`는 $13*1$의 column vector이므로 `Eigen::Matrix<T, kStateSize, kStateSize> R` 과 `R_`을 Matrix에서 Vector로 수정.(`kStateSize x (kSamples+1) = 273`)
  - CMPCC를 참고하여 `set_state_est`, `set_params`와 `solve_mpc` 수정.
  - `est_state_`의 `vt`, `at`를 이전 결과로부터 가져오도록 수정. 우선, `solve_from_scratch_` 조건문에 update되는 부분을 넣음. 추후 필요시 리팩토링
    ```cpp
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
    ```
  - `est_state`에서 `t`에 해당하는 부분을 odom의 `timestamp`로 함. `reference_states_`의 `t`를 `target_traj`의 `timestamp`로 하여 같은 타임라인 간의 차이로 생각하여 이렇게 하였으나 추후 문제 시 수정해야함.
  - `mpc_wrapper`에서 잘못된 부분들 일부 수정.(자세한 내용은 commit 내용 확인)
- 23-08-31
  - [quadrotor_model_thrustrates.cpp](model/quadrotor_model_thrustrates.cpp): $-\rho\cdot v_t$를 추가하고 이를 `mpc_wrapper`에 반영 (`acadoVariables.Wlx, Wlu`)
  - [mpc_wrapper.h](include/mpcc/mpc_wrapper.h): `kHoverInput_`을 바뀐 모델에 맞게 초기화 ($t_0, v_{t0}, a_{t0}, \bar{J_{t0}} = 0$)
  - [mpc_wrapper.cpp](src/mpc_wrapper.cpp): `
    - `hover_state`, `acado_reference_states_`과 `acado_reference_end_states_`를 바뀐 모델에 맞게 초기화 (클래스 생성자 파트 참고)
    - `setCosts`에서 linear weighting matrix `Wlx_`를 `R`로 할당하고 `R`을 `kStateSize`로 모두 수정. `Wlu_`는 사용 X. (i.e. $x^TQx+R^Tx$)
    - `setTrajectory`에서 `states_`를 `kRefSize`만큼 `acado_reference_states_` 할당. `setReferencePose`는 `mpc_test.h`에서 사용하지 않으므로 일단 변경 X.

- 23-08-26
  - [quadrotor_model_thrustrates.cpp](model/quadrotor_model_thrustrates.cpp): CMPCC $x^TQx$ 부분 반영(아래 cost function 참조)
  - [mpc_wrapper](include/mpcc/mpc_wrapper.h):
    - sh_mpc에서는 `kRefSize` = `kStateSize` + `kInputSize` 이고, `kCostSize`는 hN 함수에 대한 cost weigt matrix dimension으로 사용됨. 현재 mpcc acado 모델에서는 `kRefSize`=5 < `kStateSize`=13 이므로 kCostSize를 제거하거나 맞게 변경함.
    - mpc에서는 $x^TQx+u^TRx=state^TWstate$ 로 acado QP에 넣어주는데 mpcc에서는 $x^TQx$만 사용하고자, `mpc_wrapper`에서 `Q`의 크기를 `kRefSize`로 변경함.
  - [mpc_test.h](include/mpcc/mpc_test.h): `mpc_wrapper`에 맞게 `Eigen::Matrix Q`를 선언

### TODO List

  - [x] : Update acado model to include $-\rho\cdot v_t$
  - [ ] : How will `yaw` be applied?. (Searching other papers or reformulating $J$)
  - [x] : What is `acado_initializeNodesByForwardSimulation()`?
  - [x] : Initialize `acado_reference_states_`, `acado_reference_end_states_` accordingly
  - [x] : Initialize local variables `W_`, `WN_` in `mpc_wrapper.h` accordingly
  - [x] : Add liear term weighting matrix `Wlx_`, `Wlu_` in `setCosts`
  - [x] : `setReferencePose`, `setTrajectory`, `update`
  - [x] : Cost weight matrix `acado_W_` and `acado_W_end_` should be assigned dynamically in ~~`setCosts`~~, `set_params`
  - [x] : Modify `est_state_`, `reference_states_` with reference to CMPCC(*Update some states from last horizon*)
  - [ ] : ~~Implement `findNearestTheta` and `getGlobalCommand`~~ (Currently using target topics)
  - [x] : Fix `NaN` values from `predicted_states_(STATE::kVelT, 1)` and `(State::kAccT)`
  - [ ] : Test roslaunch and parameter tuning

### Cost function 

$$\begin{align}J&=\sum_{k=1}^N\{(\mu^{(k)}-\mu_p(t^{(k)}))^2-\rho\cdot v_t^{(k)}\}\\
&=\sum_{k=1}^N\{(\mu^{(k)}-\mu_p(\theta^{(k)})-v_p(\theta^{(k)})\cdot (t^{(k)}-\theta^{(k)}))^2-\rho\cdot v_t^{(k)}\}\\
&=\sum_{k=1}^N\begin{bmatrix}\mu\cr t\end{bmatrix}^T
\begin{bmatrix}1 & -v_p(\theta)\cr -v_p(\theta)&v_p^2(\theta)\end{bmatrix}
\begin{bmatrix}\mu\cr t\end{bmatrix}
+\begin{bmatrix}2(-\mu_p(\theta)+v_p(\theta)\cdot \theta)\cr -2v_p(\theta)(-\mu_p(\theta)+v_p(\theta)\cdot \theta)\cr -\rho\end{bmatrix}^T
\begin{bmatrix}\mu\cr t\cr v_t\end{bmatrix}\end{align}$$

---
*Convert quadratic form(OSQP) to a weighted l2-norm(ACADO)*

$$\begin{align}&=\sum_{k=1}^N\|W^{1/2}\begin{bmatrix}\mu\cr t\cr v_t\end{bmatrix}\|^2+q^T
\begin{bmatrix}\mu\cr t\cr v_t\end{bmatrix}\\
&=\sum_{k=1}^N\| \begin{bmatrix}\mu\cr t\cr v_t\end{bmatrix} \|^2_W+q^T\begin{bmatrix}\mu\cr t\cr v_t\end{bmatrix} \\
s.t & \ \ \mu=\begin{bmatrix}p_x, p_y, p_z\end{bmatrix} W=\begin{bmatrix}1 & -v_p(\theta)\cr -v_p(\theta)&v_p^2(\theta)\end{bmatrix},\ q=\begin{bmatrix}2(-\mu_p(\theta)+v_p(\theta)\cdot \theta)\cr -2v_p(\theta)(-\mu_p(\theta)+v_p(\theta)\cdot \theta)\cr -\rho\end{bmatrix}^T\end{align}$$

**The states and inputs**

$$\begin{gather}\textbf{x}=\begin{bmatrix}p_x, p_y, p_z, q_w, q_x, q_y, q_z, v_x, v_y, v_z, t, v_t, a_t\end{bmatrix}\\
\textbf{u}=\begin{bmatrix}T, w_x, w_y, w_z, \bar{J_t}\end{bmatrix}\end{gather}$$

*References*
 - Falanga, Davide, et al. "PAMPC: Perception-aware model predictive control for quadrotors." 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2018. https://doi.org/10.48550/arXiv.1804.04811
 - Ji, Jialin, et al. "Cmpcc: Corridor-based model predictive contouring control for aggressive drone flight." Experimental Robotics: The 17th International Symposium. Springer International Publishing, 2021. https://doi.org/10.48550/arXiv.2007.03271
 - Romero, Angel, et al. "Model predictive contouring control for time-optimal quadrotor flight." IEEE Transactions on Robotics 38.6 (2022): 3340-3356. https://doi.org/10.48550/arXiv.2108.13205








---
# Model Predictive Control for Quadrotors with extension to Perception-Aware MPC
Model Predictive Control for Quadrotors by "Robotics and Perception Group" at the Dep. of Informatics, "University of Zurich", and Dep. of Neuroinformatics, ETH and University of Zurich.

This MPC is intended to be used with https://github.com/uzh-rpg/rpg_quadrotor_control.
It is available with the extension to be used as a "Perception Aware Model Predictive Controller" (**PAMPC**).

[**Check out our YouTube-Video, showing PAMPC in Action**](https://www.youtube.com/watch?v=9vaj829vE18)
[![PAMPC: Perception-Aware Model Predictive Control for Quadrotors](http://rpg.ifi.uzh.ch/img/quad_control/mpc_thumb_button_small.png)](https://www.youtube.com/watch?v=9vaj829vE18)

## Publication
If you use this code in an academic context, please cite the following [IROS 2018 paper](http://rpg.ifi.uzh.ch/docs/IROS18_Falanga.pdf).

Davide Falanga, Philipp Foehn, Peng Lu, Davide Scaramuzza: **PAMPC: Perception-Aware Model Predictive Control for Quadrotors**, IEEE/RSJ Int. Conf. Intell. Robot. Syst. (IROS), 2018.

```
@InProceedings{Falanga2018
  author = {Falanga, Davide and Foehn, Philipp and Lu, Peng and Scaramuzza, Davide},
  title = {{PAMPC}: Perception-Aware Model Predictive Control for Quadrotors},
  booktitle = {IEEE/RSJ Int. Conf. Intell. Robot. Syst. (IROS)},
  year = {2018}
}
```


## License

Copyright (C) 2017-2018 Philipp Foehn, Robotics and Perception Group, University of Zurich

The RPG MPC repository provides packages that are intended to be used with [RPG Quadrotor Control](https://github.com/uzh-rpg/rpg_quadrotor_control) and [ROS](http://www.ros.org/). 
This code has been tested with ROS kinetic on Ubuntu 16.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.
For a commercial license, please contact [Davide Scaramuzza](http://rpg.ifi.uzh.ch/people_scaramuzza.html).

```
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
```

This work depends on the ACADO Toolkit, developed by the Optimization in Engineering Center (OPTEC) under supervision of Moritz Diehl. Licensing detail can be found on the [ACADO licensing page](http://acado.github.io/licensing.html). It is released under GNU Lesser General Public License as published by the Free Software Foundation, version 3.
ACADO uses qpOASES by Hans Joachim Ferreau et al., released under GPL v2.1.

## Installation, Usage, and Documentation
For detailed instructions on installation, usage, and documentation, please head to our [Wiki](../../wiki).

Quick Links:
- [Installation](../../wiki/Installation)
- [Basic Usage](../../wiki/Basic-Usage)
- [Code Structure](../../wiki/Code-Structure)
- [License](../../wiki/License)

## Code Structure Overview
The whole MPC is based on ACADO (http://acado.github.io/).
ACADO's C++ interface is used to describe the quadrotor model and parameters for transcription into a quadratic program, which is then solved with qpOASES (https://projects.coin-or.org/qpOASES). To compile and run, none of these dependencies are needed, since the generated code is already included in this repository. To modify the model and solver options, please install ACADO from http://acado.github.io/install_linux.html.

The code is organized as follows:

### Solver `mpc_solver`

The auto-generated model, transcription, and solver is built as a library called `mpc_solver`.
This library consist of purely auto-generated code with nomenclature, code-style and structure as used in ACADO.

### Wrapper `mpc_wrapper`

To wrap this into a standard interface, the library `mpc_wrapper` is used.
ACADO uses arrays with column-major format to store matrices, since this is rather inconvenient, `mpc_wrapper` provides  interfaces using Eigen objects by mapping the arrays.
* It is written to be compatible even with changing model descriptions in the `mpc_solver`.
* It should prevent the most common runtime errors caused by the user by doing some initialization and checks with hardcoded parameters, which are overwritten in normal usage.

### Controller `mpc_controller`

To provide not only a solver, but a full controller, `mpc_controller` is a library based on the previous `mpc_solver` and `mpc_wrapper`, providing all funcitonality to implement a controller with minimal effort. It provides two main execution modes:

* **Embedded**: The mpc_controller can be included in any oder controller class or copilot by generating an object with the default constructor "MPC::MpcController<T> controller();". It still registers node-handles to publish the predicted trajectory after each control cycle, but does nothing more. It only provides the interfaces of `ControllerBase` as in the `LargeAngleController` but without a specific class for parameters.

* **Standalone (not yet provided)**: The `mpc_controller` object can be passed node-handles or creates its own, registers multiple subscribers and publishers to get a state estimate as well as the control goals (pose or trajectory) and registers a timer to run a control loop. It works as a full standalone controller which can be used with the oneliner: `MPC::MpcController<double> controller(ros::NodeHandle(), ros::NodeHandle("~"));` as in `test/control_node.cpp` and `launch/mpc_controller.launch`.
