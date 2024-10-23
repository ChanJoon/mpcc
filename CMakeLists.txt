#     rpg_quadrotor_mpc
#     A model predictive control implementation for quadrotors.
#     Copyright (C) 2017-2018 Philipp Foehn,
#     Robotics and Perception Group, University of Zurich
#
#     Intended to be used with rpg_quadrotor_control and rpg_quadrotor_common.
#     https://github.com/uzh-rpg/rpg_quadrotor_control
#
#     This program is free software: you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation, either version 3 of the License, or
#     (at your option) any later version.
#
#     This program is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
#
#     You should have received a copy of the GNU General Public License
#     along with this program.  If not, see <http://www.gnu.org/licenses/>.


cmake_minimum_required(VERSION 2.8.3)
project(mpcc)

find_package(catkin_simple REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

catkin_simple()

# activate c++ 11
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

# ARM NEON flags
if("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
  message("enabling ARM neon optimizations")
endif()

# flags for speed (should already be enabled by default)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -O3")

add_compile_definitions(MODE_LPP)

cs_add_library(mpcc_solver
  qp_solver/qpoases/SRC/Bounds.cpp
  qp_solver/qpoases/SRC/Constraints.cpp
  qp_solver/qpoases/SRC/CyclingManager.cpp
  qp_solver/qpoases/SRC/Indexlist.cpp
  qp_solver/qpoases/SRC/MessageHandling.cpp
  qp_solver/qpoases/SRC/QProblem.cpp
  qp_solver/qpoases/SRC/QProblemB.cpp
  qp_solver/qpoases/SRC/SubjectTo.cpp
  qp_solver/qpoases/SRC/Utils.cpp
  qp_solver/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  model/quadrotor_mpcc/acado_qpoases_interface.cpp
  model/quadrotor_mpcc/acado_integrator.c
  model/quadrotor_mpcc/acado_solver.c
  model/quadrotor_mpcc/acado_auxiliary_functions.c)

target_include_directories(mpcc_solver PUBLIC
  model/quadrotor_mpcc/
  qp_solver/qpoases
  qp_solver/qpoases/INCLUDE
  qp_solver/qpoases/SRC)
target_compile_definitions(mpcc_solver PRIVATE MODE_LPP)

cs_add_library(mpcc_wrapper
  src/mpcc_wrapper.cpp)

target_link_libraries(mpcc_wrapper
  mpcc_solver)
target_compile_definitions(mpcc_wrapper PRIVATE MODE_LPP)

# make an executable
cs_install()
cs_export()

cs_add_executable(mpcc_test_node
  src/mpcc_test.cpp)

target_link_libraries(mpcc_test_node
  mpcc_wrapper
  mpcc_solver)
target_compile_definitions(mpcc_test_node PRIVATE MODE_LPP)

cs_install()
cs_export()

cs_add_executable(wind_node
	src/wind.cpp)

target_compile_definitions(wind_node PRIVATE MODE_LPP)

cs_install()
cs_export()

cs_add_executable(bezier_node
 test/bezier_curve_test.cpp)

target_compile_definitions(bezier_node PRIVATE MODE_LPP)

cs_install()
cs_export()
