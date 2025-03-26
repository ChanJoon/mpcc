cmake_minimum_required(VERSION 3.3)
project(mpcc)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC -fopenmp")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
  tf
  cv_bridge
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLOG REQUIRED libglog)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mpcc_solver mpcc_wrapper
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs mavros_msgs tf cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS} ${GLOG_LIBRARY_DIRS})

if("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
  message("enabling ARM neon optimizations")
endif()

add_compile_definitions(MODE_LPP)

add_library(mpcc_solver
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
  model/quadrotor_mpcc/acado_auxiliary_functions.c
)

target_include_directories(mpcc_solver PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/model/quadrotor_mpcc
  ${CMAKE_CURRENT_SOURCE_DIR}/qp_solver/qpoases
  ${CMAKE_CURRENT_SOURCE_DIR}/qp_solver/qpoases/INCLUDE
  ${CMAKE_CURRENT_SOURCE_DIR}/qp_solver/qpoases/SRC
)

add_library(mpcc_wrapper
  src/mpcc_wrapper.cpp
)

target_link_libraries(mpcc_wrapper
  mpcc_solver
  ${GLOG_LIBRARIES}
)

add_executable(mpcc_test_node
  src/mpcc_test.cpp
  src/astar.cpp
  src/utils/grid_map.cpp
  src/utils/raycast.cpp
)

target_link_libraries(mpcc_test_node
  mpcc_wrapper
  mpcc_solver
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${GLOG_LIBRARIES}
)

add_executable(wind_node
  src/utils/wind.cpp
)

target_link_libraries(wind_node
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARIES}
)

add_executable(bezier_node
  test/bezier_curve_test.cpp
)

target_link_libraries(bezier_node
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARIES}
)

install(TARGETS mpcc_solver mpcc_wrapper mpcc_test_node wind_node bezier_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
