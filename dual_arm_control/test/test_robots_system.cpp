#include <uclv_systems_lib/controllers/pi.hpp>
#include <uclv_systems_lib/sim/state_space_system_simulator.hpp>
#include <uclv_systems_lib/ss/linear_state_space.hpp>
#include <uclv_systems_lib/continuous_time/continuous_time_linear_state_space.hpp>
#include <uclv_systems_lib/discretization/forward_euler.hpp>
#include <uclv_systems_lib/observers/ekf.hpp>
#include "../include/robots_object_system.hpp"

int main()
{
  const int number_pose_measure_from_robot = 10;
  Eigen::Matrix<double, 13, 1> x0;      // Initial state
  Eigen::Matrix<double, 6, 6> Bm;      // Inertia matrix
  Eigen::Matrix<double, 3, 1> bg;      // Gravity vector
  Eigen::Matrix4d oTg1;                // Transformation matrix from object to grasp 1
  Eigen::Matrix4d oTg2;                // Transformation matrix from object to grasp 2
  Eigen::Matrix4d b1Tb2;               // Transformation matrix from base 1 to base 2 of robots (could be unknown)
  Eigen::Matrix4d bTb1;                // Transformation matrix from base to base 1  (well known)
  Eigen::Matrix<double, 6, 6> viscous_friction;  // Viscous friction matrix of the object-air


  x0 << 0.0,0.0,0.1,1,0,0,0,0,0,0,0,0,0;
  Bm << Eigen::Matrix<double, 6, 6>::Identity();
  bg << 0, 0, -9.81;
  oTg1 << Eigen::Matrix4d::Identity();
  oTg1.block<3, 1>(0, 3) << 0.1, 0.1, 0.1;
  oTg2 << Eigen::Matrix4d::Identity();
  oTg2.block<3, 1>(0, 3) << -0.1, -0.2, -0.3;
  oTg2.block<3, 3>(0, 0) << -Eigen::Matrix3d::Identity();
  b1Tb2 << Eigen::Matrix4d::Identity();
  b1Tb2.block<3, 1>(0, 3) << 0.0,0.0, 0.1;
  bTb1 << Eigen::Matrix4d::Identity();
  viscous_friction << Eigen::Matrix<double, 6, 6>::Identity();




  auto robots_object_system = std::make_shared<uclv::systems::RobotsObjectSystem<number_pose_measure_from_robot>>(x0, Bm, bg, oTg1, oTg2, b1Tb2, bTb1, viscous_friction);
  robots_object_system->display();

  Eigen::Matrix<double, 13, 1> x;
  Eigen::Matrix<double, 12, 1> u_k;
  Eigen::Matrix<double, number_pose_measure_from_robot*14, 1> y_k;
  u_k << 0.1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;

  robots_object_system->state_fcn(x0, u_k, x);
  std::cout << "State function: " << x.transpose() << std::endl;


  robots_object_system->output_fcn(x0, u_k, y_k);
  std::cout << "Output function: " << y_k.transpose() << std::endl;



  return 0;
}
