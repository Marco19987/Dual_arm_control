#include <uclv_systems_lib/controllers/pi.hpp>
#include <uclv_systems_lib/sim/state_space_system_simulator.hpp>
#include <uclv_systems_lib/ss/linear_state_space.hpp>
#include <uclv_systems_lib/continuous_time/continuous_time_linear_state_space.hpp>
#include <uclv_systems_lib/discretization/forward_euler.hpp>
#include <uclv_systems_lib/observers/ekf.hpp>
#include "../include/robots_object_system.hpp"
#include "../include/robots_object_system_ext.hpp"
#include "../include/robots_object_system_ext_forces.hpp"


int main()
{
  const int number_pose_measure_from_robot = 1;
  Eigen::Matrix<double, 13, 1> x0;      // Initial state
  Eigen::Matrix<double, 6, 6> Bm;      // Inertia matrix
  Eigen::Matrix<double, 3, 1> bg;      // Gravity vector
  Eigen::Matrix4d oTg1;                // Transformation matrix from object to grasp 1
  Eigen::Matrix4d oTg2;                // Transformation matrix from object to grasp 2
  Eigen::Matrix4d b1Tb2;               // Transformation matrix from base 1 to base 2 of robots (could be unknown)
  Eigen::Matrix4d bTb1;                // Transformation matrix from base to base 1  (well known)
  Eigen::Matrix<double, 6, 6> viscous_friction;  // Viscous friction matrix of the object-air


  x0 << 0.0,0.0,0.0,1,0,0,0,0,0,0,0,0,0;
  Bm << Eigen::Matrix<double, 6, 6>::Identity();
  bg << 0, 0, -9.81;
  oTg1 << Eigen::Matrix4d::Identity();
  oTg1.block<3, 1>(0, 3) << -0.1, 0.0, 0.0;
  oTg2 << Eigen::Matrix4d::Identity();
  oTg2.block<3, 1>(0, 3) << 0.1, 0.0, 0.0;
  oTg2.block<3, 3>(0, 0) << -Eigen::Matrix3d::Identity();
  b1Tb2 << Eigen::Matrix4d::Identity();
  b1Tb2.block<3, 1>(0, 3) << 0.2,0.0, 0.0;
  bTb1 << Eigen::Matrix4d::Identity();
  bTb1.block<3, 1>(0, 3) << -0.1,0.0, 0.0;
  viscous_friction << Eigen::Matrix<double, 6, 6>::Identity();




  auto robots_object_system = std::make_shared<uclv::systems::RobotsObjectSystem>(x0, Bm, bg, oTg1, oTg2, b1Tb2, bTb1, viscous_friction,number_pose_measure_from_robot);

  Eigen::Matrix<double, 20, 1> x;
  Eigen::Matrix<double, 12, 1> u_k;
  Eigen::Matrix<double, Eigen::Dynamic, 1> y_k;
  y_k.resize(number_pose_measure_from_robot * 14, 1);
  u_k << 0.1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;


  Eigen::Matrix<double, 20, 1> x0_ext;
  x0_ext << x0, 0,0,1,1,0,0,0;
  auto robots_object_system_ext = std::make_shared<uclv::systems::RobotsObjectSystemExt>(x0_ext, robots_object_system);

  Eigen::Matrix<double, 32, 1> x0_ext_forces;
  x0_ext_forces << x0_ext, 0,0,0,0,0,0,0,0,0,0,0,0;
  auto robots_object_system_ext_forces = std::make_shared<uclv::systems::RobotsObjectSystemExtForces>(x0_ext_forces, robots_object_system_ext);

  Eigen::Matrix<double, 0, 1> u_k_forces;
  Eigen::Matrix<double, 32, 1> x_forces;
  Eigen::Matrix<double, Eigen::Dynamic, 1> y_k_forces;


  robots_object_system_ext_forces->display();

  robots_object_system_ext_forces->state_fcn(x0_ext_forces, u_k_forces, x_forces);
  std::cout << "State function: " << x_forces.transpose() << std::endl;


  robots_object_system_ext_forces->output_fcn(x0_ext_forces, u_k_forces, y_k_forces);
  std::cout << "Output function: " << y_k.transpose() << std::endl;

  Eigen::Matrix<double, 32, 32> Jx;
  robots_object_system_ext_forces->jacobx_state_fcn(x0_ext_forces, u_k_forces, Jx);
  std::cout << "Jacobian of the state function: \n" << Jx << std::endl;

  Eigen::Matrix<double, Eigen::Dynamic, 32> Jy;
  Jy.resize(number_pose_measure_from_robot * 14 + 12, 32);
  robots_object_system_ext_forces->jacobx_output_fcn(x0_ext_forces, u_k_forces, Jy);
  std::cout << "Jacobian of the output function: \n" << Jy << std::endl;



  return 0;
}
