#include <uclv_systems_lib/controllers/pi.hpp>
#include <uclv_systems_lib/sim/state_space_system_simulator.hpp>
#include <uclv_systems_lib/ss/linear_state_space.hpp>
#include <uclv_systems_lib/continuous_time/continuous_time_linear_state_space.hpp>
#include <uclv_systems_lib/discretization/forward_euler.hpp>
#include <uclv_systems_lib/observers/ekf.hpp>
#include "../include/robots_spring_object_system.hpp"
#include "../include/robots_spring_object_system_ext.hpp"
#include "../include/robots_spring_object_system_ext_grasp.hpp"

int main()
{
  const int number_pose_measure_from_robot = 2;
  Eigen::Matrix<double, 27, 1> x0;  // Initial state
  Eigen::Matrix<double, 6, 6> Bm;   // Inertia matrix
  Eigen::Matrix<double, 3, 1> bg;   // Gravity vector
  Eigen::Matrix4d oTg1;             // Transformation matrix from object to grasp 1
  Eigen::Matrix4d oTg2;             // Transformation matrix from object to grasp 2
  Eigen::Matrix4d b2Tb1;            // Transformation matrix from base 1 to base 2 of robots (could be unknown)
  Eigen::Matrix4d bTb1;             // Transformation matrix from base to base 1  (well known)
  Eigen::Matrix<double, 6, 6> viscous_friction;  // Viscous friction matrix of the object-air
  Eigen::Matrix<double, 6, 6> K_1;
  Eigen::Matrix<double, 6, 6> B_1;
  Eigen::Matrix<double, 6, 6> K_2;
  Eigen::Matrix<double, 6, 6> B_2;

  x0 << 0.0, 0.0, 0.1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
  Bm = Eigen::Matrix<double, 6, 6>::Zero();
  Bm.diagonal() << 0.280, 0.280, 0.280, 0.1, 0.1, 0.1;
  bg << 0, 0, -9.81;
  oTg1 << Eigen::Matrix4d::Identity();
  oTg1.block<3, 1>(0, 3) << 0.1, 0.1, 0.1;
  oTg2 << Eigen::Matrix4d::Identity();
  oTg2.block<3, 1>(0, 3) << -0.1, -0.2, -0.3;
  oTg2.block<3, 3>(0, 0) << -Eigen::Matrix3d::Identity();
  b2Tb1 << Eigen::Matrix4d::Identity();
  b2Tb1.block<3, 1>(0, 3) << 0.0, 0.0, 0.1;
  bTb1 << Eigen::Matrix4d::Identity();
  viscous_friction = Eigen::Matrix<double, 6, 6>::Zero();
  viscous_friction.diagonal() << 0.0001,0.0001,0.0001,0.0001,0.0001,0.0001;
  K_1 = Eigen::Matrix<double, 6, 6>::Zero();
  K_1.diagonal() << 1000.0, 1000.0, 1000.0, 1.0, 1.0, 1.0;
  K_2 = Eigen::Matrix<double, 6, 6>::Zero();
  K_2.diagonal() << 1000.0, 1000.0, 1000.0, 1.0, 1.0, 1.0;
  B_1 = Eigen::Matrix<double, 6, 6>::Zero();
  B_1.diagonal() << 100.0, 100.0, 100.0, 1.0, 1.0, 1.0;
  B_2 = Eigen::Matrix<double, 6, 6>::Zero();
  B_2.diagonal() << 100.0, 100.0, 100.0, 1.0, 1.0, 1.0;

  auto robots_object_system = std::make_shared<uclv::systems::RobotsSpringObjectSystem>(
      x0, Bm, bg, oTg1, oTg2, b2Tb1, bTb1, viscous_friction, K_1, B_1, K_2, B_2, number_pose_measure_from_robot);

  Eigen::Matrix<double, 34, 1> x0_ext;
  x0_ext << x0, 0, 0, 0, 1, 0, 0, 0;
  auto robots_object_system_ext =
      std::make_shared<uclv::systems::RobotsSpringObjectSystemExt>(x0_ext, robots_object_system);

  Eigen::Matrix<double, 48, 1> x0_ext_grasp;
  x0_ext_grasp << x0_ext, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
  x0_ext_grasp.block<3, 1>(34, 0) << oTg1(0, 3), oTg1(1, 3), oTg1(2, 3);
  Eigen::Quaterniond oQg1(oTg1.block<3, 3>(0, 0));
  oQg1.normalize();
  x0_ext_grasp.block<4, 1>(37, 0) << oQg1.w(), oQg1.vec();
  x0_ext_grasp.block<3, 1>(41, 0) << oTg2(0, 3), oTg2(1, 3), oTg2(2, 3);
  Eigen::Quaterniond oQg2(oTg2.block<3, 3>(0, 0));
  oQg2.normalize();
  x0_ext_grasp.block<4, 1>(44, 0) << oQg2.w(), oQg2.vec();

  auto robots_object_system_ext_grasp =
      std::make_shared<uclv::systems::RobotsSpringObjectSystemExtGrasp>(x0_ext_grasp, robots_object_system_ext);

  robots_object_system_ext_grasp->display();

  Eigen::Matrix<double, 48, 1> x;
  Eigen::Matrix<double, 12, 1> u_k;
  Eigen::Matrix<double, Eigen::Dynamic, 1> y_k;
  y_k.resize(number_pose_measure_from_robot * 14 + 12 + 14, 1);
  u_k << 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  robots_object_system_ext_grasp->state_fcn(x0_ext_grasp, u_k, x);
  std::cout << "State function: " << x.transpose() << std::endl;

  robots_object_system_ext_grasp->output_fcn(x0_ext_grasp, u_k, y_k);
  std::cout << "Output function: " << y_k.transpose() << std::endl;

  Eigen::Matrix<double, 48, 48> Jx;
  robots_object_system_ext_grasp->jacobx_state_fcn(x0_ext_grasp, u_k, Jx);
  std::cout << "Jacobian of the state function: \n" << Jx << std::endl;

  Eigen::Matrix<double, Eigen::Dynamic, 48> Jy;
  Jy.resize(number_pose_measure_from_robot * 14 + 12 + 14, 48);
  robots_object_system_ext_grasp->jacobx_output_fcn(x0_ext_grasp, u_k, Jy);
  std::cout << "Jacobian of the output function: \n" << Jy << std::endl;

  return 0;
}
