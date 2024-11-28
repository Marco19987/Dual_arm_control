#include <uclv_systems_lib/controllers/pi.hpp>
#include <uclv_systems_lib/sim/state_space_system_simulator.hpp>
#include <uclv_systems_lib/ss/linear_state_space.hpp>
#include <uclv_systems_lib/continuous_time/continuous_time_linear_state_space.hpp>
#include <uclv_systems_lib/discretization/forward_euler.hpp>
#include <uclv_systems_lib/observers/ekf.hpp>
#include "../include/robots_object_system.hpp"
#include "../include/robots_object_system_ext.hpp"

int main()
{
  const int number_pose_measure_from_robot = 3;
  const int dim_state = 20;
  const int dim_input = 12;
  const int dim_output = number_pose_measure_from_robot * 14;
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
  bg << 0, 0, -9.81*0;
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
  x0_ext << x0, 0.3,0.2,0.1,1,0,0,0;
  auto robots_object_system_ext = std::make_shared<uclv::systems::RobotsObjectSystemExt>(x0_ext, robots_object_system);
  robots_object_system_ext->display();

  // Create the Forward Euler discretized system
  auto discretized_system =
      std::make_shared<uclv::systems::ForwardEuler<dim_state, dim_input, Eigen::Dynamic>>(robots_object_system_ext, 0.1);
  discretized_system->set_state(x0_ext);
  discretized_system->display();

  // extended kalman filter
  // define W and V covariance matrices
  Eigen::Matrix<double, dim_state, dim_state> W = Eigen::Matrix<double, dim_state, dim_state>::Identity() * 0.001;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V;  // Eigen::Matrix<double, dim_output, dim_output>::Identity()
                                                            // * 0.001;
  V.resize(dim_output, dim_output);
  V.setIdentity();

  // start from a different initial state
  Eigen::Matrix<double, dim_state, 1> x0_hat;
  x0_hat << 0.0, 0.0, 0.0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;

  uclv::systems::ExtendedKalmanFilter<dim_state, dim_input, Eigen::Dynamic> ekf(discretized_system, W, V);
  ekf.set_state(x0_hat);

  // simulation of the system and the observer
  Eigen::Matrix<double, dim_state, 1> x_hat_k_k;
  Eigen::Matrix<double, dim_output, 1> y_hat_k;
  uclv::systems::StateSpaceSystemSimulator<dim_state, dim_input, Eigen::Dynamic> simulator(discretized_system);

  for (int i = 0; i < 200; i++)
  {
    simulator.simulate(u_k);
    y_k = discretized_system->get_output();
    ekf.kf_apply(u_k, y_k, W, V);
    x_hat_k_k = ekf.get_state();
    y_hat_k = ekf.get_output();

    std::cout << "--------------!\n" << std::endl;
    std::cout << "x_k: " << discretized_system->get_state().transpose() << std::endl;
    std::cout << "x_hat_k_k: " << x_hat_k_k.transpose() << std::endl;
    // std::cout << "y_k: " << y_k.transpose() << std::endl;
    // std::cout << "y_hat_k: " << y_hat_k.transpose() << std::endl;
  }

  ekf.display();

  return 0;
}
