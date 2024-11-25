#include <Eigen/Dense>
#include <iostream>
namespace uclv::geometry_helper
{
void skew(const Eigen::Vector3d& v, Eigen::Matrix<double, 3, 3>& M)
{
  M << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
}

void quaternion_propagation(const Eigen::Quaterniond& q, const Eigen::Vector3d& omega, Eigen::Quaterniond& qdot)
{
  Eigen::Vector3d epsilon = q.vec();
  double eta = q.w();
  double eta_dot = -0.5 * epsilon.dot(omega);
  Eigen::Vector3d epsilon_dot = 0.5 * (eta * Eigen::Matrix3d::Identity() - epsilon * epsilon.transpose()) * omega;
  qdot.w() = eta_dot;
  qdot.vec() = epsilon_dot;
}
}  // namespace uclv::geometry_helper
