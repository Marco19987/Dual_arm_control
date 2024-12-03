#pragma once

#include <Eigen/Dense>
#include <iostream>
namespace uclv::geometry_helper
{
  void skew(const Eigen::Vector3d &v, Eigen::Matrix<double, 3, 3> &M)
  {
    M << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  }

  void quaternion_propagation(const Eigen::Quaterniond &q, const Eigen::Vector3d &omega, Eigen::Quaterniond &qdot)
  {
    Eigen::Vector3d epsilon_q = q.vec();
    double eta = q.w();
    double eta_dot = -0.5 * epsilon_q.dot(omega);
    Eigen::Matrix<double, 3, 3> skew_epsilon_q;
    skew(epsilon_q, skew_epsilon_q);
    Eigen::Vector3d epsilon_dot = 0.5 * (eta * Eigen::Matrix3d::Identity() - skew_epsilon_q) * omega;
    qdot.w() = eta_dot;
    qdot.vec() = epsilon_dot;

    // Eigen::Quaterniond qdot_(eta_dot, epsilon_dot(0), epsilon_dot(1), epsilon_dot(2));
    // qdot = qdot_;

  }

  void pose_to_matrix(const Eigen::Matrix<double, 7, 1> &pose, Eigen::Matrix<double, 4, 4> &T)
  {
    Eigen::Quaterniond q(pose(3), pose(4), pose(5), pose(6));
    q.normalize();
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
    T.block<3, 1>(0, 3) = pose.block<3, 1>(0, 0);
    T.block<1, 4>(3, 0) << 0, 0, 0, 1;
  } // namespace uclv::geometry_helper

  void quaternion_continuity(const Eigen::Ref<const Eigen::Matrix<double, 4, 1>> &qnew,
                             const Eigen::Ref<const Eigen::Matrix<double, 4, 1>> &qold, Eigen::Matrix<double, 4, 1> &q)
  {
    q = qnew;
    double tmp = qnew.block<3, 1>(1, 0).transpose() * qold.block<3, 1>(1, 0);
    if (tmp < -0.01)
    {
      q = -q;
    }
  }

} // namespace uclv::geometry_helper
