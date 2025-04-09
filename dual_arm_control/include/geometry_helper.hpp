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
  }

  void pose_to_matrix(const Eigen::Matrix<double, 7, 1> &pose, Eigen::Matrix<double, 4, 4> &T)
  {
    Eigen::Quaterniond q(pose(3), pose(4), pose(5), pose(6));
    q.normalize();
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
    T.block<3, 1>(0, 3) = pose.block<3, 1>(0, 0);
    T.block<1, 4>(3, 0) << 0, 0, 0, 1;
  }

  void quaternion_continuity(const Eigen::Matrix<double, 4, 1> &qnew,
                             const Eigen::Matrix<double, 4, 1> &qold, Eigen::Matrix<double, 4, 1> &q)
  {
    q = qnew;
    double tmp = qnew.block<3, 1>(1, 0).transpose() * qold.block<3, 1>(1, 0);
    if (tmp < -0.01)
    {
      q = -q;
    }
  }

  void quaternion_continuity( const Eigen::Quaterniond &qnew, const Eigen::Quaterniond &qold, Eigen::Quaterniond &q)
  {
    Eigen::Matrix<double, 4, 1> qnew_mat;
    qnew_mat << qnew.w(), qnew.x(), qnew.y(), qnew.z();

    Eigen::Matrix<double, 4, 1> qold_mat;
    qold_mat << qold.w(), qold.x(), qold.y(), qold.z();

    Eigen::Matrix<double, 4, 1> q_mat;

    quaternion_continuity(qnew_mat, qold_mat, q_mat);

    Eigen::Quaterniond qtmp(q_mat[0], q_mat[1], q_mat[2], q_mat[3]);
    qtmp.normalize();
    q = qtmp;
  }

} // namespace uclv::geometry_helper
