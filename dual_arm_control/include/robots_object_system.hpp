/*

    Copyright 2024 Università della Campania Luigi Vanvitelli

    Authors: Marco Costanzo  <marco.costanzo@unicampania.it>
             Marco De Simone <marco.desimone@unicampania.it>

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
*/

/*
 system describing the interaction between 2 robots and one object
 the input to the system are the exerted wrenches by the robots
 applied in the grasp frames and expressed in the same frame
 the output of the system are the measured poses of the object in the
 robots base frame
 the state (13 elements) is represented by the pose of the object in the inertial
 frame an the twist
 pose = [px py pz qx qy qz qw]';
 twist = [vx vy vz omegax omegay omegaz]';
*/

// Comments
// number_pose_measure_from_robot_: number of pose measures from each robot
// the size of the output is number_pose_measure_from_robot_*14, where 14 results from 7 (pose elements) dot 2 (number
// of robots)

#pragma once

#include <uclv_systems_lib/continuous_time/continuous_time_linear_state_space.hpp>
#include "geometry_helper.hpp"
#include <Eigen/Dense>
#include <memory>

namespace uclv::systems
{

  class RobotsObjectSystem : public ContinuousTimeStateSpaceInterface<13, 12, Eigen::Dynamic, 1, 1, 1>
  {
  public:
    using SharedPtr = std::shared_ptr<RobotsObjectSystem>;
    using ConstSharedPtr = std::shared_ptr<const RobotsObjectSystem>;
    using WeakPtr = std::weak_ptr<RobotsObjectSystem>;
    using ConstWeakPtr = std::weak_ptr<const RobotsObjectSystem>;
    using UniquePtr = std::unique_ptr<RobotsObjectSystem>;

    RobotsObjectSystem(const Eigen::Matrix<double, 13, 1> &x0, const Eigen::Matrix<double, 6, 6> &Bm,
                       const Eigen::Matrix<double, 3, 1> &bg, const Eigen::Matrix4d &oTg1, const Eigen::Matrix4d &oTg2,
                       const Eigen::Matrix4d &b1Tb2, const Eigen::Matrix4d &bTb1,
                       const Eigen::Matrix<double, 6, 6> &viscous_friction, const int number_pose_measure_from_robot)
        : x_(x0), Bm_(Bm), bg_(bg), oTg1_(oTg1), oTg2_(oTg2), b1Tb2_(b1Tb2), bTb1_(bTb1), viscous_friction_(viscous_friction), number_pose_measure_from_robot_(number_pose_measure_from_robot)
    {
      update_grasp_matrix();
      update_Rbar();
      y_.resize(number_pose_measure_from_robot_ * 14, 1);
      y_.setZero();
    }

    RobotsObjectSystem() : number_pose_measure_from_robot_(0) {};

    RobotsObjectSystem(const RobotsObjectSystem &sys) = default;

    virtual ~RobotsObjectSystem() = default;

    virtual RobotsObjectSystem *clone() const
    {
      return new RobotsObjectSystem(*this);
    }

    void update_grasp_matrix()
    {
      Eigen::Vector3d opg1 = oTg1_.block<3, 1>(0, 3);
      Eigen::Vector3d opg2 = oTg2_.block<3, 1>(0, 3);
      Eigen::Matrix<double, 6, 6> Wg1;
      Eigen::Matrix<double, 6, 6> Wg2;

      Eigen::Matrix<double, 3, 3> skew_opg1;
      uclv::geometry_helper::skew(opg1, skew_opg1);
      Eigen::Matrix<double, 3, 3> skew_opg2;
      uclv::geometry_helper::skew(opg2, skew_opg2);

      Wg1 << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), skew_opg1, Eigen::Matrix3d::Identity();
      Wg2 << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), skew_opg2, Eigen::Matrix3d::Identity();
      W_ << Wg1, Wg2;
    }

    void update_Rbar()
    {
      Eigen::Matrix3d oRg1 = oTg1_.block<3, 3>(0, 0);
      Eigen::Matrix3d oRg2 = oTg2_.block<3, 3>(0, 0);
      Rbar_ << Eigen::Matrix<double, 12, 12>::Zero();
      Rbar_.block<3, 3>(0, 0) = oRg1;
      Rbar_.block<3, 3>(3, 3) = oRg1;
      Rbar_.block<3, 3>(6, 6) = oRg2;
      Rbar_.block<3, 3>(9, 9) = oRg2;
    }

    inline virtual const Eigen::Matrix<double, 13, 1> &get_state() const
    {
      return x_;
    }

    inline virtual const Eigen::Matrix<double, Eigen::Dynamic, 1> &get_output() const
    {
      return y_;
    }

    /*==============================================*/

    /*=============SETTER===========================*/

    inline virtual void set_state(const Eigen::Ref<const Eigen::Matrix<double, 13, 1>> &x)
    {
      x_ = x;
    }

    /*==============================================*/

    /*=============RUNNER===========================*/

    //! State function
    inline virtual void state_fcn(const Eigen::Ref<const Eigen::Matrix<double, 13, 1>> &x,
                                  const Eigen::Ref<const Eigen::Matrix<double, 12, 1>> &u_k,
                                  Eigen::Matrix<double, 13, 1> &out) const
    {
      Eigen::Quaterniond bQo(x(3), x(4), x(5), x(6));
      bQo.normalize();

      Eigen::Matrix3d bRo = bQo.toRotationMatrix(); // object's rotation matrix in the base frame

      Eigen::Matrix<double, 6, 1> oh = W_ * Rbar_ * u_k; // resulting wrench in the object frame

      out.block<3, 1>(0, 0) = x.block<3, 1>(7, 0);

      Eigen::Quaterniond qdot; // quaternion derivative
      uclv::geometry_helper::quaternion_propagation(bQo, x.block<3, 1>(10, 0), qdot);
      out.block<4, 1>(3, 0) << qdot.w(), qdot.vec();

      Eigen::Matrix<double, 6, 1> air_friction = viscous_friction_ * x.block<6, 1>(7, 0);
      Eigen::Matrix<double, 6, 6> bRo_ext = Eigen::Matrix<double, 6, 6>::Zero();
      bRo_ext.block<3, 3>(0, 0) = bRo;
      bRo_ext.block<3, 3>(3, 3) = bRo;

      Eigen::Matrix<double, 6, 1> bg_ext = Eigen::Matrix<double, 6, 1>::Zero();
      bg_ext.block<3, 1>(0, 0) = bg_;

      out.block<6, 1>(7, 0) = bRo_ext * (Bm_.inverse() * oh) + bg_ext - air_friction;
    }

    //! Output function
    inline virtual void output_fcn(const Eigen::Ref<const Eigen::Matrix<double, 13, 1>> &x,
                                   const Eigen::Ref<const Eigen::Matrix<double, 12, 1>> &u_k,
                                   Eigen::Matrix<double, Eigen::Dynamic, 1> &out) const
    {
      (void)u_k;
      out.resize(number_pose_measure_from_robot_ * 14, 1);
      out.setZero();
      // in the output there are the measrements of the object pose in the base frame of the robots
      Eigen::Quaterniond bQo(x(3), x(4), x(5), x(6));
      bQo.normalize();

      Eigen::Matrix<double, 3, 1> b1po;
      Eigen::Matrix<double, 3, 1> b2po;

      // measured object position in the robots base frame b1 and b2
      b1po = bTb1_.block<3, 3>(0, 0).transpose() * (x.block<3, 1>(0, 0) - bTb1_.block<3, 1>(0, 3));
      b2po = b1Tb2_.block<3, 3>(0, 0).transpose() * (b1po - b1Tb2_.block<3, 1>(0, 3));

      // measured object orientation in the robots base frame b1 and b2
      Eigen::Quaterniond bQb1(bTb1_.block<3, 3>(0, 0));
      Eigen::Quaterniond b1Qb2(b1Tb2_.block<3, 3>(0, 0));
      Eigen::Quaterniond b1Qo = bQb1.inverse() * bQo;
      b1Qo.normalize();
      // Eigen::Quaterniond b2Qo = b1Qb2.inverse() * b1Qo;

      Eigen::Matrix<double, 3,3> b2Rb = b1Tb2_.block<3, 3>(0, 0).transpose() * bTb1_.block<3, 3>(0, 0).transpose();
      Eigen::Quaterniond b2Qb(b2Rb);
      b2Qb.normalize();
      Eigen::Quaterniond b2Qo = b2Qb*bQo;

      b2Qo.normalize();

      
      // out is [b1po;b1Qo;b1po;b1Qo; ... ;b2po;b2Qo; .. ; b2po;b2Qo]
      for (int i = 0; i < number_pose_measure_from_robot_; i++)
      {
        out.block(i * 7, 0, 3, 1) = b1po;
        out.block(i * 7 + 3, 0, 4, 1) << b1Qo.w(), b1Qo.vec();
        out.block((i + number_pose_measure_from_robot_) * 7, 0, 3, 1) = b2po;
        out.block((i + number_pose_measure_from_robot_) * 7 + 3, 0, 4, 1) << b2Qo.w(), b2Qo.vec();
      }
    }

    //! Jacobian of the state function with respect to the state
    inline virtual void jacobx_state_fcn(const Eigen::Ref<const Eigen::Matrix<double, 13, 1>> &x,
                                         const Eigen::Ref<const Eigen::Matrix<double, 12, 1>> &u_k,
                                         Eigen::Matrix<double, 13, 13> &out) const
    {
      out.setZero();
      Eigen::Quaterniond bQo(x(3), x(4), x(5), x(6));
      bQo.normalize();

      // linear velocity term
      out.block<3, 3>(0, 7) = Eigen::Matrix3d::Identity();

      // quaternion dot term
      Eigen::Matrix<double, 4, 4> J_qdot_q; // jacobian of 0.5*E[omega]*Q wrt Q
      jacobian_qdot_q(x.block<3, 1>(10, 0), J_qdot_q);
      Eigen::Matrix<double, 4, 3> J_qdot_omega; // jacobian of 0.5*E[omega]*Q wrt omega
      jacobian_qdot_omega(bQo, J_qdot_omega);

      out.block<4, 4>(3, 3) = J_qdot_q;
      out.block<4, 3>(3, 10) = J_qdot_omega;

      // acceleration term
      Eigen::Matrix<double, 6, 1> oh = W_ * Rbar_ * u_k;
      Eigen::Matrix<double, 6, 4> J_dynamics_q; // jacobian of the dynamics wrt quaternion
      Eigen::Matrix<double, 6, 1> Bm_h = Bm_.inverse() * oh;
      jacob_dynamics_to_quaternion(bQo, Bm_h, J_dynamics_q);

      out.block<6, 4>(7, 3) = J_dynamics_q;
      
      // viscous force term depending from velocity
      out.block<6, 6>(7, 7) = -viscous_friction_;

      // std::cout << "\n jacobian \n" << out;
      // std::cout << "\n Bm_h \n" << Bm_h;
      // std::cout << "\n oh \n" << oh;
      // std::cout << "\n uh \n" << u_k;
      
    }

    void jacobian_qdot_q(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>> &omega,
                         Eigen::Matrix<double, 4, 4> &out) const
    {

      out << 0, -omega(0) / 2, -omega(1) / 2, -omega(2) / 2, omega(0) / 2, 0, -omega(2) / 2, omega(1) / 2, omega(1) / 2,
          omega(2) / 2, 0, -omega(0) / 2, omega(2) / 2, -omega(1) / 2, omega(0) / 2, 0;

    }
    void jacobian_qdot_omega(const Eigen::Quaterniond &q, Eigen::Matrix<double, 4, 3> &out) const
    {
      out << -q.x() / 2, -q.y() / 2, -q.z() / 2,
          q.w() / 2, q.z() / 2, -q.y() / 2,
          -q.z() / 2, q.w() / 2, q.x() / 2,
          q.y() / 2, -q.x() / 2, q.w() / 2;
    }

    void jacob_dynamics_to_quaternion(const Eigen::Quaterniond &q, const Eigen::Ref<const Eigen::Matrix<double, 6, 1>> &a,
                                      Eigen::Matrix<double, 6, 4> &out) const
    {
      //  this function computes the jacobian of the term relative
      //  to the accelerations wrt the quaternion variable
      // the vector "a" should be the result of the product (Bm_\oh)

      out << 4 * a(0) * q.w() + 2 * a(2) * q.y() - 2 * a(1) * q.z(),
          4 * a(0) * q.x() + 2 * a(1) * q.y() + 2 * a(2) * q.z(), 2 * a(2) * q.w() + 2 * a(1) * q.x(),
          2 * a(2) * q.x() - 2 * a(1) * q.w(), 4 * a(1) * q.w() - 2 * a(2) * q.x() + 2 * a(0) * q.z(),
          2 * a(0) * q.y() - 2 * a(2) * q.w(), 2 * a(0) * q.x() + 4 * a(1) * q.y() + 2 * a(2) * q.z(),
          2 * a(0) * q.w() + 2 * a(2) * q.y(), 4 * a(2) * q.w() + 2 * a(1) * q.x() - 2 * a(0) * q.y(),
          2 * a(1) * q.w() + 2 * a(0) * q.z(), 2 * a(1) * q.z() - 2 * a(0) * q.w(),
          2 * a(0) * q.x() + 2 * a(1) * q.y() + 4 * a(2) * q.z(), 4 * a(3) * q.w() + 2 * a(5) * q.y() - 2 * a(4) * q.z(),
          4 * a(3) * q.x() + 2 * a(4) * q.y() + 2 * a(5) * q.z(), 2 * a(5) * q.w() + 2 * a(4) * q.x(),
          2 * a(5) * q.x() - 2 * a(4) * q.w(), 4 * a(4) * q.w() - 2 * a(5) * q.x() + 2 * a(3) * q.z(),
          2 * a(3) * q.y() - 2 * a(5) * q.w(), 2 * a(3) * q.x() + 4 * a(4) * q.y() + 2 * a(5) * q.z(),
          2 * a(3) * q.w() + 2 * a(5) * q.y(), 4 * a(5) * q.w() + 2 * a(4) * q.x() - 2 * a(3) * q.y(),
          2 * a(4) * q.w() + 2 * a(3) * q.z(), 2 * a(4) * q.z() - 2 * a(3) * q.w(),
          2 * a(3) * q.x() + 2 * a(4) * q.y() + 4 * a(5) * q.z();
    }

    //! Jacobian of the output function with respect to the state
    inline virtual void jacobx_output_fcn(const Eigen::Ref<const Eigen::Matrix<double, 13, 1>> &x,
                                          const Eigen::Ref<const Eigen::Matrix<double, 12, 1>> &u_k,
                                          Eigen::Matrix<double, Eigen::Dynamic, 13> &out) const
    {
      (void)u_k;
      out.resize(number_pose_measure_from_robot_ * 14, 13);

      out.setZero();
      Eigen::Quaterniond bQo(x(3), x(4), x(5), x(6));
      bQo.normalize();

      // jacobian measures from robot 1
      Eigen::Matrix<double, 4, 4> b1Tb = bTb1_.inverse();
      Eigen::Matrix<double, 3, 3> Jp_1; // jacobian position wrt position robot 1

      jacobian_output_to_position(b1Tb, Jp_1);

      Eigen::Matrix<double, 4, 4> JQ_1; // jacobian quaternion wrt quaternion robot 1
      jacobian_output_to_quaternion_right(bQo, JQ_1);

      Eigen::Matrix<double, 7, 13> output_J1_kth;
      output_J1_kth.setZero();
      output_J1_kth.block(0, 0, 3, 3) = Jp_1;
      output_J1_kth.block(3, 3, 4, 4) = JQ_1;
      output_J1_kth.block(0, 7, 7, 3) = Eigen::Matrix<double, 7, 3>::Zero();
      output_J1_kth.block(0, 10, 7, 3) = Eigen::Matrix<double, 7, 3>::Zero();

      // jacobian measures from robot 2
      Eigen::Matrix<double, 4, 4> b2Tb = b1Tb2_.inverse() * bTb1_.inverse();
      Eigen::Matrix<double, 3, 3> Jp_2; // jacobian position wrt position robot 2

      jacobian_output_to_position(b2Tb, Jp_2);

      Eigen::Matrix<double, 4, 4> JQ_2; // jacobian quaternion wrt quaternion robot 2
      jacobian_output_to_quaternion_right(bQo, JQ_2);

      Eigen::Matrix<double, 7, 13> output_J2_kth;
      output_J2_kth.setZero();
      output_J2_kth.block(0, 0, 3, 3) = Jp_2;
      output_J2_kth.block(3, 3, 4, 4) = JQ_2;
      output_J2_kth.block(0, 7, 7, 3) = Eigen::Matrix<double, 7, 3>::Zero();
      output_J2_kth.block(0, 10, 7, 3) = Eigen::Matrix<double, 7, 3>::Zero();


      for (int i = 0; i < number_pose_measure_from_robot_; i++)
      {
        out.block(i * 7, 0, 7, 13) = output_J1_kth;
        out.block((i + number_pose_measure_from_robot_) * 7, 0, 7, 13) = output_J2_kth;
      }
    }

    void jacobian_output_to_position(const Eigen::Ref<Eigen::Matrix<double, 4, 4>> &T,
                                     Eigen::Matrix<double, 3, 3> &out) const
    {
      // depends only from the rotation matrix between the base frame and the k-th base frame of the robot
      out.setZero();
      out.block<3, 3>(0, 0) = T.block<3, 3>(0, 0);
    }
    void jacobian_output_to_quaternion_right(const Eigen::Quaterniond &q, Eigen::Matrix<double, 4, 4> &out) const
    {
      // derivative of Q1* Q2 wrt Q2
      out << q.w(), -q.x(), -q.y(), -q.z(), q.x(), q.w(), -q.z(), q.y(), q.y(), q.z(), q.w(), -q.x(), q.z(), -q.y(),
          q.x(), q.w();
    }

    inline virtual void reset()
    {
      x_.setZero();
      y_.setZero();
    }

    virtual unsigned int get_size_state() const
    {
      return 13;
    }

    virtual unsigned int get_size1_state() const
    {
      return 1;
    }

    virtual unsigned int get_size2_state() const
    {
      return 1;
    }

    virtual void display() const
    {
      std::cout << "RobotsObjectSystem" << std::endl;

      std::cout << "State: " << x_.transpose() << std::endl;
      std::cout << "Output: " << y_.transpose() << std::endl;

      std::cout << "Inertia matrix: \n"
                << Bm_ << std::endl;
      std::cout << "Gravity vector: \n"
                << bg_.transpose() << std::endl;
      std::cout << "Grasp matrix: \n"
                << W_ << std::endl;
      std::cout << "Extended Rotation matrix: \n"
                << Rbar_ << std::endl;
      std::cout << "Transformation matrix from object to grasp 1: \n"
                << oTg1_ << std::endl;
      std::cout << "Transformation matrix from object to grasp 2: \n"
                << oTg2_ << std::endl;
      std::cout << "Transformation matrix from base 1 to base 2 of robots: \n"
                << b1Tb2_ << std::endl;
      std::cout << "Transformation matrix from base to base 1: \n"
                << bTb1_ << std::endl;
      std::cout << "Viscous friction matrix: \n"
                << viscous_friction_ << std::endl;
    }

  public:
    Eigen::Matrix<double, 13, 1> x_;             // state
    Eigen::Matrix<double, Eigen::Dynamic, 1> y_; // output

    Eigen::Matrix<double, 6, 6> Bm_;               // Inertia matrix
    Eigen::Matrix<double, 3, 1> bg_;               // Gravity vector
    Eigen::Matrix<double, 6, 12> W_;               // Grasp matrix
    Eigen::Matrix<double, 12, 12> Rbar_;           // Extended Rotation matrix
    Eigen::Matrix4d oTg1_;                         // Transformation matrix from object to grasp 1
    Eigen::Matrix4d oTg2_;                         // Transformation matrix from object to grasp 2
    Eigen::Matrix4d b1Tb2_;                        // Transformation matrix from base 1 to base 2 of robots (could be unknown)
    Eigen::Matrix4d bTb1_;                         // Transformation matrix from base to base 1  (well known)
    Eigen::Matrix<double, 6, 6> viscous_friction_; // Viscous friction matrix of the object-air

    const int number_pose_measure_from_robot_;
  };

} // namespace uclv::systems