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
This class is a extension of the robots_object_system.hpp file.
To the state has been added the transformation matrix T_hat_.
T_hat_ is the transformation matrix that multiplied by b1Tb2 gives the real transformation matrix
between the two robots. If b1Tb2 stored in robots_object_system_ptr_ is the real one, then
T_hat_ is the identity matrix.
The output is the same as the robots_object_system.hpp file.
*/

#pragma once

#include <uclv_systems_lib/continuous_time/continuous_time_linear_state_space.hpp>
#include "geometry_helper.hpp"
#include <Eigen/Dense>
#include <memory>

namespace uclv::systems
{

template <int number_pose_measure_from_robot>
class RobotsObjectSystemExt : public ContinuousTimeStateSpaceInterface<20, 12, Eigen::Dynamic, 1, 1, 1>
{
public:
  using SharedPtr = std::shared_ptr<RobotsObjectSystemExt>;
  using ConstSharedPtr = std::shared_ptr<const RobotsObjectSystemExt>;
  using WeakPtr = std::weak_ptr<RobotsObjectSystemExt>;
  using ConstWeakPtr = std::weak_ptr<const RobotsObjectSystemExt>;
  using UniquePtr = std::unique_ptr<RobotsObjectSystemExt>;

  RobotsObjectSystemExt(
      const Eigen::Matrix<double, 20, 1>& x0,
      typename uclv::systems::RobotsObjectSystem<number_pose_measure_from_robot>::SharedPtr robots_object_system_ptr)
    : x_(x0), robots_object_system_ptr_(robots_object_system_ptr)
  {
    y_.resize(number_pose_measure_from_robot * 14, 1);
    uclv::geometry_helper::pose_to_matrix(x0.block<7, 1>(13, 0), T_hat_);
  }

  RobotsObjectSystemExt() = default;

  RobotsObjectSystemExt(const RobotsObjectSystemExt& sys) = default;

  virtual ~RobotsObjectSystemExt() = default;

  virtual RobotsObjectSystemExt* clone() const
  {
    return new RobotsObjectSystemExt(*this);
  }

  inline virtual const Eigen::Matrix<double, 20, 1>& get_state() const
  {
    return x_;
  }

  inline virtual const Eigen::Matrix<double, Eigen::Dynamic, 1>& get_output() const
  {
    return y_;
  }

  /*==============================================*/

  /*=============SETTER===========================*/

  inline virtual void set_state(const Eigen::Ref<const Eigen::Matrix<double, 20, 1>>& x)
  {
    x_ = x;
  }

  /*==============================================*/

  /*=============RUNNER===========================*/

  //! State function
  inline virtual void state_fcn(const Eigen::Ref<const Eigen::Matrix<double, 20, 1>>& x,
                                const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                Eigen::Matrix<double, 20, 1>& out) const
  {
    out.setZero();
    // the state is composed by the state of the robots_object_system and the transformation matrix T_hat_
    Eigen::Matrix<double, 13, 1> x_out;
    robots_object_system_ptr_->state_fcn(x.block<13, 1>(0, 0), u_k, x_out);
    out.block<13, 1>(0, 0) = x_out;

    // the transformation matrix T_hat_, stored in the state as position and quaternion is assumed constant, so the
    // derivative is zero and the state is not modified
    out.block<7, 1>(13, 0) = Eigen::Matrix<double, 7, 1>::Zero();
  }

  //! Output function
  inline virtual void output_fcn(const Eigen::Ref<const Eigen::Matrix<double, 20, 1>>& x,
                                 const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                 Eigen::Matrix<double, Eigen::Dynamic, 1>& out) const
  {
    (void)u_k;
    out.resize(number_pose_measure_from_robot * 14, 1);
    out.setZero();

    // the output is composed by the output of the robots_object_system
    robots_object_system_ptr_->output_fcn(x.block<13, 1>(0, 0), u_k, out);

    // update the output coming from the second robot multiplying it by the transformation matrix T_hat_
    Eigen::Matrix<double, 3, 1> b2po = out.block<3, 1>(
        (number_pose_measure_from_robot)*7, 0, 3, 1);  // position of the object in the base frame of the second robot
    Eigen::Quaterniond b2Qo(out(3 + (number_pose_measure_from_robot)*7), out(4 + (number_pose_measure_from_robot)*7),
                            out(5 + (number_pose_measure_from_robot)*7),
                            out(6 + (number_pose_measure_from_robot)*7));  // quaternion of the object in the base frame
                                                                           // of the second robot

    Eigen::Matrix<double, 3, 1> p_hat = x.block<3, 1>(13, 0);
    Eigen::Quaterniond Qhat(x(16), x(17), x(18), x(19));
    Eigen::Matrix<double, 3, 1> b2po_hat = p_hat + Qhat.toRotationMatrix() * b2po;

    Eigen::Quaterniond b2Qo_hat = Qhat * b2Qo;
    b2Qo_hat.normalize();

    for (int i = 0; i < number_pose_measure_from_robot; i++)
    {
      out.block((i + number_pose_measure_from_robot) * 7, 0, 3, 1) = b2po_hat;
      out.block((i + number_pose_measure_from_robot) * 7 + 3, 0, 4, 1) << b2Qo_hat.w(), b2Qo_hat.vec();
    }
  }

  //! Jacobian of the state function with respect to the state
  inline virtual void jacobx_state_fcn(const Eigen::Ref<const Eigen::Matrix<double, 20, 1>>& x,
                                       const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                       Eigen::Matrix<double, 20, 20>& out) const
  {
    // the jacobian of the state function is the jacobian of the robots_object_system
    // the jacobian of the transformation matrix T_hat_ is zero
    out.setZero();
    Eigen::Matrix<double, 13, 13> Jx;
    robots_object_system_ptr_->jacobx_state_fcn(x.block<13, 1>(0, 0), u_k, Jx);
    out.block<13, 13>(0, 0) = Jx;
  }

  //! Jacobian of the output function with respect to the state
  inline virtual void jacobx_output_fcn(const Eigen::Ref<const Eigen::Matrix<double, 20, 1>>& x,
                                        const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                        Eigen::Matrix<double, Eigen::Dynamic, 20>& out) const
  {
    (void)u_k;
    out.resize(number_pose_measure_from_robot * 14, 20);

    out.setZero();
    Eigen::Quaterniond bQo(x(3), x(4), x(5), x(6));
    bQo.normalize();

    // jacobian measures from robot 1
    Eigen::Matrix<double, 4, 4> bTb1 = robots_object_system_ptr_->bTb1_;
    Eigen::Matrix<double, 4, 4> b1Tb = bTb1.inverse();
    Eigen::Matrix<double, 3, 3> Jp_1;  // jacobian position wrt position robot 1

    jacobian_output_to_position(b1Tb, Jp_1);

    Eigen::Matrix<double, 4, 4> JQ_1;  // jacobian quaternion wrt quaternion robot 1
    jacobian_output_to_quaternion_right(bQo, JQ_1);

    Eigen::Matrix<double, 7, 20> output_J1_kth;
    output_J1_kth.setZero();
    output_J1_kth.block(0, 0, 3, 3) = Jp_1;
    output_J1_kth.block(3, 3, 4, 4) = JQ_1;
    output_J1_kth.block(0, 7, 7, 3) = Eigen::Matrix<double, 7, 3>::Zero();
    output_J1_kth.block(0, 10, 7, 3) = Eigen::Matrix<double, 7, 3>::Zero();

    // jacobian measures from robot 2
    Eigen::Matrix<double, 4, 4> T_hat;
    Eigen::Quaterniond Qhat(x(16), x(17), x(18), x(19));
    T_hat.block<3, 1>(0, 3) = x.block<3, 1>(13, 0);
    T_hat.block<3, 3>(0, 0) = Qhat.toRotationMatrix();

    Eigen::Matrix<double, 4, 4> b1Tb2 = robots_object_system_ptr_->b1Tb2_;
    Eigen::Matrix<double, 4, 4> b2Tb_hat = T_hat * b1Tb2.inverse() * b1Tb;
    Eigen::Matrix<double, 3, 3> Jp_2;  // jacobian position wrt position robot 2

    jacobian_output_to_position(b2Tb_hat, Jp_2);

    Eigen::Matrix<double, 4, 4> JQ_2;  // jacobian quaternion wrt quaternion robot 2
    Eigen::Quaterniond bQb1(bTb1.block<3, 3>(0, 0));
    Eigen::Quaterniond b1Qb2(b1Tb2.block<3, 3>(0, 0));
    Eigen::Quaterniond b2Qb_hat = Qhat * b1Qb2.inverse() * bQb1.inverse();
    jacobian_output_to_quaternion_right(b2Qb_hat, JQ_2);

    Eigen::Matrix<double, 3, 4> Jp_Qhat;
    Eigen::Matrix<double, 4, 4> bTo;
    uclv::geometry_helper::pose_to_matrix(x.block<7, 1>(13, 0), bTo);
    Eigen::Matrix<double, 4, 4> b2To = b1Tb2.inverse() * bTb1.inverse() * bTo;
    jacobian_output_position_to_quaternion_hat(b2To, Qhat, Jp_Qhat);

    Eigen::Matrix<double, 4, 4> JQ_2_left;
    Eigen::Quaterniond b2Qo = b1Qb2.inverse() * bQb1.inverse() * bQo;
    jacobian_output_to_quaternion_left(b2Qo, JQ_2_left);

    Eigen::Matrix<double, 7, 20> output_J2_kth;
    output_J2_kth.setZero();
    output_J2_kth.block(0, 0, 3, 3) = Jp_2;
    output_J2_kth.block(3, 3, 4, 4) = JQ_2;
    output_J2_kth.block(0, 7, 7, 3) = Eigen::Matrix<double, 7, 3>::Zero();
    output_J2_kth.block(0, 10, 7, 3) = Eigen::Matrix<double, 7, 3>::Zero();
    output_J2_kth.block(0, 13, 3, 3) =
        Eigen::Matrix<double, 3, 3>::Identity();   // derivative of the output position wrt the position og T_hat
    output_J2_kth.block(0, 16, 3, 4) = Jp_Qhat;    // derivative of the output position wrt the quaternion of T_hat
    output_J2_kth.block(3, 16, 4, 4) = JQ_2_left;  // derivative of the output quaternion wrt the quaternion of T_hat

    for (int i = 0; i < number_pose_measure_from_robot; i++)
    {
      out.block(i * 7, 0, 7, 20) = output_J1_kth;
      out.block((i + number_pose_measure_from_robot) * 7, 0, 7, 20) = output_J2_kth;
    }
  }

  void jacobian_output_to_position(const Eigen::Ref<Eigen::Matrix<double, 4, 4>>& T,
                                   Eigen::Matrix<double, 3, 3>& out) const
  {
    // depends only from the rotation matrix between the base frame and the k-th base frame of the robot
    out.setZero();
    out.block<3, 3>(0, 0) = T.block<3, 3>(0, 0);
  }

  void jacobian_output_position_to_quaternion_hat(const Eigen::Ref<Eigen::Matrix<double, 4, 4>>& T,
                                                  const Eigen::Quaterniond& q, Eigen::Matrix<double, 3, 4>& out) const
  {
    out << 2 * T(1, 3) * q.z() - 2 * T(2, 3) * q.y(), 2 * T(1, 3) * q.y() + 2 * T(2, 3) * q.z(),
        2 * T(1, 3) * q.x() - 2 * T(2, 3) * q.w() - 4 * T(0, 3) * q.y(),
        2 * T(1, 3) * q.w() + 2 * T(2, 3) * q.x() - 4 * T(0, 3) * q.z(), 2 * T(2, 3) * q.x() - 2 * T(0, 3) * q.z(),
        2 * T(2, 3) * q.w() - 4 * T(1, 3) * q.x() + 2 * T(0, 3) * q.y(), 2 * T(0, 3) * q.x() + 2 * T(2, 3) * q.z(),
        2 * T(2, 3) * q.y() - 2 * T(0, 3) * q.w() - 4 * T(1, 3) * q.z(), -2 * T(1, 3) * q.x() - 2 * T(0, 3) * q.y(),
        2 * T(0, 3) * q.z() - 4 * T(2, 3) * q.x() - 2 * T(1, 3) * q.w(),
        2 * T(1, 3) * q.z() - 4 * T(2, 3) * q.y() - 2 * T(0, 3) * q.w(), 2 * T(0, 3) * q.x() + 2 * T(1, 3) * q.y();
  }
  void jacobian_output_to_quaternion_right(const Eigen::Quaterniond& q, Eigen::Matrix<double, 4, 4>& out) const
  {
    // derivative of Q1* Q2 wrt Q2
    out << q.w(), -q.x(), -q.y(), -q.z(), q.x(), q.w(), -q.z(), q.y(), q.y(), q.z(), q.w(), -q.x(), q.z(), -q.y(),
        q.x(), q.w();
  }

  void jacobian_output_to_quaternion_left(const Eigen::Quaterniond& q, Eigen::Matrix<double, 4, 4>& out) const
  {
    // derivative of Q1* Q2 wrt Q1
    out << q.w(), -q.x(), -q.y(), -q.z(), q.x(), q.w(), q.z(), -q.y(), q.y(), -q.z(), q.w(), q.x(), q.z(), q.y(),
        -q.x(), q.w();
  }

  inline virtual void reset()
  {
    x_.setZero();
    y_.setZero();
    T_hat_ = Eigen::Matrix<double, 4, 4>::Identity();
  }

  virtual unsigned int get_size_state() const
  {
    return 20;
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
    std::cout << "RobotsObjectSystemExt" << std::endl;

    std::cout << "State: " << x_.transpose() << std::endl;
    std::cout << "Output: " << y_.transpose() << std::endl;

    std::cout << "T_hat_: \n" << T_hat_ << std::endl;
    robots_object_system_ptr_->display();
  }

private:
  Eigen::Matrix<double, 20, 1> x_;              // state
  Eigen::Matrix<double, Eigen::Dynamic, 1> y_;  // output

  Eigen::Matrix<double, 4, 4> T_hat_;  // Transformation matrix that multiplied by b1Tb2 gives the real transformation
                                       // matrix between the two robots. If b1Tb2 stored in robots_object_system_ptr_ is
                                       // the real one, then T_hat_ is the identity matrix.

  typename uclv::systems::RobotsObjectSystem<number_pose_measure_from_robot>::SharedPtr robots_object_system_ptr_;
};

}  // namespace uclv::systems