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
To the state has been added the transformation matrix b2Tb1_.
b2Tb1_ is the transformation matrix from the base frame of the second robot to the base frame of the first robot.
The output is the same as the robots_object_system.hpp file.
*/

#pragma once

#include <uclv_systems_lib/continuous_time/continuous_time_linear_state_space.hpp>
#include "geometry_helper.hpp"
#include <Eigen/Dense>
#include <memory>
#include "jacobian_h_to_xstate/jacobian_h_to_b2Tb1.h"
#include "jacobian_h_to_xstate/jacobian_h_to_b2Tb1.cpp"

namespace uclv::systems
{

class RobotsSpringObjectSystemExt : public ContinuousTimeStateSpaceInterface<34, 12, Eigen::Dynamic, 1, 1, 1>
{
public:
  using SharedPtr = std::shared_ptr<RobotsSpringObjectSystemExt>;
  using ConstSharedPtr = std::shared_ptr<const RobotsSpringObjectSystemExt>;
  using WeakPtr = std::weak_ptr<RobotsSpringObjectSystemExt>;
  using ConstWeakPtr = std::weak_ptr<const RobotsSpringObjectSystemExt>;
  using UniquePtr = std::unique_ptr<RobotsSpringObjectSystemExt>;

  RobotsSpringObjectSystemExt(const Eigen::Matrix<double, 34, 1>& x0,
                              typename uclv::systems::RobotsSpringObjectSystem::SharedPtr robots_object_system_ptr)
    : x_(x0)
    , robots_object_system_ptr_(robots_object_system_ptr)
    , number_pose_measure_from_robot_(robots_object_system_ptr->number_pose_measure_from_robot_)
  {
    y_.resize(number_pose_measure_from_robot_ * 14 + 12 + 14, 1);
    y_.setZero();
  }

  RobotsSpringObjectSystemExt() = default;

  RobotsSpringObjectSystemExt(const RobotsSpringObjectSystemExt& sys) = default;

  virtual ~RobotsSpringObjectSystemExt() = default;

  virtual RobotsSpringObjectSystemExt* clone() const
  {
    return new RobotsSpringObjectSystemExt(*this);
  }

  inline virtual const Eigen::Matrix<double, 34, 1>& get_state() const
  {
    return x_;
  }

  inline virtual const Eigen::Matrix<double, Eigen::Dynamic, 1>& get_output() const
  {
    return y_;
  }

  /*==============================================*/

  /*=============SETTER===========================*/

  inline virtual void set_state(const Eigen::Ref<const Eigen::Matrix<double, 34, 1>>& x)
  {
    x_ = x;
  }

  /*==============================================*/

  /*=============RUNNER===========================*/

  //! State function
  inline virtual void state_fcn(const Eigen::Ref<const Eigen::Matrix<double, 34, 1>>& x,
                                const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                Eigen::Matrix<double, 34, 1>& out) const
  {
    out.setZero();

    Eigen::Matrix<double, 4, 4> b2Tb1;
    uclv::geometry_helper::pose_to_matrix(x.block<7, 1>(27, 0), b2Tb1);
    robots_object_system_ptr_->set_b2Tb1(b2Tb1);

    // the state is composed by the state of the robots_object_system and the transformation matrix b2Tb1_
    Eigen::Matrix<double, 27, 1> x_out;
    robots_object_system_ptr_->state_fcn(x.block<27, 1>(0, 0), u_k, x_out);
    out.block<27, 1>(0, 0) = x_out;

    // the transformation matrix b2Tb1_, stored in the state as position and quaternion is assumed constant, so the
    // derivative is zero and the state is not modified
    out.block<7, 1>(27, 0) = Eigen::Matrix<double, 7, 1>::Zero();
  }

  //! Output function
  inline virtual void output_fcn(const Eigen::Ref<const Eigen::Matrix<double, 34, 1>>& x,
                                 const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                 Eigen::Matrix<double, Eigen::Dynamic, 1>& out) const
  {
    (void)u_k;
    out.resize(number_pose_measure_from_robot_ * 14 + 12 + 14, 1);
    out.setZero();

    // update b2Tb1 in the robots_object_system
    Eigen::Matrix<double, 4, 4> b2Tb1;
    uclv::geometry_helper::pose_to_matrix(x.block<7, 1>(27, 0), b2Tb1);
    robots_object_system_ptr_->set_b2Tb1(b2Tb1);

    // the output is composed by the output of the robots_object_system
    robots_object_system_ptr_->output_fcn(x.block<27, 1>(0, 0), u_k, out);
  }

  //! Jacobian of the state function with respect to the state
  inline virtual void jacobx_state_fcn(const Eigen::Ref<const Eigen::Matrix<double, 34, 1>>& x,
                                       const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                       Eigen::Matrix<double, 34, 34>& out) const
  {
    // the jacobian of the state function is the jacobian of the robots_object_system
    // the jacobian of the transformation matrix b2Tb1_ is zero
    out.setZero();

    // update b2Tb1 in the robots_object_system
    Eigen::Matrix<double, 4, 4> b2Tb1;
    uclv::geometry_helper::pose_to_matrix(x.block<7, 1>(27, 0), b2Tb1);
    robots_object_system_ptr_->set_b2Tb1(b2Tb1);

    Eigen::Matrix<double, 27, 27> Jx;
    robots_object_system_ptr_->jacobx_state_fcn(x.block<27, 1>(0, 0), u_k, Jx);
    out.block<27, 27>(0, 0) = Jx;
    // Add jacobian h to b2Tb1 to the twist term
    Eigen::Matrix<double, 12, 7> J_h_b2Tb1;
    get_jacobian_h_to_b2Tb1(x, u_k, J_h_b2Tb1);

    out.block<6, 7>(7, 27) = robots_object_system_ptr_->Bm_.inverse() *
                             (robots_object_system_ptr_->W_ * robots_object_system_ptr_->Rbar_ * J_h_b2Tb1);

    // debug print jacobian out
    // std::cout << "PRINT JACOBIAN ELEMENTS: \n"<< std::endl;
    // std::cout << "\n J_bpo_dot \n"<< out.block<3,34>(0, 0) << std::endl;
    // std::cout << "\n J_bQo_dot \n"<< out.block<4,34>(3, 0) << std::endl;
    // std::cout << "\n J_otwisto_dot \n"<< out.block<6,34>(7, 0) << std::endl;
    // std::cout << "\n J_b1pe1_dot \n"<< out.block<3,34>(13, 0) << std::endl;
    // std::cout << "\n J_b1Qe1_dot \n"<< out.block<4,34>(16, 0) << std::endl;
    // std::cout << "\n J_b2pe2_dot \n"<< out.block<3,34>(20, 0) << std::endl;
    // std::cout << "\n J_b2Qe2_dot \n"<< out.block<4,34>(23, 0) << std::endl;
    // std::cout << "\n J_h_b2Tb1 \n"<< J_h_b2Tb1 << std::endl;
  }

  //! Jacobian of the output function with respect to the state
  inline virtual void jacobx_output_fcn(const Eigen::Ref<const Eigen::Matrix<double, 34, 1>>& x,
                                        const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                        Eigen::Matrix<double, Eigen::Dynamic, 34>& out) const
  {
    (void)u_k;
    out.resize(number_pose_measure_from_robot_ * 14 + 12 + 14, 34);
    out.setZero();

    // update b2Tb1 in the robots_object_system
    Eigen::Matrix<double, 4, 4> b2Tb1;
    uclv::geometry_helper::pose_to_matrix(x.block<7, 1>(27, 0), b2Tb1);
    robots_object_system_ptr_->set_b2Tb1(b2Tb1);

    Eigen::Quaterniond bQo(x(3), x(4), x(5), x(6));
    bQo.normalize();

    Eigen::Quaterniond b2Qb1(x(30), x(31), x(32), x(33));
    b2Qb1.normalize();

    // jacobian measures from robot 1
    Eigen::Matrix<double, 4, 4> bTb1 = robots_object_system_ptr_->bTb1_;
    Eigen::Quaterniond bQb1(bTb1.block<3, 3>(0, 0));
    Eigen::Quaterniond b1Qo = bQb1.inverse() * bQo;
    b1Qo.normalize();

    Eigen::Matrix<double, Eigen::Dynamic, 27> jacob_base;
    robots_object_system_ptr_->jacobx_output_fcn(x.block<27, 1>(0, 0), u_k, jacob_base);
    out.block(0, 0, number_pose_measure_from_robot_ * 14 + 12 + 14, 27) = jacob_base;

    // jacobian measures from robot 2
    Eigen::Matrix<double, 4, 4> bTo;
    uclv::geometry_helper::pose_to_matrix(x.block<7, 1>(0, 0), bTo);
    Eigen::Matrix<double, 4, 4> b1To = bTb1 * bTo;

    Eigen::Matrix<double, 3, 3> J_b2po_b2pb1;
    J_b2po_b2pb1 = Eigen::Matrix<double, 3, 3>::Identity();

    Eigen::Matrix<double, 3, 4> J_b2po_b2Qb1;
    Jacobian_Rp_to_Q(b2Qb1, b1To.block<3, 1>(0, 3), J_b2po_b2Qb1);

    Eigen::Matrix<double, 4, 4> J_b2Qo_b2Qb1;
    robots_object_system_ptr_->jacobian_output_to_quaternion_left(b1Qo, J_b2Qo_b2Qb1);

    Eigen::Matrix<double, 7, 7> output_J2_kth;
    output_J2_kth.setZero();
    output_J2_kth.block(0, 0, 3, 3) = J_b2po_b2pb1;
    output_J2_kth.block(0, 3, 3, 4) = J_b2po_b2Qb1;
    output_J2_kth.block(3, 3, 4, 4) = J_b2Qo_b2Qb1;

    for (int i = 0; i < number_pose_measure_from_robot_; i++)
    {
      out.block((i + number_pose_measure_from_robot_) * 7, 27, 7, 7) = output_J2_kth;
    }

    // Add the jacobian wrench spring to the output jacobian
    Eigen::Matrix<double, 12, 7> J_h_b2Tb1;
    get_jacobian_h_to_b2Tb1(x, u_k, J_h_b2Tb1);
    out.block(number_pose_measure_from_robot_ * 14, 27, 12, 7) = J_h_b2Tb1;
  }

  void Jacobian_Rp_to_Q(const Eigen::Quaterniond& q, const Eigen::Matrix<double, 3, 1> p,
                        Eigen::Matrix<double, 3, 4>& out) const
  {
    double t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13;
    double Q1 = q.w(), Q2 = q.x(), Q3 = q.y(), Q4 = q.z();
    double p1 = p(0), p2 = p(1), p3 = p(2);

    t2 = Q1 * p1 * 2.0;
    t3 = Q1 * p2 * 2.0;
    t4 = Q2 * p1 * 2.0;
    t5 = Q1 * p3 * 2.0;
    t6 = Q2 * p2 * 2.0;
    t7 = Q3 * p1 * 2.0;
    t8 = Q2 * p3 * 2.0;
    t9 = Q3 * p2 * 2.0;
    t10 = Q4 * p1 * 2.0;
    t11 = Q3 * p3 * 2.0;
    t12 = Q4 * p2 * 2.0;
    t13 = Q4 * p3 * 2.0;

    out(0, 0) = t11 - t12;
    out(0, 1) = t9 + t13;
    out(0, 2) = t5 + t6 - Q3 * p1 * 4.0;
    out(0, 3) = -t3 + t8 - Q4 * p1 * 4.0;
    out(1, 0) = -t8 + t10;
    out(1, 1) = -t5 + t7 - Q2 * p2 * 4.0;
    out(1, 2) = t4 + t13;
    out(1, 3) = t2 + t11 - Q4 * p2 * 4.0;
    out(2, 0) = t6 - t7;
    out(2, 1) = t3 + t10 - Q2 * p3 * 4.0;
    out(2, 2) = -t2 + t12 - Q3 * p3 * 4.0;
    out(2, 3) = t4 + t9;
  }

  inline virtual void get_jacobian_h_to_b2Tb1(const Eigen::Ref<const Eigen::Matrix<double, 34, 1>>& x,
                                              const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                              Eigen::Matrix<double, 12, 7>& out) const
  {
    Eigen::Matrix<double, 3, 1> bpo_b = x.block<3, 1>(0, 0);     // object's position in the base frame
    Eigen::Quaterniond bQo(x(3), x(4), x(5), x(6));              // object's quaternion in the base frame
    Eigen::Matrix<double, 3, 1> ovo = x.block<3, 1>(7, 0);       // object's linear velocity in the object frame
    Eigen::Matrix<double, 3, 1> oomegao = x.block<3, 1>(10, 0);  // object's angular velocity in the object frame
    Eigen::Matrix<double, 3, 1> b1pe1_b1 =
        x.block<3, 1>(13, 0);                              // robot 1 end-effector position in the robot 1 base frame
    Eigen::Quaterniond b1Qe1(x(16), x(17), x(18), x(19));  // robot 1 end-effector quaternion in the robot 1 base frame
    Eigen::Matrix<double, 3, 1> b2pe2_b2 =
        x.block<3, 1>(20, 0);                              // robot 2 end-effector position in the robot 2 base frame
    Eigen::Quaterniond b2Qe2(x(23), x(24), x(25), x(26));  // robot 2 end-effector quaternion in the robot 2 base frame

    Eigen::Matrix<double, 3, 1> b2pb1 = x.block<3, 1>(27, 0);
    Eigen::Quaterniond b2Qb1(x(30), x(31), x(32), x(33));

    Eigen::Matrix<double, 3, 1> b1pe1_dot =
        u_k.block<3, 1>(0, 0);  // robot 1 end-effector linear velocity in the robot 1 base frame
    Eigen::Matrix<double, 3, 1> b1_omega_e1 =
        u_k.block<3, 1>(3, 0);  // robot 1 end-effector angular velocity in the robot 1 base frame
    Eigen::Matrix<double, 3, 1> b2pe2_dot =
        u_k.block<3, 1>(6, 0);  // robot 2 end-effector linear velocity in the robot 2 base frame
    Eigen::Matrix<double, 3, 1> b2_omega_e2 =
        u_k.block<3, 1>(9, 0);  // robot 2 end-effector angular velocity in the robot 2 base frame

    Eigen::Matrix<double, 3, 1> bpb1 = robots_object_system_ptr_->bTb1_.block<3, 1>(0, 3);
    Eigen::Quaterniond bQb1(robots_object_system_ptr_->bTb1_.block<3, 3>(0, 0));

    Eigen::Matrix<double, 3, 1> opg1 = robots_object_system_ptr_->oTg1_.block<3, 1>(0, 3);
    Eigen::Quaterniond oQg1(robots_object_system_ptr_->oTg1_.block<3, 3>(0, 0));
    Eigen::Matrix<double, 3, 1> opg2 = robots_object_system_ptr_->oTg2_.block<3, 1>(0, 3);
    Eigen::Quaterniond oQg2(robots_object_system_ptr_->oTg2_.block<3, 3>(0, 0));

    Eigen::Matrix<double, 6, 1> K_1_diag = robots_object_system_ptr_->K_1_.diagonal();
    Eigen::Matrix<double, 6, 1> B_1_diag = robots_object_system_ptr_->B_1_.diagonal();
    Eigen::Matrix<double, 6, 1> K_2_diag = robots_object_system_ptr_->K_2_.diagonal();
    Eigen::Matrix<double, 6, 1> B_2_diag = robots_object_system_ptr_->B_2_.diagonal();

    double in1[3], in2[4], in3[3], in4[3], in5[3], in6[4], in7[3], in8[4], in9[3], in10[4];
    double in11[3], in12[3], in13[3], in14[3], in15[3], in16[4], in17[3], in18[4], in19[3], in20[4];
    double in21[6], in22[6], in23[6], in24[6];
    double b_jacobian_h_to_x_state_not_ext[84];

    Eigen::Map<Eigen::Matrix<double, 3, 1>>(in1) << bpo_b;
    Eigen::Map<Eigen::Matrix<double, 4, 1>>(in2) << bQo.w(), bQo.x(), bQo.y(), bQo.z();
    Eigen::Map<Eigen::Matrix<double, 3, 1>>(in3) << ovo;
    Eigen::Map<Eigen::Matrix<double, 3, 1>>(in4) << oomegao;
    Eigen::Map<Eigen::Matrix<double, 3, 1>>(in5) << b1pe1_b1;
    Eigen::Map<Eigen::Matrix<double, 4, 1>>(in6) << b1Qe1.w(), b1Qe1.x(), b1Qe1.y(), b1Qe1.z();
    Eigen::Map<Eigen::Matrix<double, 3, 1>>(in7) << b2pe2_b2;
    Eigen::Map<Eigen::Matrix<double, 4, 1>>(in8) << b2Qe2.w(), b2Qe2.x(), b2Qe2.y(), b2Qe2.z();
    Eigen::Map<Eigen::Matrix<double, 3, 1>>(in9) << b2pb1;
    Eigen::Map<Eigen::Matrix<double, 4, 1>>(in10) << b2Qb1.w(), b2Qb1.x(), b2Qb1.y(), b2Qb1.z();
    Eigen::Map<Eigen::Matrix<double, 3, 1>>(in11) << b1pe1_dot;
    Eigen::Map<Eigen::Matrix<double, 3, 1>>(in12) << b1_omega_e1;
    Eigen::Map<Eigen::Matrix<double, 3, 1>>(in13) << b2pe2_dot;
    Eigen::Map<Eigen::Matrix<double, 3, 1>>(in14) << b2_omega_e2;
    Eigen::Map<Eigen::Matrix<double, 3, 1>>(in15) << bpb1;
    Eigen::Map<Eigen::Matrix<double, 4, 1>>(in16) << bQb1.w(), bQb1.x(), bQb1.y(), bQb1.z();
    Eigen::Map<Eigen::Matrix<double, 3, 1>>(in17) << opg1;
    Eigen::Map<Eigen::Matrix<double, 4, 1>>(in18) << oQg1.w(), oQg1.x(), oQg1.y(), oQg1.z();
    Eigen::Map<Eigen::Matrix<double, 3, 1>>(in19) << opg2;
    Eigen::Map<Eigen::Matrix<double, 4, 1>>(in20) << oQg2.w(), oQg2.x(), oQg2.y(), oQg2.z();
    Eigen::Map<Eigen::Matrix<double, 6, 1>>(in21) << K_1_diag;
    Eigen::Map<Eigen::Matrix<double, 6, 1>>(in22) << B_1_diag;
    Eigen::Map<Eigen::Matrix<double, 6, 1>>(in23) << K_2_diag;
    Eigen::Map<Eigen::Matrix<double, 6, 1>>(in24) << B_2_diag;

    jacobian_h_to_b2Tb1(in1, in2, in3, in4, in5, in6, in7, in8, in9, in10, in11, in12, in13, in14, in15, in16, in17,
                        in18, in19, in20, in21, in22, in23, in24, b_jacobian_h_to_x_state_not_ext);

    Eigen::Map<Eigen::Matrix<double, 12, 7>>(out.data())
        << Eigen::Matrix<double, 12, 7>(b_jacobian_h_to_x_state_not_ext);
  }

  inline virtual void reset()
  {
    x_.setZero();
    y_.setZero();
  }

  virtual unsigned int get_size_state() const
  {
    return 34;
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
    std::cout << "RobotsSpringObjectSystemExt" << std::endl;

    std::cout << "State: " << x_.transpose() << std::endl;
    std::cout << "Output: " << y_.transpose() << std::endl;

    robots_object_system_ptr_->display();
  }

public:
  Eigen::Matrix<double, 34, 1> x_;              // state
  Eigen::Matrix<double, Eigen::Dynamic, 1> y_;  // output

  typename uclv::systems::RobotsSpringObjectSystem::SharedPtr robots_object_system_ptr_;
  const int number_pose_measure_from_robot_;
};

}  // namespace uclv::systems