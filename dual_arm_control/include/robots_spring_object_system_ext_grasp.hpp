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
This class is a extension of the robots_object_system_ext.hpp file.
To the state has been added the transformation matrix oTg1 and oTg2.
The output is the same as the robots_object_system_ext.hpp file.
*/

#pragma once

#include <uclv_systems_lib/continuous_time/continuous_time_linear_state_space.hpp>
#include "geometry_helper.hpp"
#include <Eigen/Dense>
#include <memory>
#include "jacobian_h_to_xstate/jacobian_WRh_to_oTg1_oTg2.h"
#include "jacobian_h_to_xstate/jacobian_WRh_to_oTg1_oTg2.cpp"
#include "jacobian_h_to_xstate/jacobian_h_to_oTg1_oTg2.h"
#include "jacobian_h_to_xstate/jacobian_h_to_oTg1_oTg2.cpp"

namespace uclv::systems
{

class RobotsSpringObjectSystemExtGrasp : public ContinuousTimeStateSpaceInterface<48, 12, Eigen::Dynamic, 1, 1, 1>
{
public:
  using SharedPtr = std::shared_ptr<RobotsSpringObjectSystemExtGrasp>;
  using ConstSharedPtr = std::shared_ptr<const RobotsSpringObjectSystemExtGrasp>;
  using WeakPtr = std::weak_ptr<RobotsSpringObjectSystemExtGrasp>;
  using ConstWeakPtr = std::weak_ptr<const RobotsSpringObjectSystemExtGrasp>;
  using UniquePtr = std::unique_ptr<RobotsSpringObjectSystemExtGrasp>;

  RobotsSpringObjectSystemExtGrasp(
      const Eigen::Matrix<double, 48, 1>& x0,
      typename uclv::systems::RobotsSpringObjectSystemExt::SharedPtr robots_object_system_ext_ptr)
    : x_(x0)
    , robots_object_system_ext_ptr_(robots_object_system_ext_ptr)
    , robots_object_system_ptr_(robots_object_system_ext_ptr->robots_object_system_ptr_)
    , number_pose_measure_from_robot_(robots_object_system_ext_ptr->number_pose_measure_from_robot_)
  {
    y_.resize(number_pose_measure_from_robot_ * 14 + 12 + 14, 1);
    y_.setZero();
  }

  RobotsSpringObjectSystemExtGrasp() = default;

  RobotsSpringObjectSystemExtGrasp(const RobotsSpringObjectSystemExtGrasp& sys) = default;

  virtual ~RobotsSpringObjectSystemExtGrasp() = default;

  virtual RobotsSpringObjectSystemExtGrasp* clone() const
  {
    return new RobotsSpringObjectSystemExtGrasp(*this);
  }

  inline virtual const Eigen::Matrix<double, 48, 1>& get_state() const
  {
    return x_;
  }

  inline virtual const Eigen::Matrix<double, Eigen::Dynamic, 1>& get_output() const
  {
    return y_;
  }

  /*==============================================*/

  /*=============SETTER===========================*/

  inline virtual void set_state(const Eigen::Ref<const Eigen::Matrix<double, 48, 1>>& x)
  {
    x_ = x;
  }

  /*==============================================*/

  /*=============RUNNER===========================*/

  //! State function
  inline virtual void state_fcn(const Eigen::Ref<const Eigen::Matrix<double, 48, 1>>& x,
                                const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                Eigen::Matrix<double, 48, 1>& out) const
  {
    out.setZero();

    update_oTg1_oTg2(x);

    Eigen::Matrix<double, 34, 1> x_out;
    robots_object_system_ext_ptr_->state_fcn(x.block<34, 1>(0, 0), u_k, x_out);
    out.block<34, 1>(0, 0) = x_out;

    // the transformation matrixs oTg1 and oTg2, stored in the state as position and quaternion are assumed constant, so
    // the derivative is zero and the state is not modified
    out.block<14, 1>(34, 0) = Eigen::Matrix<double, 14, 1>::Zero();
  }

  //! Output function
  inline virtual void output_fcn(const Eigen::Ref<const Eigen::Matrix<double, 48, 1>>& x,
                                 const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                 Eigen::Matrix<double, Eigen::Dynamic, 1>& out) const
  {
    (void)u_k;
    out.resize(number_pose_measure_from_robot_ * 14 + 12 + 14, 1);
    out.setZero();

    update_oTg1_oTg2(x);

    // the output is composed by the output of the robots_object_system
    robots_object_system_ext_ptr_->output_fcn(x.block<34, 1>(0, 0), u_k, out);
  }

  //! Jacobian of the state function with respect to the state
  inline virtual void jacobx_state_fcn(const Eigen::Ref<const Eigen::Matrix<double, 48, 1>>& x,
                                       const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                       Eigen::Matrix<double, 48, 48>& out) const
  {
    // the jacobian of the state function is the jacobian of the robots_object_system
    // the jacobian of the transformation matrix b2Tb1_ is zero
    out.setZero();

    update_oTg1_oTg2(x);

    Eigen::Matrix<double, 34, 34> Jx;
    robots_object_system_ext_ptr_->jacobx_state_fcn(x.block<34, 1>(0, 0), u_k, Jx);
    out.block<34, 34>(0, 0) = Jx;

    // Add jacobian WRh to oTg1 oTg2 to the twist term
    Eigen::Matrix<double, 6, 14> J_WRh_oTg1_oTg2;
    get_jacobian_WRh_to_oTg1_oTg2(x, u_k, J_WRh_oTg1_oTg2);
    out.block<6, 14>(7, 34) = robots_object_system_ptr_->Bm_.inverse() * J_WRh_oTg1_oTg2;

    // debug print jacobian out
    // std::cout << "state" << x.transpose() << std::endl;
    // std::cout << "u_k" << u_k.transpose() << std::endl;
    // std::cout << "PRINT JACOBIAN ELEMENTS: \n"<< std::endl;
    // std::cout << "\n J_bpo_dot \n"<< out.block<3,34>(0, 0) << std::endl;
    // std::cout << "\n J_bQo_dot \n"<< out.block<4,34>(3, 0) << std::endl;
    // std::cout << "\n J_otwisto_dot \n"<< out.block<6,34>(7, 0) << std::endl;
    // std::cout << "\n J_b1pe1_dot \n"<< out.block<3,34>(13, 0) << std::endl;
    // std::cout << "\n J_b1Qe1_dot \n"<< out.block<4,34>(16, 0) << std::endl;
    // std::cout << "\n J_b2pe2_dot \n"<< out.block<3,34>(20, 0) << std::endl;
    // std::cout << "\n J_b2Qe2_dot \n"<< out.block<4,34>(23, 0) << std::endl;
    // std::cout << "\n out.block<6, 14>(7, 34) \n"<< out.block<6, 14>(7, 34) << std::endl;
    // std::cout << "\n J_WRh_oTg1_oTg2 \n"<< J_WRh_oTg1_oTg2 << std::endl;
    // std::cout << "\n  robots_object_system_ptr_->Bm_.inverse()  \n"<<  robots_object_system_ptr_->Bm_.inverse()  << std::endl;
    // std::cout << "\n J_WRh_oTg1_oTg2 \n"<< J_WRh_oTg1_oTg2 << std::endl;

  }

  //! Jacobian of the output function with respect to the state
  inline virtual void jacobx_output_fcn(const Eigen::Ref<const Eigen::Matrix<double, 48, 1>>& x,
                                        const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                        Eigen::Matrix<double, Eigen::Dynamic, 48>& out) const
  {
    (void)u_k;
    out.resize(number_pose_measure_from_robot_ * 14 + 12 + 14, 48);
    out.setZero();

    update_oTg1_oTg2(x);


    Eigen::Matrix<double, Eigen::Dynamic, 34> jacob_base;
    robots_object_system_ext_ptr_->jacobx_output_fcn(x.block<34, 1>(0, 0), u_k, jacob_base);
    out.block(0, 0, number_pose_measure_from_robot_ * 14 + 12 + 14, 34) = jacob_base;

    // Add jacobian h to oTg1 oTg2 
    Eigen::Matrix<double, 12, 14> J_h_oTg1_oTg2;
    get_jacobian_h_to_oTg1_oTg2(x, u_k, J_h_oTg1_oTg2);
    out.block(number_pose_measure_from_robot_ * 14, 34, 12, 14) = J_h_oTg1_oTg2;
  }

  inline virtual void get_jacobian_WRh_to_oTg1_oTg2(const Eigen::Ref<const Eigen::Matrix<double, 48, 1>>& x,
                                                    const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                                    Eigen::Matrix<double, 6, 14>& out) const
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

    Eigen::Matrix<double, 3, 1> opg1 = x.block<3, 1>(34, 0);
    Eigen::Quaterniond oQg1(x(37), x(38), x(39), x(40));
    Eigen::Matrix<double, 3, 1> opg2 = x.block<3, 1>(41, 0);
    Eigen::Quaterniond oQg2(x(44), x(45), x(46), x(47));
    oQg1.normalize();
    oQg2.normalize();

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

    jacobian_WRh_to_oTg1_oTg2(in1, in2, in3, in4, in5, in6, in7, in8, in9, in10, in11, in12, in13, in14, in15, in16,
                              in17, in18, in19, in20, in21, in22, in23, in24, b_jacobian_h_to_x_state_not_ext);

    Eigen::Map<Eigen::Matrix<double, 6, 14>>(out.data())
        << Eigen::Matrix<double, 6, 14>(b_jacobian_h_to_x_state_not_ext);
  }

  inline virtual void get_jacobian_h_to_oTg1_oTg2(const Eigen::Ref<const Eigen::Matrix<double, 48, 1>>& x,
                                                    const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                                    Eigen::Matrix<double, 12, 14>& out) const
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


    Eigen::Matrix<double, 3, 1> opg1 = x.block<3, 1>(34, 0);
    Eigen::Quaterniond oQg1(x(37), x(38), x(39), x(40));
    Eigen::Matrix<double, 3, 1> opg2 = x.block<3, 1>(41, 0);
    Eigen::Quaterniond oQg2(x(44), x(45), x(46), x(47));
    oQg1.normalize();
    oQg2.normalize();

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

    Eigen::Matrix<double, 6, 1> K_1_diag = robots_object_system_ptr_->K_1_.diagonal();
    Eigen::Matrix<double, 6, 1> B_1_diag = robots_object_system_ptr_->B_1_.diagonal();
    Eigen::Matrix<double, 6, 1> K_2_diag = robots_object_system_ptr_->K_2_.diagonal();
    Eigen::Matrix<double, 6, 1> B_2_diag = robots_object_system_ptr_->B_2_.diagonal();

    double in1[3], in2[4], in3[3], in4[3], in5[3], in6[4], in7[3], in8[4], in9[3], in10[4];
    double in11[3], in12[3], in13[3], in14[3], in15[3], in16[4], in17[3], in18[4], in19[3], in20[4];
    double in21[6], in22[6], in23[6], in24[6];
    double b_jacobian_h_to_x_state_not_ext[168];

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

    jacobian_h_to_oTg1_oTg2(in1, in2, in3, in4, in5, in6, in7, in8, in9, in10, in11, in12, in13, in14, in15, in16,
                              in17, in18, in19, in20, in21, in22, in23, in24, b_jacobian_h_to_x_state_not_ext);

    Eigen::Map<Eigen::Matrix<double, 12, 14>>(out.data())
        << Eigen::Matrix<double, 12, 14>(b_jacobian_h_to_x_state_not_ext);
  }

  inline virtual void reset()
  {
    x_.setZero();
    y_.setZero();
  }

  virtual unsigned int get_size_state() const
  {
    return 48;
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
    std::cout << "RobotsSpringObjectSystemExtGrasp" << std::endl;

    std::cout << "State: " << x_.transpose() << std::endl;
    std::cout << "Output: " << y_.transpose() << std::endl;

    robots_object_system_ext_ptr_->display();
  }

  inline virtual void update_oTg1_oTg2(const Eigen::Ref<const Eigen::Matrix<double, 48, 1>>& x) const
  {
    Eigen::Matrix<double, 4, 4> oTg1;
    uclv::geometry_helper::pose_to_matrix(x.block<7, 1>(34, 0), oTg1);
    Eigen::Matrix<double, 4, 4> oTg2;
    uclv::geometry_helper::pose_to_matrix(x.block<7, 1>(41, 0), oTg2);
    robots_object_system_ptr_->set_oTg1(oTg1);
    robots_object_system_ptr_->set_oTg2(oTg2);
  }

public:
  Eigen::Matrix<double, 48, 1> x_;              // state
  Eigen::Matrix<double, Eigen::Dynamic, 1> y_;  // output

  typename uclv::systems::RobotsSpringObjectSystemExt::SharedPtr robots_object_system_ext_ptr_;
  typename uclv::systems::RobotsSpringObjectSystem::SharedPtr robots_object_system_ptr_;
  const int number_pose_measure_from_robot_;
};

}  // namespace uclv::systems