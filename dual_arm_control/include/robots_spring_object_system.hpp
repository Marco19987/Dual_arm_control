/*

    Copyright 2024 Università della Campania Luigi Vanvitelli

    Authors: Marco De Simone <marco.desimone@unicampania.it>

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
    % system describing the interaction between 2 robots and one object.
    % The contact between robots and object is modeled as a spring-damper
    % system. The object is assumed to be a rigid body with known inertia.
    % The input to the system are the twist of the robots end-effector expressed
    % in the robots base frame.
    % the output of the system are the measured poses of the object in the
    % robots base frame, the wrenches exerted by robots on the object
    % expressed and applied in the grasp frames and the fkines.
    % the state is represented by the pose of the object in the inertial
    % frame an the twist, and the robots fkine wrt their base frames.
    % pose = [px py pz qw qx qy qz]' epressed in the base frame;
    % twist = [vx vy vz omegax omegay omegaz]' expressed in the object frame;
*/

// Comments
// number_pose_measure_from_robot_: number of pose measures from each robot
// the size of the output is number_pose_measure_from_robot_*14 +12 +14, where 14 results from 7 (pose elements) dot 2
// (number of robots), 12 is the measured wrench by the robots and 14 is the fkine of the robots

#pragma once

#include <uclv_systems_lib/continuous_time/continuous_time_linear_state_space.hpp>
#include "geometry_helper.hpp"
#include <Eigen/Dense>
#include <memory>
#include "jacobian_h_to_xstate/jacobian_h_to_x_state_not_ext.h"
#include "jacobian_h_to_xstate/jacobian_h_to_x_state_not_ext.cpp"

namespace uclv::systems
{

class RobotsSpringObjectSystem : public ContinuousTimeStateSpaceInterface<27, 12, Eigen::Dynamic, 1, 1, 1>
{
public:
  using SharedPtr = std::shared_ptr<RobotsSpringObjectSystem>;
  using ConstSharedPtr = std::shared_ptr<const RobotsSpringObjectSystem>;
  using WeakPtr = std::weak_ptr<RobotsSpringObjectSystem>;
  using ConstWeakPtr = std::weak_ptr<const RobotsSpringObjectSystem>;
  using UniquePtr = std::unique_ptr<RobotsSpringObjectSystem>;

  RobotsSpringObjectSystem(const Eigen::Matrix<double, 27, 1>& x0, const Eigen::Matrix<double, 6, 6>& Bm,
                           const Eigen::Matrix<double, 3, 1>& bg, const Eigen::Matrix4d& oTg1,
                           const Eigen::Matrix4d& oTg2, const Eigen::Matrix4d& b2Tb1, const Eigen::Matrix4d& bTb1,
                           const Eigen::Matrix<double, 6, 6>& viscous_friction, const Eigen::Matrix<double, 6, 6>& K_1,
                           const Eigen::Matrix<double, 6, 6>& B_1, const Eigen::Matrix<double, 6, 6>& K_2,
                           const Eigen::Matrix<double, 6, 6>& B_2, const int number_pose_measure_from_robot)
    : x_(x0)
    , Bm_(Bm)
    , bg_(bg)
    , oTg1_(oTg1)
    , oTg2_(oTg2)
    , b2Tb1_(b2Tb1)
    , bTb1_(bTb1)
    , viscous_friction_(viscous_friction)
    , K_1_(K_1)
    , B_1_(B_1)
    , K_2_(K_2)
    , B_2_(B_2)
    , number_pose_measure_from_robot_(number_pose_measure_from_robot)
  {
    update_grasp_matrix();
    update_Rbar();
    y_.resize(number_pose_measure_from_robot_ * 14 + 12 + 14, 1);
    y_.setZero();
    oh_bias_.setZero();
  }

  RobotsSpringObjectSystem() : number_pose_measure_from_robot_(0){};

  RobotsSpringObjectSystem(const RobotsSpringObjectSystem& sys) = default;

  virtual ~RobotsSpringObjectSystem() = default;

  virtual RobotsSpringObjectSystem* clone() const
  {
    return new RobotsSpringObjectSystem(*this);
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

  inline virtual const Eigen::Matrix<double, 27, 1>& get_state() const
  {
    return x_;
  }

  inline virtual const Eigen::Matrix<double, Eigen::Dynamic, 1>& get_output() const
  {
    return y_;
  }

  /*==============================================*/

  /*=============SETTER===========================*/

  inline virtual void set_state(const Eigen::Ref<const Eigen::Matrix<double, 27, 1>>& x)
  {
    x_ = x;
  }

  /*==============================================*/

  /*=============RUNNER===========================*/

  //! State function
  inline virtual void state_fcn(const Eigen::Ref<const Eigen::Matrix<double, 27, 1>>& x,
                                const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                Eigen::Matrix<double, 27, 1>& out) const
  {
    Eigen::Quaterniond bQo(x(3), x(4), x(5), x(6));
    bQo.normalize();
    Eigen::Quaterniond b1Qe1(x(16), x(17), x(18), x(19));
    b1Qe1.normalize();
    Eigen::Quaterniond b2Qe2(x(23), x(24), x(25), x(26));
    b2Qe2.normalize();

    Eigen::Matrix3d bRo = bQo.toRotationMatrix();  // object's rotation matrix in the base frame
    Eigen::Matrix<double, 6, 6> bRo_bar = Eigen::Matrix<double, 6, 6>::Zero();
    bRo_bar.block<3, 3>(0, 0) = bRo;
    bRo_bar.block<3, 3>(3, 3) = bRo;

    Eigen::Matrix<double, 12, 1> h;
    spring_model(x, u_k, h);  // spring model

    Eigen::Matrix<double, 6, 1> oh = W_ * Rbar_ * h;  // resulting wrench in the object frame
    // std::cout << "h spring" << h.transpose() << "\n";

    // std::cout << "oh: " << oh.transpose() << "\n";
    // std::cout << "oh_bias_: " << oh_bias_.transpose() << "\n";

    oh = oh - oh_bias_;  // remove bias

    // std::cout << "oh debiased: " << oh.transpose() << "\n";

    Eigen::Matrix<double, 3, 1> ovo = x.block<3, 1>(7, 0);       // object's linear velocity in the object frame
    Eigen::Matrix<double, 3, 1> oomegao = x.block<3, 1>(10, 0);  // object's angular velocity in the object frame
    Eigen::Matrix<double, 6, 1> o_twist_o;
    o_twist_o << x.block<3, 1>(7, 0), x.block<3, 1>(10, 0);  // object twist in the object frame

    Eigen::Matrix<double, 3, 3> skew_oomegao;
    Eigen::Matrix<double, 3, 3> skew_ovo;
    uclv::geometry_helper::skew(oomegao, skew_oomegao);
    uclv::geometry_helper::skew(ovo, skew_ovo);
    Eigen::Matrix<double, 6, 6> double_skew = Eigen::Matrix<double, 6, 6>::Zero();
    double_skew.block<3, 3>(0, 0) = skew_oomegao;
    double_skew.block<3, 3>(3, 0) = skew_ovo;
    double_skew.block<3, 3>(3, 3) = skew_oomegao;

    Eigen::Matrix<double, 6, 1> bg_ext = Eigen::Matrix<double, 6, 1>::Zero();
    bg_ext.block<3, 1>(0, 0) = bg_;

    Eigen::Matrix<double, 6, 1> o_twist_dot_o =
        Bm_.inverse() *
        (oh + Bm_ * bRo_bar.transpose() * bg_ext - viscous_friction_ * o_twist_o - double_skew * Bm_ * o_twist_o);

    // std::cout << "oh external: " << (oh + Bm_ * bRo_bar.transpose() * bg_ext).transpose() << "\n";

    out.block<3, 1>(0, 0) = bRo * ovo;
    Eigen::Quaterniond qdot;
    uclv::geometry_helper::quaternion_propagation(bQo, bRo * oomegao, qdot);
    out.block<4, 1>(3, 0) << qdot.w(), qdot.vec();
    out.block<6, 1>(7, 0) = o_twist_dot_o;
    out.block<3, 1>(13, 0) = u_k.block<3, 1>(0, 0);  // b1pe1_dot
    uclv::geometry_helper::quaternion_propagation(b1Qe1, u_k.block<3, 1>(3, 0), qdot);
    out.block<4, 1>(16, 0) << qdot.w(), qdot.vec();
    out.block<3, 1>(20, 0) = u_k.block<3, 1>(6, 0);  // b2pe2_dot
    uclv::geometry_helper::quaternion_propagation(b2Qe2, u_k.block<3, 1>(9, 0), qdot);
    out.block<4, 1>(23, 0) << qdot.w(), qdot.vec();
  }

  inline virtual void spring_model(const Eigen::Ref<const Eigen::Matrix<double, 27, 1>>& x,
                                   const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                   Eigen::Matrix<double, 12, 1>& out) const
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

    Eigen::Matrix<double, 3, 1> b1pe1_dot =
        u_k.block<3, 1>(0, 0);  // robot 1 end-effector linear velocity in the robot 1 base frame
    Eigen::Matrix<double, 3, 1> b1_omega_e1 =
        u_k.block<3, 1>(3, 0);  // robot 1 end-effector angular velocity in the robot 1 base frame
    Eigen::Matrix<double, 3, 1> b2pe2_dot =
        u_k.block<3, 1>(6, 0);  // robot 2 end-effector linear velocity in the robot 2 base frame
    Eigen::Matrix<double, 3, 1> b2_omega_e2 =
        u_k.block<3, 1>(9, 0);  // robot 2 end-effector angular velocity in the robot 2 base frame

    bQo.normalize();
    Eigen::Matrix3d bRo = bQo.toRotationMatrix();

    // Compute wrench spring 1
    Eigen::Matrix3d oRg1 = oTg1_.block<3, 3>(0, 0);
    Eigen::Matrix3d g1Rb = oRg1.transpose() * bRo.transpose();
    Eigen::Matrix<double, 3, 1> bpe1_b = bTb1_.block<3, 1>(0, 3) + bTb1_.block<3, 3>(0, 0) * b1pe1_b1;
    Eigen::Matrix<double, 3, 1> bpg1_b = bpo_b + bRo * oTg1_.block<3, 1>(0, 3);
    Eigen::Matrix<double, 3, 1> g1fg1_e = K_1_.block<3, 3>(0, 0) * g1Rb * (bpe1_b - bpg1_b);

    Eigen::Matrix<double, 3, 1> bpe1_dot = bTb1_.block<3, 3>(0, 0) * b1pe1_dot;
    Eigen::Matrix<double, 3, 3> skew_b_omega_o;
    uclv::geometry_helper::skew(bRo * oomegao, skew_b_omega_o);
    Eigen::Matrix<double, 3, 1> bpg1_dot = bRo * ovo + skew_b_omega_o * bRo * oTg1_.block<3, 1>(0, 3);
    Eigen::Matrix<double, 3, 1> g1fg1_beta = B_1_.block<3, 3>(0, 0) * g1Rb * (bpe1_dot - bpg1_dot);

    Eigen::Quaterniond bQb1(bTb1_.block<3, 3>(0, 0));
    Eigen::Quaterniond oQg1(oRg1);
    Eigen::Quaterniond g1Qb = oQg1.inverse() * bQo.inverse();
    Eigen::Quaterniond bQe1 = bQb1 * b1Qe1;
    Eigen::Quaterniond g1Qe1 = g1Qb * bQe1;
    Eigen::Matrix<double, 3, 1> g1_tau_g1_e = K_1_.block<3, 3>(3, 3) * g1Qe1.vec();

    Eigen::Matrix<double, 3, 1> g1_tau_g1_beta =
        B_1_.block<3, 3>(3, 3) * g1Rb * (bTb1_.block<3, 3>(0, 0) * b1_omega_e1 - bRo * oomegao);

    Eigen::Matrix<double, 6, 1> g1hg1;
    g1hg1 << g1fg1_e + g1fg1_beta, g1_tau_g1_e + g1_tau_g1_beta;

    // Compute wrench spring 2
    Eigen::Matrix3d oRg2 = oTg2_.block<3, 3>(0, 0);
    Eigen::Matrix3d g2Rb = oRg2.transpose() * bRo.transpose();
    Eigen::Matrix4d b1Tb2 = b2Tb1_.inverse();
    Eigen::Matrix<double, 3, 1> bpe2_b =
        bTb1_.block<3, 1>(0, 3) +
        bTb1_.block<3, 3>(0, 0) * (b1Tb2.block<3, 1>(0, 3) + b1Tb2.block<3, 3>(0, 0) * b2pe2_b2);
    Eigen::Matrix<double, 3, 1> bpg2_b = bpo_b + bRo * oTg2_.block<3, 1>(0, 3);
    Eigen::Matrix<double, 3, 1> g2fg2_e = K_2_.block<3, 3>(0, 0) * g2Rb * (bpe2_b - bpg2_b);

    Eigen::Matrix<double, 3, 1> bpe2_dot = bTb1_.block<3, 3>(0, 0) * b1Tb2.block<3, 3>(0, 0) * b2pe2_dot;
    Eigen::Matrix<double, 3, 1> bpg2_dot = bRo * ovo + skew_b_omega_o * bRo * oTg2_.block<3, 1>(0, 3);
    Eigen::Matrix<double, 3, 1> g2fg2_beta = B_2_.block<3, 3>(0, 0) * g2Rb * (bpe2_dot - bpg2_dot);

    Eigen::Quaterniond b1Qb2(b1Tb2.block<3, 3>(0, 0));
    Eigen::Quaterniond oQg2(oRg2);
    Eigen::Quaterniond g2Qb = oQg2.inverse() * bQo.inverse();
    Eigen::Quaterniond b1Qe2 = b1Qb2 * b2Qe2;
    Eigen::Quaterniond bQe2 = bQb1 * b1Qe2;
    Eigen::Quaterniond g2Qe2 = g2Qb * bQe2;
    Eigen::Matrix<double, 3, 1> g2_tau_g2_e = K_2_.block<3, 3>(3, 3) * g2Qe2.vec();

    Eigen::Matrix<double, 3, 1> g2_tau_g2_beta =
        B_2_.block<3, 3>(3, 3) * g2Rb *
        (bTb1_.block<3, 3>(0, 0) * b1Tb2.block<3, 3>(0, 0) * b2_omega_e2 - bRo * oomegao);

    Eigen::Matrix<double, 6, 1> g2hg2;
    g2hg2 << g2fg2_e + g2fg2_beta, g2_tau_g2_e + g2_tau_g2_beta;

    out << g1hg1, g2hg2;
  }

  void get_object_wrench(const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                         Eigen::Matrix<double, 6, 1>& oh) const
  {
    oh = W_ * Rbar_ * u_k;  // resulting wrench in the object frame
  }

  //! Output function
  inline virtual void output_fcn(const Eigen::Ref<const Eigen::Matrix<double, 27, 1>>& x,
                                 const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                 Eigen::Matrix<double, Eigen::Dynamic, 1>& out) const
  {
    (void)u_k;
    out.resize(number_pose_measure_from_robot_ * 14 + 12 + 14, 1);
    out.setZero();
    // in the output there are the measrements of the object pose in the base frame of the robots
    Eigen::Quaterniond bQo(x(3), x(4), x(5), x(6));
    bQo.normalize();

    Eigen::Matrix<double, 3, 1> b1po;
    Eigen::Matrix<double, 3, 1> b2po;

    // measured object position in the robots base frame b1 and b2
    b1po = bTb1_.block<3, 3>(0, 0).transpose() * (x.block<3, 1>(0, 0) - bTb1_.block<3, 1>(0, 3));
    b2po = b2Tb1_.block<3, 1>(0, 3) + b2Tb1_.block<3, 3>(0, 0) * b1po;

    // measured object orientation in the robots base frame b1 and b2
    Eigen::Quaterniond bQb1(bTb1_.block<3, 3>(0, 0));
    Eigen::Quaterniond b2Qb1(b2Tb1_.block<3, 3>(0, 0));
    Eigen::Quaterniond b1Qo = bQb1.inverse() * bQo;
    b1Qo.normalize();
    Eigen::Quaterniond b2Qo = b2Qb1 * b1Qo;
    b2Qo.normalize();

    // out is [b1po;b1Qo;b1po;b1Qo; ... ;b2po;b2Qo; .. ; b2po;b2Qo]
    for (int i = 0; i < number_pose_measure_from_robot_; i++)
    {
      out.block(i * 7, 0, 3, 1) = b1po;
      out.block(i * 7 + 3, 0, 4, 1) << b1Qo.w(), b1Qo.vec();
      out.block((i + number_pose_measure_from_robot_) * 7, 0, 3, 1) = b2po;
      out.block((i + number_pose_measure_from_robot_) * 7 + 3, 0, 4, 1) << b2Qo.w(), b2Qo.vec();
    }

    // output of the wrenches exerted by the robots on the object
    Eigen::Matrix<double, 12, 1> wrenches;
    spring_model(x, u_k, wrenches);
    out.block<12, 1>(7 * number_pose_measure_from_robot_ * 2, 0) = wrenches;

    // output of the fkine of the robots
    out.block<3, 1>(7 * number_pose_measure_from_robot_ * 2 + 12 + 0, 0) = x.block<3, 1>(13, 0);
    out.block<4, 1>(7 * number_pose_measure_from_robot_ * 2 + 12 + 3, 0) = x.block<4, 1>(16, 0);
    out.block<3, 1>(7 * number_pose_measure_from_robot_ * 2 + 12 + 7, 0) = x.block<3, 1>(20, 0);
    out.block<4, 1>(7 * number_pose_measure_from_robot_ * 2 + 12 + 10, 0) = x.block<4, 1>(23, 0);
  }

  //! Jacobian of the state function with respect to the state
  inline virtual void jacobx_state_fcn(const Eigen::Ref<const Eigen::Matrix<double, 27, 1>>& x,
                                       const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                       Eigen::Matrix<double, 27, 27>& out) const
  {
    out.setZero();
    Eigen::Quaterniond bQo(x(3), x(4), x(5), x(6));
    bQo.normalize();

    Eigen::Matrix<double, 3, 1> ovo = x.block<3, 1>(7, 0);       // object's linear velocity in the object frame
    Eigen::Matrix<double, 3, 1> oomegao = x.block<3, 1>(10, 0);  // object's angular velocity in the object frame
    Eigen::Matrix<double, 6, 1> o_twist_o;
    o_twist_o << ovo, oomegao;  // object twist in the object frame

    Eigen::Matrix3d bRo = bQo.toRotationMatrix();

    // linear velocity term
    Eigen::Matrix<double, 3, 4> J_bpo_dot_to_bQo;
    Jacobian_bpo_dot_to_bQo(bQo, ovo, J_bpo_dot_to_bQo);
    out.block<3, 3>(0, 7) = bRo;
    out.block<3, 4>(0, 3) = J_bpo_dot_to_bQo;

    // quaternion dot term
    Eigen::Matrix<double, 4, 4> J_bQo_dot_to_bQo;  // jacobian of 0.5*E[omega]*Q wrt Q

    Jacobian_bQo_dot_to_bQo(oomegao, J_bQo_dot_to_bQo);

    Eigen::Matrix<double, 4, 3> J_bQo_dot_to_oomegao;  // jacobian of 0.5*E[omega]*Q wrt omega

    Jacobian_bQo_dot_to_oomegao(bQo, J_bQo_dot_to_oomegao);

    out.block<4, 4>(3, 3) = J_bQo_dot_to_bQo;
    out.block<4, 3>(3, 10) = J_bQo_dot_to_oomegao;

    // acceleration term
    Eigen::Matrix<double, 6, 4> J_otwisto_dot_to_bQo;

    Jacobian_otwisto_dot_to_bQo(bQo, bg_, J_otwisto_dot_to_bQo);

    out.block<6, 4>(7, 3) = J_otwisto_dot_to_bQo;

    Eigen::Matrix<double, 6, 6> J_otwisto_dot_to_otwisto_skew_term;

    Jacobian_otwisto_dot_to_otwisto_skew_term(Bm_, o_twist_o, J_otwisto_dot_to_otwisto_skew_term);

    J_otwisto_dot_to_otwisto_skew_term = Bm_.inverse() * J_otwisto_dot_to_otwisto_skew_term;

    out.block<6, 6>(7, 7) = J_otwisto_dot_to_otwisto_skew_term - Bm_.inverse() * viscous_friction_;

    // Add Jacobian of h with respect to the state to the twist term
    Eigen::Matrix<double, 12, 27> J_h_x;
    get_jacobian_h_to_state(x, u_k, J_h_x);
    out.block<6, 27>(7, 0) += Bm_.inverse() * (W_ * Rbar_ * J_h_x);

    // Jacobian of fkine robots with respect to their quaternions
    Eigen::Matrix<double, 4, 4> J_b1Qe1_dot_to_b1Qe1;
    Jacobian_bQo_dot_to_bQo(u_k.block<3, 1>(3, 0), J_b1Qe1_dot_to_b1Qe1);
    out.block<4, 4>(16, 16) = J_b1Qe1_dot_to_b1Qe1;

    Eigen::Matrix<double, 4, 4> J_b2Qe2_dot_to_b2Qe2;
    Jacobian_bQo_dot_to_bQo(u_k.block<3, 1>(9, 0), J_b2Qe2_dot_to_b2Qe2);
    out.block<4, 4>(23, 23) = J_b2Qe2_dot_to_b2Qe2;


    // std::cout << "PRINT JACOBIAN ELEMENTS: \n"<< std::endl;
    // std::cout << "\n J_bpo_dot \n"<< out.block<3,27>(0, 0).transpose() << std::endl;
    // std::cout << "\n J_bQo_dot \n"<< out.block<4,27>(3, 0).transpose() << std::endl;
    // std::cout << "\n J_otwisto_dot \n"<< out.block<6,27>(7, 0).transpose() << std::endl;
    // std::cout << "\n J_b1pe1_dot \n"<< out.block<3,27>(13, 0).transpose() << std::endl;
    // std::cout << "\n J_b1Qe1_dot \n"<< out.block<4,27>(16, 0).transpose() << std::endl;
    // std::cout << "\n J_b2pe2_dot \n"<< out.block<3,27>(20, 0).transpose() << std::endl;
    // std::cout << "\n J_b2Qe2_dot \n"<< out.block<4,27>(23, 0).transpose() << std::endl;
    // std::cout << "\n J_h_x \n"<< J_h_x.transpose() << std::endl; 
  }
  inline virtual void get_jacobian_h_to_state(const Eigen::Ref<const Eigen::Matrix<double, 27, 1>>& x,
                                              const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                              Eigen::Matrix<double, 12, 27>& out) const
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

    Eigen::Matrix<double, 3, 1> b2pb1 = b2Tb1_.block<3, 1>(0, 3);
    Eigen::Quaterniond b2Qb1(b2Tb1_.block<3, 3>(0, 0));

    Eigen::Matrix<double, 3, 1> b1pe1_dot =
        u_k.block<3, 1>(0, 0);  // robot 1 end-effector linear velocity in the robot 1 base frame
    Eigen::Matrix<double, 3, 1> b1_omega_e1 =
        u_k.block<3, 1>(3, 0);  // robot 1 end-effector angular velocity in the robot 1 base frame
    Eigen::Matrix<double, 3, 1> b2pe2_dot =
        u_k.block<3, 1>(6, 0);  // robot 2 end-effector linear velocity in the robot 2 base frame
    Eigen::Matrix<double, 3, 1> b2_omega_e2 =
        u_k.block<3, 1>(9, 0);  // robot 2 end-effector angular velocity in the robot 2 base frame

    Eigen::Matrix<double, 3, 1> bpb1 = bTb1_.block<3, 1>(0, 3);
    Eigen::Quaterniond bQb1(bTb1_.block<3, 3>(0, 0));

    Eigen::Matrix<double, 3, 1> opg1 = oTg1_.block<3, 1>(0, 3);
    Eigen::Quaterniond oQg1(oTg1_.block<3, 3>(0, 0));
    Eigen::Matrix<double, 3, 1> opg2 = oTg2_.block<3, 1>(0, 3);
    Eigen::Quaterniond oQg2(oTg2_.block<3, 3>(0, 0));

    Eigen::Matrix<double, 6, 1> K_1_diag = K_1_.diagonal();
    Eigen::Matrix<double, 6, 1> B_1_diag = B_1_.diagonal();
    Eigen::Matrix<double, 6, 1> K_2_diag = K_2_.diagonal();
    Eigen::Matrix<double, 6, 1> B_2_diag = B_2_.diagonal();

    double in1[3], in2[4], in3[3], in4[3], in5[3], in6[4], in7[3], in8[4], in9[3], in10[4];
    double in11[3], in12[3], in13[3], in14[3], in15[3], in16[4], in17[3], in18[4], in19[3], in20[4];
    double in21[6], in22[6], in23[6], in24[6];
    double b_jacobian_h_to_x_state_not_ext[324];

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

    
    // debug print
    jacobian_h_to_x_state_not_ext(in1, in2, in3, in4, in5, in6, in7, in8, in9, in10, in11, in12, in13, in14, in15, in16,
                                  in17, in18, in19, in20, in21, in22, in23, in24, b_jacobian_h_to_x_state_not_ext);

    Eigen::Map<Eigen::Matrix<double, 12, 27>>(out.data())
        << Eigen::Matrix<double, 12, 27>(b_jacobian_h_to_x_state_not_ext);
  }

  void Jacobian_bpo_dot_to_bQo(const Eigen::Quaterniond& bQo, const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& ovo,
                               Eigen::Matrix<double, 3, 4>& out) const
  {
    out.setZero();
    double t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13;
    t2 = bQo.w() * ovo(0) * 2.0;
    t3 = bQo.w() * ovo(1) * 2.0;
    t4 = bQo.x() * ovo(0) * 2.0;
    t5 = bQo.w() * ovo(2) * 2.0;
    t6 = bQo.x() * ovo(1) * 2.0;
    t7 = bQo.y() * ovo(0) * 2.0;
    t8 = bQo.x() * ovo(2) * 2.0;
    t9 = bQo.y() * ovo(1) * 2.0;
    t10 = bQo.z() * ovo(0) * 2.0;
    t11 = bQo.y() * ovo(2) * 2.0;
    t12 = bQo.z() * ovo(1) * 2.0;
    t13 = bQo.z() * ovo(2) * 2.0;
    out(0, 0) = t11 - t12;
    out(0, 1) = t9 + t13;
    out(0, 2) = t5 + t6 - bQo.y() * ovo(0) * 4.0;
    out(0, 3) = -t3 + t8 - bQo.z() * ovo(0) * 4.0;
    out(1, 0) = -t8 + t10;
    out(1, 1) = -t5 + t7 - bQo.x() * ovo(1) * 4.0;
    out(1, 2) = t4 + t13;
    out(1, 3) = t2 + t11 - bQo.z() * ovo(1) * 4.0;
    out(2, 0) = t6 - t7;
    out(2, 1) = t3 + t10 - bQo.x() * ovo(2) * 4.0;
    out(2, 2) = -t2 + t12 - bQo.y() * ovo(2) * 4.0;
    out(2, 3) = t4 + t9;
  }

  void Jacobian_bQo_dot_to_bQo(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& oomegao,
                               Eigen::Matrix<double, 4, 4>& out) const
  {
    out.setZero();
    double oomegao1 = oomegao(0);
    double oomegao2 = oomegao(1);
    double oomegao3 = oomegao(2);
    double t2 = oomegao1 / 2.0;
    double t3 = oomegao2 / 2.0;
    double t4 = oomegao3 / 2.0;
    double t5 = -t2;
    double t6 = -t3;
    double t7 = -t4;
    out(0, 1) = t5;
    out(0, 2) = t6;
    out(0, 3) = t7;
    out(1, 0) = t2;
    out(1, 2) = t4;
    out(1, 3) = t6;
    out(2, 0) = t3;
    out(2, 1) = t7;
    out(2, 3) = t2;
    out(3, 0) = t4;
    out(3, 1) = t3;
    out(3, 2) = t5;
  }

  void Jacobian_bQo_dot_to_oomegao(const Eigen::Quaterniond& bQo, Eigen::Matrix<double, 4, 3>& out) const
  {
    out.setZero();
    double t2 = bQo.w() / 2.0;
    double t3 = bQo.x() / 2.0;
    double t4 = bQo.y() / 2.0;
    double t5 = bQo.z() / 2.0;
    double t6 = -t3;
    double t7 = -t4;
    double t8 = -t5;
    out(0, 0) = t6;
    out(0, 1) = t7;
    out(0, 2) = t8;
    out(1, 0) = t2;
    out(1, 1) = t8;
    out(1, 2) = t4;
    out(2, 0) = t5;
    out(2, 1) = t2;
    out(2, 2) = t6;
    out(3, 0) = t7;
    out(3, 1) = t3;
    out(3, 2) = t2;
  }

  void Jacobian_otwisto_dot_to_bQo(const Eigen::Quaterniond& bQo,
                                   const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& bg,
                                   Eigen::Matrix<double, 6, 4>& out) const
  {
    out.setZero();
    double bQo1 = bQo.w();
    double bQo2 = bQo.x();
    double bQo3 = bQo.y();
    double bQo4 = bQo.z();
    double bg1 = bg(0);
    double bg2 = bg(1);
    double bg3 = bg(2);
    double t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13;
    t2 = bQo1 * bg1 * 2.0;
    t3 = bQo1 * bg2 * 2.0;
    t4 = bQo2 * bg1 * 2.0;
    t5 = bQo1 * bg3 * 2.0;
    t6 = bQo2 * bg2 * 2.0;
    t7 = bQo3 * bg1 * 2.0;
    t8 = bQo2 * bg3 * 2.0;
    t9 = bQo3 * bg2 * 2.0;
    t10 = bQo4 * bg1 * 2.0;
    t11 = bQo3 * bg3 * 2.0;
    t12 = bQo4 * bg2 * 2.0;
    t13 = bQo4 * bg3 * 2.0;
    out(0, 0) = -t11 + t12;
    out(0, 1) = t9 + t13;
    out(0, 2) = -t5 + t6 - bQo3 * bg1 * 4.0;
    out(0, 3) = t3 + t8 - bQo4 * bg1 * 4.0;
    out(1, 0) = t8 - t10;
    out(1, 1) = t5 + t7 - bQo2 * bg2 * 4.0;
    out(1, 2) = t4 + t13;
    out(1, 3) = -t2 + t11 - bQo4 * bg2 * 4.0;
    out(2, 0) = -t6 + t7;
    out(2, 1) = -t3 + t10 - bQo2 * bg3 * 4.0;
    out(2, 2) = t2 + t12 - bQo3 * bg3 * 4.0;
    out(2, 3) = t4 + t9;
  }

  void Jacobian_otwisto_dot_to_otwisto_skew_term(const Eigen::Ref<const Eigen::Matrix<double, 6, 6>>& Bm,
                                                 const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& o_twist_o,
                                                 Eigen::Matrix<double, 6, 6>& out) const
  {
    out.setZero();
    double ovo1 = o_twist_o(0);
    double ovo2 = o_twist_o(1);
    double ovo3 = o_twist_o(2);
    double oomegao1 = o_twist_o(3);
    double oomegao2 = o_twist_o(4);
    double oomegao3 = o_twist_o(5);
    double Bm1_1 = Bm(0, 0);
    double Bm1_2 = Bm(0, 1);
    double Bm1_3 = Bm(0, 2);
    double Bm1_4 = Bm(0, 3);
    double Bm1_5 = Bm(0, 4);
    double Bm1_6 = Bm(0, 5);
    double Bm2_1 = Bm(1, 0);
    double Bm2_2 = Bm(1, 1);
    double Bm2_3 = Bm(1, 2);
    double Bm2_4 = Bm(1, 3);
    double Bm2_5 = Bm(1, 4);
    double Bm2_6 = Bm(1, 5);
    double Bm3_1 = Bm(2, 0);
    double Bm3_2 = Bm(2, 1);
    double Bm3_3 = Bm(2, 2);
    double Bm3_4 = Bm(2, 3);
    double Bm3_5 = Bm(2, 4);
    double Bm3_6 = Bm(2, 5);
    double Bm4_1 = Bm(3, 0);
    double Bm4_2 = Bm(3, 1);
    double Bm4_3 = Bm(3, 2);
    double Bm4_4 = Bm(3, 3);
    double Bm4_5 = Bm(3, 4);
    double Bm4_6 = Bm(3, 5);
    double Bm5_1 = Bm(4, 0);
    double Bm5_2 = Bm(4, 1);
    double Bm5_3 = Bm(4, 2);
    double Bm5_4 = Bm(4, 3);
    double Bm5_5 = Bm(4, 4);
    double Bm5_6 = Bm(4, 5);
    double Bm6_1 = Bm(5, 0);
    double Bm6_2 = Bm(5, 1);
    double Bm6_3 = Bm(5, 2);
    double Bm6_4 = Bm(5, 3);
    double Bm6_5 = Bm(5, 4);
    double Bm6_6 = Bm(5, 5);

    double t2 = Bm1_4 * oomegao1;
    double t3 = Bm1_5 * oomegao2;
    double t4 = Bm1_6 * oomegao3;
    double t5 = Bm2_4 * oomegao1;
    double t6 = Bm2_5 * oomegao2;
    double t7 = Bm2_6 * oomegao3;
    double t8 = Bm3_4 * oomegao1;
    double t9 = Bm3_5 * oomegao2;
    double t10 = Bm3_6 * oomegao3;
    double t11 = Bm4_4 * oomegao1;
    double t12 = Bm5_5 * oomegao2;
    double t13 = Bm6_6 * oomegao3;
    double t14 = Bm1_1 * ovo1;
    double t15 = Bm1_2 * ovo2;
    double t16 = Bm1_3 * ovo3;
    double t17 = Bm2_1 * ovo1;
    double t18 = Bm2_2 * ovo2;
    double t19 = Bm2_3 * ovo3;
    double t20 = Bm3_1 * ovo1;
    double t21 = Bm3_2 * ovo2;
    double t22 = Bm3_3 * ovo3;
    double t23 = Bm4_1 * ovo1;
    double t24 = Bm4_2 * ovo2;
    double t25 = Bm4_3 * ovo3;
    double t26 = Bm5_1 * ovo1;
    double t27 = Bm5_2 * ovo2;
    double t28 = Bm5_3 * ovo3;
    double t29 = Bm6_1 * ovo1;
    double t30 = Bm6_2 * ovo2;
    double t31 = Bm6_3 * ovo3;
    double t32 = -t2;
    double t33 = -t3;
    double t34 = -t6;
    double t35 = -t7;
    double t36 = -t8;
    double t37 = -t10;
    double t38 = -t14;
    double t39 = -t15;
    double t40 = -t18;
    double t41 = -t19;
    double t42 = -t20;
    double t43 = -t22;
    out(0, 0) = Bm2_1 * oomegao3 - Bm3_1 * oomegao2;
    out(0, 1) = Bm2_2 * oomegao3 - Bm3_2 * oomegao2;
    out(0, 2) = Bm2_3 * oomegao3 - Bm3_3 * oomegao2;
    out(0, 3) = Bm2_4 * oomegao3 - Bm3_4 * oomegao2;
    out(0, 4) = t9 * -2.0 - t21 + t36 + t37 + t42 + t43 + Bm2_5 * oomegao3;
    out(0, 5) = t5 + t6 + t7 * 2.0 + t17 + t18 + t19 - Bm3_6 * oomegao2;
    out(1, 0) = -Bm1_1 * oomegao3 + Bm3_1 * oomegao1;
    out(1, 1) = -Bm1_2 * oomegao3 + Bm3_2 * oomegao1;
    out(1, 2) = -Bm1_3 * oomegao3 + Bm3_3 * oomegao1;
    out(1, 3) = t8 * 2.0 + t9 + t10 + t20 + t21 + t22 - Bm1_4 * oomegao3;
    out(1, 4) = -Bm1_5 * oomegao3 + Bm3_5 * oomegao1;
    out(1, 5) = t4 * -2.0 - t16 + t32 + t33 + t38 + t39 + Bm3_6 * oomegao1;
    out(2, 0) = Bm1_1 * oomegao2 - Bm2_1 * oomegao1;
    out(2, 1) = Bm1_2 * oomegao2 - Bm2_2 * oomegao1;
    out(2, 2) = Bm1_3 * oomegao2 - Bm2_3 * oomegao1;
    out(2, 3) = t5 * -2.0 - t17 + t34 + t35 + t40 + t41 + Bm1_4 * oomegao2;
    out(2, 4) = t2 + t3 * 2.0 + t4 + t14 + t15 + t16 - Bm2_5 * oomegao1;
    out(2, 5) = Bm1_6 * oomegao2 - Bm2_6 * oomegao1;
    out(3, 0) = Bm5_1 * oomegao3 - Bm6_1 * oomegao2 + Bm2_1 * ovo3 - Bm3_1 * ovo2;
    out(3, 1) = -t9 - t21 * 2.0 + t36 + t37 + t42 + t43 + Bm5_2 * oomegao3 - Bm6_2 * oomegao2 + Bm2_2 * ovo3;
    out(3, 2) = t5 + t6 + t7 + t17 + t18 + t19 * 2.0 + Bm5_3 * oomegao3 - Bm6_3 * oomegao2 - Bm3_3 * ovo2;
    out(3, 3) = Bm5_4 * oomegao3 - Bm6_4 * oomegao2 + Bm2_4 * ovo3 - Bm3_4 * ovo2;
    out(3, 4) = -t13 - t29 - t30 - t31 + Bm5_5 * oomegao3 - Bm6_4 * oomegao1 - Bm6_5 * oomegao2 * 2.0 + Bm2_5 * ovo3 -
                Bm3_5 * ovo2;
    out(3, 5) = t12 + t26 + t27 + t28 + Bm5_4 * oomegao1 + Bm5_6 * oomegao3 * 2.0 - Bm6_6 * oomegao2 + Bm2_6 * ovo3 -
                Bm3_6 * ovo2;
    out(4, 0) = t8 + t9 + t10 + t20 * 2.0 + t21 + t22 - Bm4_1 * oomegao3 + Bm6_1 * oomegao1 - Bm1_1 * ovo3;
    out(4, 1) = -Bm4_2 * oomegao3 + Bm6_2 * oomegao1 - Bm1_2 * ovo3 + Bm3_2 * ovo1;
    out(4, 2) = -t4 - t16 * 2.0 + t32 + t33 + t38 + t39 - Bm4_3 * oomegao3 + Bm6_3 * oomegao1 + Bm3_3 * ovo1;
    out(4, 3) = t13 + t29 + t30 + t31 - Bm4_4 * oomegao3 + Bm6_4 * oomegao1 * 2.0 + Bm6_5 * oomegao2 - Bm1_4 * ovo3 +
                Bm3_4 * ovo1;
    out(4, 4) = -Bm4_5 * oomegao3 + Bm6_5 * oomegao1 - Bm1_5 * ovo3 + Bm3_5 * ovo1;
    out(4, 5) = -t11 - t23 - t24 - t25 - Bm4_5 * oomegao2 - Bm4_6 * oomegao3 * 2.0 + Bm6_6 * oomegao1 - Bm1_6 * ovo3 +
                Bm3_6 * ovo1;
    out(5, 0) = -t5 - t17 * 2.0 + t34 + t35 + t40 + t41 + Bm4_1 * oomegao2 - Bm5_1 * oomegao1 + Bm1_1 * ovo2;
    out(5, 1) = t2 + t3 + t4 + t14 + t15 * 2.0 + t16 + Bm4_2 * oomegao2 - Bm5_2 * oomegao1 - Bm2_2 * ovo1;
    out(5, 2) = Bm4_3 * oomegao2 - Bm5_3 * oomegao1 + Bm1_3 * ovo2 - Bm2_3 * ovo1;
    out(5, 3) = -t12 - t26 - t27 - t28 + Bm4_4 * oomegao2 - Bm5_4 * oomegao1 * 2.0 - Bm5_6 * oomegao3 + Bm1_4 * ovo2 -
                Bm2_4 * ovo1;
    out(5, 4) = t11 + t23 + t24 + t25 + Bm4_5 * oomegao2 * 2.0 + Bm4_6 * oomegao3 - Bm5_5 * oomegao1 + Bm1_5 * ovo2 -
                Bm2_5 * ovo1;
    out(5, 5) = Bm4_6 * oomegao2 - Bm5_6 * oomegao1 + Bm1_6 * ovo2 - Bm2_6 * ovo1;
  }

  //! Jacobian of the output function with respect to the state
  inline virtual void jacobx_output_fcn(const Eigen::Ref<const Eigen::Matrix<double, 27, 1>>& x,
                                        const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                        Eigen::Matrix<double, Eigen::Dynamic, 27>& out) const
  {
    (void)u_k;
    out.resize(number_pose_measure_from_robot_ * 14 + 12 + 14, 27);

    out.setZero();
    Eigen::Quaterniond bQo(x(3), x(4), x(5), x(6));
    bQo.normalize();

    // jacobian measures from robot 1
    Eigen::Matrix<double, 4, 4> b1Tb = bTb1_.inverse();
    Eigen::Matrix<double, 3, 3> Jp_1;  // jacobian position wrt position robot 1

    jacobian_output_to_position(b1Tb, Jp_1);

    Eigen::Matrix<double, 4, 4> JQ_1;  // jacobian quaternion wrt quaternion robot 1
    Eigen::Quaterniond bQb1(bTb1_.block<3, 3>(0, 0));
    jacobian_output_to_quaternion_right(bQb1.inverse(), JQ_1);

    Eigen::Matrix<double, 7, 13> output_J1_kth;
    output_J1_kth.setZero();
    output_J1_kth.block(0, 0, 3, 3) = Jp_1;
    output_J1_kth.block(3, 3, 4, 4) = JQ_1;
    output_J1_kth.block(0, 7, 7, 3) = Eigen::Matrix<double, 7, 3>::Zero();
    output_J1_kth.block(0, 10, 7, 3) = Eigen::Matrix<double, 7, 3>::Zero();

    // jacobian measures from robot 2
    Eigen::Matrix<double, 4, 4> b2Tb = b2Tb1_ * bTb1_.inverse();
    Eigen::Matrix<double, 3, 3> Jp_2;  // jacobian position wrt position robot 2

    jacobian_output_to_position(b2Tb, Jp_2);

    Eigen::Matrix<double, 4, 4> JQ_2;  // jacobian quaternion wrt quaternion robot 2
    Eigen::Quaterniond b2Qb1(b2Tb1_.block<3, 3>(0, 0));
    Eigen::Quaterniond b2Qb = b2Qb1 * bQb1.inverse();
    jacobian_output_to_quaternion_right(b2Qb, JQ_2);

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

    // Jacobian of the wrenches exerted by the robots on the object
    Eigen::Matrix<double, 12, 27> J_h_x;
    get_jacobian_h_to_state(x, u_k, J_h_x);
    out.block<12, 27>(7 * number_pose_measure_from_robot_ * 2, 0) = J_h_x;

    // Jacobian of the fkine of the robots with respect to their state
    out.block<14, 14>(7 * number_pose_measure_from_robot_ * 2 + 12, 13) = Eigen::Matrix<double, 14, 14>::Identity();
  }

  void jacobian_output_to_position(const Eigen::Ref<Eigen::Matrix<double, 4, 4>>& T,
                                   Eigen::Matrix<double, 3, 3>& out) const
  {
    // depends only from the rotation matrix between the base frame and the k-th base frame of the robot
    out.setZero();
    out.block<3, 3>(0, 0) = T.block<3, 3>(0, 0);
  }
  void jacobian_output_to_quaternion_right(const Eigen::Quaterniond& q, Eigen::Matrix<double, 4, 4>& out) const
  {
    double Q11, Q12, Q13, Q14, t2, t3, t4;
    Q11 = q.w();
    Q12 = q.x();
    Q13 = q.y();
    Q14 = q.z();
    t2 = -Q12;
    t3 = -Q13;
    t4 = -Q14;
    out(0, 0) = Q11;
    out(0, 1) = t2;
    out(0, 2) = t3;
    out(0, 3) = t4;
    out(1, 0) = Q12;
    out(1, 1) = Q11;
    out(1, 2) = t4;
    out(1, 3) = Q13;
    out(2, 0) = Q13;
    out(2, 1) = Q14;
    out(2, 2) = Q11;
    out(2, 3) = t2;
    out(3, 0) = Q14;
    out(3, 1) = t3;
    out(3, 2) = Q12;
    out(3, 3) = Q11;
  }

  void jacobian_output_to_quaternion_left(const Eigen::Quaterniond& q, Eigen::Matrix<double, 4, 4>& out) const
  {
    double Q21, Q22, Q23, Q24, t2, t3, t4;
    Q21 = q.w();
    Q22 = q.x();
    Q23 = q.y();
    Q24 = q.z();
    t2 = -Q22;
    t3 = -Q23;
    t4 = -Q24;
    out(0, 0) = Q21;
    out(0, 1) = t2;
    out(0, 2) = t3;
    out(0, 3) = t4;
    out(1, 0) = Q22;
    out(1, 1) = Q21;
    out(1, 2) = Q24;
    out(1, 3) = t3;
    out(2, 0) = Q23;
    out(2, 1) = t4;
    out(2, 2) = Q21;
    out(2, 3) = Q22;
    out(3, 0) = Q24;
    out(3, 1) = Q23;
    out(3, 2) = t2;
    out(3, 3) = Q21;
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
    std::cout << "RobotsSpringObjectSystem" << std::endl;

    std::cout << "State: " << x_.transpose() << std::endl;
    std::cout << "Output: " << y_.transpose() << std::endl;

    std::cout << "Inertia matrix: \n" << Bm_ << std::endl;
    std::cout << "Gravity vector: \n" << bg_.transpose() << std::endl;
    std::cout << "Grasp matrix: \n" << W_ << std::endl;
    std::cout << "Extended Rotation matrix: \n" << Rbar_ << std::endl;
    std::cout << "Transformation matrix from object to grasp 1: \n" << oTg1_ << std::endl;
    std::cout << "Transformation matrix from object to grasp 2: \n" << oTg2_ << std::endl;
    std::cout << "Transformation matrix from base 2 to base 1 of robots: \n" << b2Tb1_ << std::endl;
    std::cout << "Transformation matrix from base to base 1: \n" << bTb1_ << std::endl;
    std::cout << "Viscous friction matrix: \n" << viscous_friction_ << std::endl;
    std::cout << "Stiffness matrix for robot 1: \n" << K_1_ << std::endl;
    std::cout << "Damping matrix for robot 1: \n" << B_1_ << std::endl;
    std::cout << "Stiffness matrix for robot 2: \n" << K_2_ << std::endl;
    std::cout << "Damping matrix for robot 2: \n" << B_2_ << std::endl;
  }

  void set_mass(double mass)
  {
    Bm_.block<3, 3>(0, 0) = mass * Eigen::Matrix3d::Identity();
  }
  void set_gravity(const Eigen::Matrix<double, 3, 1>& bg)
  {
    bg_ = bg;
  }
  void set_ho_bias(const Eigen::Matrix<double, 6, 1>& ho_bias)
  {
    oh_bias_ = ho_bias;
  }

  void set_b2Tb1(const Eigen::Matrix4d& b2Tb1)
  {
    b2Tb1_ = b2Tb1;
  }

  void set_K_1(const Eigen::Matrix<double, 6, 6>& K_1)
  {
    K_1_ = K_1;
  }
  
  void set_B_1(const Eigen::Matrix<double, 6, 6>& B_1)
  {
    B_1_ = B_1;
  }

  void set_K_2(const Eigen::Matrix<double, 6, 6>& K_2)
  {
    K_2_ = K_2;
  }

  void set_B_2(const Eigen::Matrix<double, 6, 6>& B_2)
  {
    B_2_ = B_2;
  }

public:
  Eigen::Matrix<double, 27, 1> x_;              // state
  Eigen::Matrix<double, Eigen::Dynamic, 1> y_;  // output

  Eigen::Matrix<double, 6, 6> Bm_;      // Inertia matrix
  Eigen::Matrix<double, 3, 1> bg_;      // Gravity vector
  Eigen::Matrix<double, 6, 12> W_;      // Grasp matrix
  Eigen::Matrix<double, 12, 12> Rbar_;  // Extended Rotation matrix
  Eigen::Matrix4d oTg1_;                // Transformation matrix from object to grasp 1
  Eigen::Matrix4d oTg2_;                // Transformation matrix from object to grasp 2
  Eigen::Matrix4d b2Tb1_;               // Transformation matrix from base 2 to base 1 of robots (could be unknown)
  Eigen::Matrix4d bTb1_;                // Transformation matrix from base to base 1  (well known)
  Eigen::Matrix<double, 6, 6> viscous_friction_;  // Viscous friction matrix of the object-air
  Eigen::Matrix<double, 6, 6> K_1_;               // Stiffness matrix for robot 1
  Eigen::Matrix<double, 6, 6> B_1_;               // Damping matrix for robot 1
  Eigen::Matrix<double, 6, 6> K_2_;               // Stiffness matrix for robot 2
  Eigen::Matrix<double, 6, 6> B_2_;               // Damping matrix for robot 2

  Eigen::Matrix<double, 6, 1> oh_bias_;  // bias of the object wrench - to be estimated

  const int number_pose_measure_from_robot_;


};

}  // namespace uclv::systems