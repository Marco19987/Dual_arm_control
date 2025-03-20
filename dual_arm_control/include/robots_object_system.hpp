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
 system describing the interaction between 2 robots and one object
 the input to the system are the exerted wrenches by the robots
 applied in the grasp frames and expressed in the same frame
 the output of the system are the measured poses of the object in the
 robots base frame
 the state (13 elements) is represented by the pose of the object in the inertial
 frame an the twist in the object frame
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

  RobotsObjectSystem(const Eigen::Matrix<double, 13, 1>& x0, const Eigen::Matrix<double, 6, 6>& Bm,
                     const Eigen::Matrix<double, 3, 1>& bg, const Eigen::Matrix4d& oTg1, const Eigen::Matrix4d& oTg2,
                     const Eigen::Matrix4d& b2Tb1, const Eigen::Matrix4d& bTb1,
                     const Eigen::Matrix<double, 6, 6>& viscous_friction, const int number_pose_measure_from_robot)
    : x_(x0)
    , Bm_(Bm)
    , bg_(bg)
    , oTg1_(oTg1)
    , oTg2_(oTg2)
    , b2Tb1_(b2Tb1)
    , bTb1_(bTb1)
    , viscous_friction_(viscous_friction)
    , number_pose_measure_from_robot_(number_pose_measure_from_robot)
  {
    update_grasp_matrix();
    update_Rbar();
    y_.resize(number_pose_measure_from_robot_ * 14, 1);
    y_.setZero();
    oh_bias_.setZero();
  }

  RobotsObjectSystem() : number_pose_measure_from_robot_(0){};

  RobotsObjectSystem(const RobotsObjectSystem& sys) = default;

  virtual ~RobotsObjectSystem() = default;

  virtual RobotsObjectSystem* clone() const
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

  inline virtual const Eigen::Matrix<double, 13, 1>& get_state() const
  {
    return x_;
  }

  inline virtual const Eigen::Matrix<double, Eigen::Dynamic, 1>& get_output() const
  {
    return y_;
  }

  /*==============================================*/

  /*=============SETTER===========================*/

  inline virtual void set_state(const Eigen::Ref<const Eigen::Matrix<double, 13, 1>>& x)
  {
    x_ = x;
  }

  /*==============================================*/

  /*=============RUNNER===========================*/

  //! State function
  inline virtual void state_fcn(const Eigen::Ref<const Eigen::Matrix<double, 13, 1>>& x,
                                const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                Eigen::Matrix<double, 13, 1>& out) const
  {
    Eigen::Quaterniond bQo(x(3), x(4), x(5), x(6));
    bQo.normalize();

    Eigen::Matrix3d bRo = bQo.toRotationMatrix();  // object's rotation matrix in the base frame
    Eigen::Matrix<double, 6, 6> bRo_bar = Eigen::Matrix<double, 6, 6>::Zero();
    bRo_bar.block<3, 3>(0, 0) = bRo;
    bRo_bar.block<3, 3>(3, 3) = bRo;

    Eigen::Matrix<double, 6, 1> oh = W_ * Rbar_ * u_k;  // resulting wrench in the object frame

    std::cout << "oh: " << oh.transpose() << std::endl;
    std::cout << "oh_bias_: " << oh_bias_.transpose() << std::endl;


    oh = oh - oh_bias_;                                 // remove bias

    std::cout << "oh debiased: " << oh.transpose() << std::endl;


    Eigen::Matrix<double, 3, 1> ovo = x.block<3, 1>(7, 0);       // object's linear velocity in the object frame
    Eigen::Matrix<double, 3, 1> oomegao = x.block<3, 1>(10, 0);  // object's angular velocity in the object frame
    Eigen::Matrix<double, 6, 1> o_twist_o;
    o_twist_o << ovo, oomegao;  // object twist in the object frame

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

    std::cout << "oh debiased: " << (oh + Bm_ * bRo_bar.transpose() * bg_ext).transpose() << std::endl;


    out.block<3, 1>(0, 0) = bRo * ovo;
    Eigen::Quaterniond qdot;
    uclv::geometry_helper::quaternion_propagation(bQo, bRo * oomegao, qdot);
    out.block<4, 1>(3, 0) << qdot.w(), qdot.vec();
    out.block<6, 1>(7, 0) = o_twist_dot_o;
  }

  void get_object_wrench(const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                         Eigen::Matrix<double, 6, 1>& oh) const
  {
    oh = W_ * Rbar_ * u_k;  // resulting wrench in the object frame
  }

  //! Output function
  inline virtual void output_fcn(const Eigen::Ref<const Eigen::Matrix<double, 13, 1>>& x,
                                 const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                 Eigen::Matrix<double, Eigen::Dynamic, 1>& out) const
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
  }

  //! Jacobian of the state function with respect to the state
  inline virtual void jacobx_state_fcn(const Eigen::Ref<const Eigen::Matrix<double, 13, 1>>& x,
                                       const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                       Eigen::Matrix<double, 13, 13>& out) const
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
    out(0,0) = Bm2_1 * oomegao3 - Bm3_1 * oomegao2;
    out(0,1) = Bm2_2 * oomegao3 - Bm3_2 * oomegao2;
    out(0,2) = Bm2_3 * oomegao3 - Bm3_3 * oomegao2;
    out(0,3) = Bm2_4 * oomegao3 - Bm3_4 * oomegao2;
    out(0,4) = t9 * -2.0 - t21 + t36 + t37 + t42 + t43 + Bm2_5 * oomegao3;
    out(0,5) = t5 + t6 + t7 * 2.0 + t17 + t18 + t19 - Bm3_6 * oomegao2;
    out(1,0) = -Bm1_1 * oomegao3 + Bm3_1 * oomegao1;
    out(1,1) = -Bm1_2 * oomegao3 + Bm3_2 * oomegao1;
    out(1,2) = -Bm1_3 * oomegao3 + Bm3_3 * oomegao1;
    out(1,3) = t8 * 2.0 + t9 + t10 + t20 + t21 + t22 - Bm1_4 * oomegao3;
    out(1,4) = -Bm1_5 * oomegao3 + Bm3_5 * oomegao1;
    out(1,5) = t4 * -2.0 - t16 + t32 + t33 + t38 + t39 + Bm3_6 * oomegao1;
    out(2,0) = Bm1_1 * oomegao2 - Bm2_1 * oomegao1;
    out(2,1) = Bm1_2 * oomegao2 - Bm2_2 * oomegao1;
    out(2,2) = Bm1_3 * oomegao2 - Bm2_3 * oomegao1;
    out(2,3) = t5 * -2.0 - t17 + t34 + t35 + t40 + t41 + Bm1_4 * oomegao2;
    out(2,4) = t2 + t3 * 2.0 + t4 + t14 + t15 + t16 - Bm2_5 * oomegao1;
    out(2,5) = Bm1_6 * oomegao2 - Bm2_6 * oomegao1;
    out(3,0) = Bm5_1 * oomegao3 - Bm6_1 * oomegao2 + Bm2_1 * ovo3 - Bm3_1 * ovo2;
    out(3,1) = -t9 - t21 * 2.0 + t36 + t37 + t42 + t43 + Bm5_2 * oomegao3 - Bm6_2 * oomegao2 + Bm2_2 * ovo3;
    out(3,2) = t5 + t6 + t7 + t17 + t18 + t19 * 2.0 + Bm5_3 * oomegao3 - Bm6_3 * oomegao2 - Bm3_3 * ovo2;
    out(3,3) = Bm5_4 * oomegao3 - Bm6_4 * oomegao2 + Bm2_4 * ovo3 - Bm3_4 * ovo2;
    out(3,4) = -t13 - t29 - t30 - t31 + Bm5_5 * oomegao3 - Bm6_4 * oomegao1 - Bm6_5 * oomegao2 * 2.0 + Bm2_5 * ovo3 -
               Bm3_5 * ovo2;
    out(3,5) = t12 + t26 + t27 + t28 + Bm5_4 * oomegao1 + Bm5_6 * oomegao3 * 2.0 - Bm6_6 * oomegao2 + Bm2_6 * ovo3 -
               Bm3_6 * ovo2;
    out(4,0) = t8 + t9 + t10 + t20 * 2.0 + t21 + t22 - Bm4_1 * oomegao3 + Bm6_1 * oomegao1 - Bm1_1 * ovo3;
    out(4,1) = -Bm4_2 * oomegao3 + Bm6_2 * oomegao1 - Bm1_2 * ovo3 + Bm3_2 * ovo1;
    out(4,2) = -t4 - t16 * 2.0 + t32 + t33 + t38 + t39 - Bm4_3 * oomegao3 + Bm6_3 * oomegao1 + Bm3_3 * ovo1;
    out(4,3) = t13 + t29 + t30 + t31 - Bm4_4 * oomegao3 + Bm6_4 * oomegao1 * 2.0 + Bm6_5 * oomegao2 - Bm1_4 * ovo3 +
               Bm3_4 * ovo1;
    out(4,4) = -Bm4_5 * oomegao3 + Bm6_5 * oomegao1 - Bm1_5 * ovo3 + Bm3_5 * ovo1;
    out(4,5) = -t11 - t23 - t24 - t25 - Bm4_5 * oomegao2 - Bm4_6 * oomegao3 * 2.0 + Bm6_6 * oomegao1 - Bm1_6 * ovo3 +
               Bm3_6 * ovo1;
    out(5,0) = -t5 - t17 * 2.0 + t34 + t35 + t40 + t41 + Bm4_1 * oomegao2 - Bm5_1 * oomegao1 + Bm1_1 * ovo2;
    out(5,1) = t2 + t3 + t4 + t14 + t15 * 2.0 + t16 + Bm4_2 * oomegao2 - Bm5_2 * oomegao1 - Bm2_2 * ovo1;
    out(5,2) = Bm4_3 * oomegao2 - Bm5_3 * oomegao1 + Bm1_3 * ovo2 - Bm2_3 * ovo1;
    out(5,3) = -t12 - t26 - t27 - t28 + Bm4_4 * oomegao2 - Bm5_4 * oomegao1 * 2.0 - Bm5_6 * oomegao3 + Bm1_4 * ovo2 -
               Bm2_4 * ovo1;
    out(5,4) = t11 + t23 + t24 + t25 + Bm4_5 * oomegao2 * 2.0 + Bm4_6 * oomegao3 - Bm5_5 * oomegao1 + Bm1_5 * ovo2 -
               Bm2_5 * ovo1;
    out(5,5) = Bm4_6 * oomegao2 - Bm5_6 * oomegao1 + Bm1_6 * ovo2 - Bm2_6 * ovo1;
  }

  //! Jacobian of the output function with respect to the state
  inline virtual void jacobx_output_fcn(const Eigen::Ref<const Eigen::Matrix<double, 13, 1>>& x,
                                        const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& u_k,
                                        Eigen::Matrix<double, Eigen::Dynamic, 13>& out) const
  {
    (void)u_k;
    out.resize(number_pose_measure_from_robot_ * 14, 13);

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
    std::cout << "RobotsObjectSystem" << std::endl;

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

public:
  Eigen::Matrix<double, 13, 1> x_;              // state
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

  Eigen::Matrix<double, 6, 1> oh_bias_;  // bias of the object wrench - to be estimated

  const int number_pose_measure_from_robot_;
};

}  // namespace uclv::systems