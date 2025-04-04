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

namespace uclv::systems
{

  class RobotsObjectSystemExt : public ContinuousTimeStateSpaceInterface<20, 12, Eigen::Dynamic, 1, 1, 1>
  {
  public:
    using SharedPtr = std::shared_ptr<RobotsObjectSystemExt>;
    using ConstSharedPtr = std::shared_ptr<const RobotsObjectSystemExt>;
    using WeakPtr = std::weak_ptr<RobotsObjectSystemExt>;
    using ConstWeakPtr = std::weak_ptr<const RobotsObjectSystemExt>;
    using UniquePtr = std::unique_ptr<RobotsObjectSystemExt>;

    RobotsObjectSystemExt(const Eigen::Matrix<double, 20, 1> &x0,
                          typename uclv::systems::RobotsObjectSystem::SharedPtr robots_object_system_ptr)
        : x_(x0), robots_object_system_ptr_(robots_object_system_ptr), number_pose_measure_from_robot_(robots_object_system_ptr->number_pose_measure_from_robot_)
    {
      y_.resize(number_pose_measure_from_robot_ * 14, 1);
      y_.setZero();
    }

    RobotsObjectSystemExt() = default;

    RobotsObjectSystemExt(const RobotsObjectSystemExt &sys) = default;

    virtual ~RobotsObjectSystemExt() = default;

    virtual RobotsObjectSystemExt *clone() const
    {
      return new RobotsObjectSystemExt(*this);
    }

    inline virtual const Eigen::Matrix<double, 20, 1> &get_state() const
    {
      return x_;
    }

    inline virtual const Eigen::Matrix<double, Eigen::Dynamic, 1> &get_output() const
    {
      return y_;
    }

    /*==============================================*/

    /*=============SETTER===========================*/

    inline virtual void set_state(const Eigen::Ref<const Eigen::Matrix<double, 20, 1>> &x)
    {
      x_ = x;
    }

    /*==============================================*/

    /*=============RUNNER===========================*/

    //! State function
    inline virtual void state_fcn(const Eigen::Ref<const Eigen::Matrix<double, 20, 1>> &x,
                                  const Eigen::Ref<const Eigen::Matrix<double, 12, 1>> &u_k,
                                  Eigen::Matrix<double, 20, 1> &out) const
    {
      out.setZero();
      // the state is composed by the state of the robots_object_system and the transformation matrix b2Tb1_
      Eigen::Matrix<double, 13, 1> x_out;
      robots_object_system_ptr_->state_fcn(x.block<13, 1>(0, 0), u_k, x_out);
      out.block<13, 1>(0, 0) = x_out;

      // the transformation matrix b2Tb1_, stored in the state as position and quaternion is assumed constant, so the
      // derivative is zero and the state is not modified
      out.block<7, 1>(13, 0) = Eigen::Matrix<double, 7, 1>::Zero();
    }

    //! Output function
    inline virtual void output_fcn(const Eigen::Ref<const Eigen::Matrix<double, 20, 1>> &x,
                                   const Eigen::Ref<const Eigen::Matrix<double, 12, 1>> &u_k,
                                   Eigen::Matrix<double, Eigen::Dynamic, 1> &out) const
    {
      (void)u_k;
      out.resize(number_pose_measure_from_robot_ * 14, 1);
      out.setZero();

      // update b2Tb1 in the robots_object_system
      Eigen::Matrix<double, 4, 4> b2Tb1;
      uclv::geometry_helper::pose_to_matrix(x.block<7, 1>(13, 0),b2Tb1);
      robots_object_system_ptr_->set_b2Tb1(b2Tb1);
    
      // the output is composed by the output of the robots_object_system
      robots_object_system_ptr_->output_fcn(x.block<13, 1>(0, 0), u_k, out);

    }

    //! Jacobian of the state function with respect to the state
    inline virtual void jacobx_state_fcn(const Eigen::Ref<const Eigen::Matrix<double, 20, 1>> &x,
                                         const Eigen::Ref<const Eigen::Matrix<double, 12, 1>> &u_k,
                                         Eigen::Matrix<double, 20, 20> &out) const
    {
      // the jacobian of the state function is the jacobian of the robots_object_system
      // the jacobian of the transformation matrix b2Tb1_ is zero
      out.setZero();
      Eigen::Matrix<double, 13, 13> Jx;
      robots_object_system_ptr_->jacobx_state_fcn(x.block<13, 1>(0, 0), u_k, Jx);
      out.block<13, 13>(0, 0) = Jx;
    }

    //! Jacobian of the output function with respect to the state
    inline virtual void jacobx_output_fcn(const Eigen::Ref<const Eigen::Matrix<double, 20, 1>> &x,
                                          const Eigen::Ref<const Eigen::Matrix<double, 12, 1>> &u_k,
                                          Eigen::Matrix<double, Eigen::Dynamic, 20> &out) const
    {
      (void)u_k;
      out.resize(number_pose_measure_from_robot_ * 14, 20);
      out.setZero();

      // update b2Tb1 in the robots_object_system
      Eigen::Matrix<double, 4, 4> b2Tb1;
      uclv::geometry_helper::pose_to_matrix(x.block<7, 1>(13, 0),b2Tb1);
      robots_object_system_ptr_->set_b2Tb1(b2Tb1);


      Eigen::Quaterniond bQo(x(3), x(4), x(5), x(6));
      bQo.normalize();

      Eigen::Quaterniond b2Qb1(x(16), x(17), x(18), x(19));
      b2Qb1.normalize();


      // jacobian measures from robot 1
      Eigen::Matrix<double, 4, 4> bTb1 = robots_object_system_ptr_->bTb1_;
      Eigen::Quaterniond bQb1(bTb1.block<3, 3>(0, 0));
      Eigen::Quaterniond b1Qo = bQb1.inverse() * bQo;
      b1Qo.normalize();

      Eigen::Matrix<double, Eigen::Dynamic, 13> jacob_base; 
      robots_object_system_ptr_->jacobx_output_fcn(x.block<13, 1>(0, 0), u_k, jacob_base);
      out.block(0, 0, number_pose_measure_from_robot_ * 14, 13) = jacob_base;


      // jacobian measures from robot 2
      Eigen::Matrix<double, 4, 4> bTo;
      uclv::geometry_helper::pose_to_matrix(x.block<7, 1>(0, 0),bTo);
      Eigen::Matrix<double, 4, 4> b1To = bTb1 * bTo;
      
      Eigen::Matrix<double, 3, 3> J_b2po_b2pb1; 
      J_b2po_b2pb1 = Eigen::Matrix<double, 3, 3>::Identity();

      Eigen::Matrix<double, 3, 4> J_b2po_b2Qb1;
      Jacobian_Rp_to_Q(b2Qb1, b1To.block<3,1>(0,3),J_b2po_b2Qb1);

      Eigen::Matrix<double, 4, 4> J_b2Qo_b2Qb1; 
      robots_object_system_ptr_->jacobian_output_to_quaternion_left(b1Qo,J_b2Qo_b2Qb1);



      Eigen::Matrix<double, 7, 7> output_J2_kth;
      output_J2_kth.setZero();
      output_J2_kth.block(0, 0, 3, 3) = J_b2po_b2pb1;   
      output_J2_kth.block(0, 3, 3, 4) = J_b2po_b2Qb1;  
      output_J2_kth.block(3, 3, 4, 4) = J_b2Qo_b2Qb1; 

      for (int i = 0; i < number_pose_measure_from_robot_; i++)
      {
        out.block((i + number_pose_measure_from_robot_) * 7, 13, 7, 7) = output_J2_kth;
      }
    }
   

    void Jacobian_Rp_to_Q(const Eigen::Quaterniond &q, const Eigen::Matrix<double,3,1> p,Eigen::Matrix<double, 3, 4> &out) const
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

    inline virtual void reset()
    {
      x_.setZero();
      y_.setZero();
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

      robots_object_system_ptr_->display();
    }

  public:
    Eigen::Matrix<double, 20, 1> x_;             // state
    Eigen::Matrix<double, Eigen::Dynamic, 1> y_; // output

    typename uclv::systems::RobotsObjectSystem::SharedPtr robots_object_system_ptr_;
    const int number_pose_measure_from_robot_;
  };

} // namespace uclv::systems