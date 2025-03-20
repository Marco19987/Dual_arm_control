/*

    Copyright 3224 Università della Campania Luigi Vanvitelli

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
This class is a extension of the robots_object_system_ext.hpp file.
To the state has been added the wrenches exerted by the robots and
they are considered to be constant.
In the output equations the wrenches are measured directly.
*/

#pragma once

#include <uclv_systems_lib/continuous_time/continuous_time_linear_state_space.hpp>
#include "geometry_helper.hpp"
#include <Eigen/Dense>
#include <memory>

namespace uclv::systems
{

class RobotsObjectSystemExtForces : public ContinuousTimeStateSpaceInterface<32, 0, Eigen::Dynamic, 1, 1, 1>
{
public:
  using SharedPtr = std::shared_ptr<RobotsObjectSystemExtForces>;
  using ConstSharedPtr = std::shared_ptr<const RobotsObjectSystemExtForces>;
  using WeakPtr = std::weak_ptr<RobotsObjectSystemExtForces>;
  using ConstWeakPtr = std::weak_ptr<const RobotsObjectSystemExtForces>;
  using UniquePtr = std::unique_ptr<RobotsObjectSystemExtForces>;

  RobotsObjectSystemExtForces(const Eigen::Matrix<double, 32, 1>& x0,
                              typename uclv::systems::RobotsObjectSystemExt::SharedPtr robots_object_system_ext_ptr)
    : x_(x0)
    , robots_object_system_ext_ptr_(robots_object_system_ext_ptr)
    , number_pose_measure_from_robot_(robots_object_system_ext_ptr->number_pose_measure_from_robot_)
  {
    y_.resize(number_pose_measure_from_robot_ * 14, 1);
    y_.setZero();
  }

  RobotsObjectSystemExtForces() = default;

  RobotsObjectSystemExtForces(const RobotsObjectSystemExtForces& sys) = default;

  virtual ~RobotsObjectSystemExtForces() = default;

  virtual RobotsObjectSystemExtForces* clone() const
  {
    return new RobotsObjectSystemExtForces(*this);
  }

  inline virtual const Eigen::Matrix<double, 32, 1>& get_state() const
  {
    return x_;
  }

  inline virtual const Eigen::Matrix<double, Eigen::Dynamic, 1>& get_output() const
  {
    return y_;
  }

  /*==============================================*/

  /*=============SETTER===========================*/

  inline virtual void set_state(const Eigen::Ref<const Eigen::Matrix<double, 32, 1>>& x)
  {
    x_ = x;
  }

  /*==============================================*/

  /*=============RUNNER===========================*/

  //! State function
  inline virtual void state_fcn(const Eigen::Ref<const Eigen::Matrix<double, 32, 1>>& x,
                                const Eigen::Ref<const Eigen::Matrix<double, 0, 1>>& u_k,
                                Eigen::Matrix<double, 32, 1>& out) const
  {
    out.setZero();
    // the state is composed by the state of the robots_object_system and the transformation matrix b2Tb1_
    Eigen::Matrix<double, 20, 1> x_out;
    robots_object_system_ext_ptr_->state_fcn(x.block<20, 1>(0, 0), x.block<12, 1>(20, 0), x_out);
    out.block<20, 1>(0, 0) = x_out;

    // wrenches assumed to be constant
    out.block<12, 1>(20, 0) = Eigen::Matrix<double, 12, 1>::Zero();
  }

  //! Output function
  inline virtual void output_fcn(const Eigen::Ref<const Eigen::Matrix<double, 32, 1>>& x,
                                 const Eigen::Ref<const Eigen::Matrix<double, 0, 1>>& u_k,
                                 Eigen::Matrix<double, Eigen::Dynamic, 1>& out) const
  {
    (void)u_k;
    out.resize(number_pose_measure_from_robot_ * 14 + 12, 1);
    out.setZero();

    Eigen::Matrix<double, Eigen::Dynamic, 1> base_system_out;
    robots_object_system_ext_ptr_->output_fcn(x.block<20, 1>(0, 0), x.block<12, 1>(20, 0), base_system_out);
    out.block(0, 0, number_pose_measure_from_robot_ * 14, 1) = base_system_out;

    // wrenches assumed to be directly measured
    out.block(number_pose_measure_from_robot_ * 14, 0, 12, 1) = x.block<12, 1>(20, 0);
  }

  //! Jacobian of the state function with respect to the state
  inline virtual void jacobx_state_fcn(const Eigen::Ref<const Eigen::Matrix<double, 32, 1>>& x,
                                       const Eigen::Ref<const Eigen::Matrix<double, 0, 1>>& u_k,
                                       Eigen::Matrix<double, 32, 32>& out) const
  {
    // the jacobian of the state function is the jacobian of the robots_object_system
    out.setZero();
  
    Eigen::Matrix<double,12,12> Rbar = robots_object_system_ext_ptr_->robots_object_system_ptr_->Rbar_;
    Eigen::Matrix<double,6,12> W = robots_object_system_ext_ptr_->robots_object_system_ptr_->W_;
    Eigen::Matrix<double,6,6> Bm = robots_object_system_ext_ptr_->robots_object_system_ptr_->Bm_;
    out.block<6,12>(7, 20) = Bm.inverse() * (W * Rbar);


    Eigen::Matrix<double, 20, 20> Jx;
    robots_object_system_ext_ptr_->jacobx_state_fcn(x.block<20, 1>(0, 0), x.block<12, 1>(20, 0), Jx);
    out.block<20,20>(0, 0) = Jx;


  }

  //! Jacobian of the output function with respect to the state
  inline virtual void jacobx_output_fcn(const Eigen::Ref<const Eigen::Matrix<double, 32, 1>>& x,
                                        const Eigen::Ref<const Eigen::Matrix<double, 0, 1>>& u_k,
                                        Eigen::Matrix<double, Eigen::Dynamic, 32>& out) const
  {
    (void)u_k;
    out.resize(number_pose_measure_from_robot_ * 14 + 12, 32);
    out.setZero();

    Eigen::Matrix<double, Eigen::Dynamic, 20> base_system_jacob_out;
    robots_object_system_ext_ptr_->jacobx_output_fcn(x.block<20, 1>(0, 0), x.block<12, 1>(20, 0),base_system_jacob_out);
    out.block(0, 0, number_pose_measure_from_robot_ * 14, 20) = base_system_jacob_out;

    // wrenches assumed to be constant -> jacobian is I
    out.block(number_pose_measure_from_robot_ * 14, 20, 12, 12) = Eigen::Matrix<double, 12, 12>::Identity();

  }

  inline virtual void reset()
  {
    x_.setZero();
    y_.setZero();
  }

  virtual unsigned int get_size_state() const
  {
    return 32;
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
    std::cout << "RobotsObjectSystemExtForces" << std::endl;

    std::cout << "State: " << x_.transpose() << std::endl;
    std::cout << "Output: " << y_.transpose() << std::endl;

    robots_object_system_ext_ptr_->display();
  }

private:
  Eigen::Matrix<double, 32, 1> x_;              // state
  Eigen::Matrix<double, Eigen::Dynamic, 1> y_;  // output

  typename uclv::systems::RobotsObjectSystemExt::SharedPtr robots_object_system_ext_ptr_;
  const int number_pose_measure_from_robot_;
};

}  // namespace uclv::systems