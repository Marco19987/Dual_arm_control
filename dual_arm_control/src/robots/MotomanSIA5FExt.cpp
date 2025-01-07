/*

    Robot Class for the Motoman SIA5F

    Copyright 2023 Università della Campania Luigi Vanvitelli

    Author: Marco Costanzo <marco.costanzo@unicampania.it>

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
#include "../include/robots/MotomanSIA5FExt.hpp"
#include <uclv_robot_lib/links/Revolute.hpp>

namespace uclv::robot
{

   MotomanSIA5FExt::MotomanSIA5FExt(const Eigen::Isometry3d& n_T_e, const std::string& name)
  : SerialLink(Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.3095)), n_T_e, name)
{
  _model = MOTOMANSIA5FEXT_MODEL_STR;
  // L1
  push_back_link(std::make_shared<link::Revolute>(
      // a,   alpha,     d,
      0.0, -M_PI / 2.0, 0.0,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -M_PI, M_PI,
      // hard_velocity_limit
      200.0 * M_PI / 180.0,
      // string name
      "S"));
  // L2
  push_back_link(std::make_shared<link::Revolute>(
      // a,alpha,d,
      0.0, M_PI / 2.0, 0.0,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -110.0 * M_PI / 180.0, 110.0 * M_PI / 180.0,
      // hard_velocity_limit
      200.0 * M_PI / 180.0,
      // string name
      "L"));
  // L3
  push_back_link(std::make_shared<link::Revolute>(
      // a,alpha,d,
      0.085, M_PI / 2.0, 0.27,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -170.0 * M_PI / 180.0, 170.0 * M_PI / 180.0,
      // hard_velocity_limit
      200.0 * M_PI / 180.0,
      // string name
      "E"));
  // L4
  push_back_link(std::make_shared<link::Revolute>(
      // a,alpha,d,
      0.06, M_PI / 2.0, 0.0,
      // robot2dh_offset, bool robot2dh_flip
      M_PI / 2.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -M_PI_2, 115.0 * M_PI / 180.0,
      // hard_velocity_limit
      200.0 * M_PI / 180.0,
      // string name
      "U"));
  // L5
  push_back_link(std::make_shared<link::Revolute>(
      // a,alpha,d,
      0.0, -M_PI / 2.0, 0.27,
      // robot2dh_offset, bool robot2dh_flip
      0.0, true,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -M_PI, M_PI,
      // hard_velocity_limit
      200.0 * M_PI / 180.0,
      // string name
      "R"));
  // L6
  push_back_link(std::make_shared<link::Revolute>(
      // a,alpha,d,
      0.0, M_PI / 2.0, 0.0,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -110.0 * M_PI / 180.0, 110.0 * M_PI / 180.0,
      // hard_velocity_limit
      230.0 * M_PI / 180.0,
      // string name
      "B"));
  // L7
  push_back_link(std::make_shared<link::Revolute>(
      // a,alpha,d,
      0.0, M_PI / 2.0, 0.148+0.20,
      // robot2dh_offset, bool robot2dh_flip
      0.0, true,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -M_PI, M_PI,
      // hard_velocity_limit
      350.0 * M_PI / 180.0,
      // string name
      "T"));
  push_back_link(std::make_shared<link::Revolute>(
      // a,alpha,d,
      0.0, 0.0, 0.0,
      // robot2dh_offset, bool robot2dh_flip
      0.0, true,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -INFINITY, INFINITY,
      // hard_velocity_limit
      INFINITY,
      // string name
      "Pivoting"));
}

/*
    Constructor with name only
*/
MotomanSIA5FExt::MotomanSIA5FExt(const std::string& name) : MotomanSIA5FExt(Eigen::Isometry3d::Identity(), name)
{
}

/*
    Empty constructor
*/
MotomanSIA5FExt::MotomanSIA5FExt() : MotomanSIA5FExt("SIA5F_NO_NAME")
{
}

} // namespace uclv::robot
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(uclv::robot::MotomanSIA5FExt, uclv::robot::SerialLinkInterface)