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
#include "../include/robots/LBRiiwa7Ext.hpp"
#include <uclv_robot_lib/links/Revolute.hpp>

namespace uclv::robot
{

   LBRiiwa7Ext::LBRiiwa7Ext(const Eigen::Isometry3d& n_T_e, const std::string& name)
  : SerialLink(Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.340)), n_T_e, name)
{
  _model = LBRIIWA7EXT_MODEL_STR;
    // L1
  push_back_link(std::make_shared<link::Revolute>(
      // a,   alpha,     d,
      0.0, -M_PI / 2.0, 0.0,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -170.0 * M_PI / 180.0, 170.0 * M_PI / 180.0,
      // hard_velocity_limit
      1.71,
      // string name
      "A1"));

  // L2
  push_back_link(std::make_shared<link::Revolute>(
      // a,alpha,d,
      0.0, M_PI / 2.0, 0.0,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -120.0 * M_PI / 180.0, 120.0 * M_PI / 180.0,
      // hard_velocity_limit
      1.71,
      // string name
      "A2"));
  // L3
  push_back_link(std::make_shared<link::Revolute>(
      // a,alpha,d,
      0.0, -M_PI / 2.0, 0.400,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -170.0 * M_PI / 180.0, 170.0 * M_PI / 180.0,
      // hard_velocity_limit
      1.74,
      // string name
      "A3"));
  // L4
  push_back_link(std::make_shared<link::Revolute>(
      // a,alpha,d,
      0.0, M_PI / 2.0, 0.0,
      // robot2dh_offset, bool robot2dh_flip
      0.0, true,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -120.0 * M_PI / 180.0, 120.0 * M_PI / 180.0,
      // hard_velocity_limit
      2.26,
      // string name
      "A4"));
  // L5
  push_back_link(std::make_shared<link::Revolute>(
      // a,alpha,d,
      0.0, -M_PI / 2.0, 0.400,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -170.0 * M_PI / 180.0, 170.0 * M_PI / 180.0,
      // hard_velocity_limit
      2.44,
      // string name
      "A5"));
  // L6
  push_back_link(std::make_shared<link::Revolute>(
      // a,alpha,d,
      0.0, M_PI / 2.0, 0.0,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -120.0 * M_PI / 180.0, 120.0 * M_PI / 180.0,
      // hard_velocity_limit
      3.14,
      // string name
      "A6"));
  // L7
  push_back_link(std::make_shared<link::Revolute>(
      // a,alpha,d,
      0.0, M_PI/2, 0.126 + 0.2,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -175.0 * M_PI / 180.0, 175.0 * M_PI / 180.0,
      // hard_velocity_limit
      3.14,
      // string name
      "A7"));
  push_back_link(std::make_shared<link::Revolute>(
      // a,alpha,d,
      0.0, 0.0, 0.0,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -INFINITY, INFINITY,
      // hard_velocity_limit
      350.0 * M_PI / 180.0,
      // string name
      "Pivoting"));
}

/*
    Constructor with name only
*/
LBRiiwa7Ext::LBRiiwa7Ext(const std::string& name) : LBRiiwa7Ext(Eigen::Isometry3d::Identity(), name)
{
}

/*
    Empty constructor
*/
LBRiiwa7Ext::LBRiiwa7Ext() : LBRiiwa7Ext("LBRiiwa7Ext_NO_NAME")
{
}

} // namespace uclv::robot
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(uclv::robot::LBRiiwa7Ext, uclv::robot::SerialLinkInterface)