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
#pragma once

#include <uclv_robot_lib/SerialLink.hpp>
#define MOTOMANSIA5FEXT_MODEL_STR "MotomanSIA5FExt"

namespace uclv::robot
{
    class MotomanSIA5FExt : public SerialLink
    {
    public:
        /*!
            Full constructor
        */
        MotomanSIA5FExt(const Eigen::Isometry3d& n_T_e, const std::string& name);

        /*!
            Constructor with name only
        */
        MotomanSIA5FExt(const std::string &name);

        /*!
            Empty constructor
        */
        MotomanSIA5FExt();
    };

} 