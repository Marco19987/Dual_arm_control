/*
    System interface Class Discrete Time System

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
 system describing the interaction between 2 robots and one object
 the input to the system are the exerted wrenches by the robots
 applied in the grasp frames and expressed in the same frame
 the output of the system are the measured poses of the object in the
 robots base frame
 the state is represented by the pose of the object in the inertial
 frame an the twist
 pose = [px py pz qx qy qz qw]';
 twist = [vx vy vz omegax omegay omegaz]';
*/

#pragma once

#include <uclv_systems_lib/continuous_time/continuous_time_linear_state_space.hpp>
#include <Eigen/Dense>
#include <memory>

namespace uclv::systems
{

    template <int dim_output>
    class RobotsObjectSystem : public ContinuousTimeStateSpaceInterface<12, 12, dim_output, 1, 1, 1>
    {
    public:
        using SharedPtr = std::shared_ptr<RobotsObjectSystem>;
        using ConstSharedPtr = std::shared_ptr<const RobotsObjectSystem>;
        using WeakPtr = std::weak_ptr<RobotsObjectSystem>;
        using ConstWeakPtr = std::weak_ptr<const RobotsObjectSystem>;
        using UniquePtr = std::unique_ptr<RobotsObjectSystem>;

        RobotsObjectSystem() = default;

        RobotsObjectSystem(const RobotsObjectSystem &sys) = default;

        virtual ~RobotsObjectSystem() = default;

        virtual RobotsObjectSystem *clone() const
        {
            return new RobotsObjectSystem(*this);
        }

        inline virtual const Eigen::Matrix<double, 12, 1> &get_state() const
        {
            return x_;
        }

        void update_grasp_matrix()
        {
            Eigen::Vector3d opg1 = oTg1_.block<3, 1>(0, 3);
            Eigen::Vector3d opg2 = oTg2_.block<3, 1>(0, 3);
            Eigen::Matrix<double, 6, 6> Wg1;
            Wg1 << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
                -skew(opg1).transpose(), Eigen::Matrix3d::Identity();
            Eigen::Matrix<double, 6, 6> Wg2;
            Wg2 << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
                -skew(opg2).transpose(), Eigen::Matrix3d::Identity();
            W_.block<6, 6>(0, 0) = Wg1;
            W_.block<6, 6>(0, 6) = Wg2;
        }

        void update_Rbar()
        {
            Eigen::Matrix3d oRg1 = oTg1_.block<3, 3>(0, 0);
            Eigen::Matrix3d oRg2 = oTg2_.block<3, 3>(0, 0);
            Rbar_.block<3, 3>(0, 0) = oRg1;
            Rbar_.block<3, 3>(3, 3) = oRg1;
            Rbar_.block<3, 3>(6, 6) = oRg2;
            Rbar_.block<3, 3>(9, 9) = oRg2;
        }

        inline virtual const Eigen::Matrix<double, 12, 1> &get_state() const
        {
            return x_;
        }

        inline virtual const Eigen::Matrix<double, dim_output, 1> &get_output() const
        {
            return y_;
        }

        /*==============================================*/

        /*=============SETTER===========================*/

        inline virtual void set_state(const Eigen::Ref<const Eigen::Matrix<double, 12, 1>> &x)
        {
            x_ = x;
        }

        /*==============================================*/

        /*=============RUNNER===========================*/

        //! State function
        inline virtual void state_fcn(const Eigen::Ref<const Eigen::Matrix<double, 12, 1>> &x,
                                      const Eigen::Ref<const Eigen::Matrix<double, 12, 1>> &u_k,
                                      Eigen::Ref<Eigen::Matrix<double, 12, 1>> out) const
        {
        }

        //! Output function
        inline virtual void output_fcn(const Eigen::Ref<const Eigen::Matrix<double, 12, 1>> &x,
                                       const Eigen::Ref<const Eigen::Matrix<double, 12, 1>> &u_k,
                                       Eigen::Ref<Eigen::Matrix<double, dim_output, 1>> out) const
        {
        }

        //! Jacobian of the state function with respect to the state
        inline virtual void jacobx_state_fcn(const Eigen::Ref<const Eigen::Matrix<double, 12, 1>> &x,
                                             const Eigen::Ref<const Eigen::Matrix<double, 12, 1>> &u_k,
                                             Eigen::Ref<Eigen::Matrix<double, 12, 12>> out) const
        {
            (void)x;
            (void)u_k;
            (void)out;
            if (1 != 1)
            {
                throw std::runtime_error("The Jacobian of the state function is not defined for 1 != 1");
            }
        }

        //! Jacobian of the output function with respect to the state
        inline virtual void jacobx_output_fcn(const Eigen::Ref<const Eigen::Matrix<double, 12, 1>> &x,
                                              const Eigen::Ref<const Eigen::Matrix<double, 12, 1>> &u_k,
                                              Eigen::Ref<Eigen::Matrix<double, dim_output, 12>> out) const
        {
            (void)x;
            (void)u_k;
            (void)out;
            if (1 != 1)
            {
                throw std::runtime_error("The Jacobian of the output function is not defined for 1 != 1");
            }
        }

        inline virtual void reset()
        {
            x_.setZero();
            y_.setZero();
        }

        virtual unsigned int get_size_state() const
        {
            return 12;
        }

        virtual unsigned int get_size1_state() const
        {
            return 12;
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

            std::cout << "Inertia matrix: " << Bm_ << std::endl;
            std::cout << "Gravity vector: " << bg_.transpose() << std::endl;
            std::cout << "Grasp matrix: " << W_ << std::endl;
            std::cout << "Extended Rotation matrix: " << Rbar_ << std::endl;
            std::cout << "Transformation matrix from object to grasp 1: " << oTg1_ << std::endl;
            std::cout << "Transformation matrix from object to grasp 2: " << oTg2_ << std::endl;
            std::cout << "Transformation matrix from base 1 to base 2 of robots: " << b1Tb2_ << std::endl;
            std::cout << "Transformation matrix from base to base 1: " << bTb1_ << std::endl;
            std::cout << "Viscous friction matrix: " << viscous_friction_ << std::endl;
        }

    private:
        Eigen::Matrix<double, 12, 1> x_;         // state
        Eigen::Matrix<double, dim_output, 1> y_; // output

        Eigen::Matrix<double, 6, 6> Bm_;               // Inertia matrix
        Eigen::Matrix<double, 6, 1> bg_;               // Gravity vector
        Eigen::Matrix<double, 6, 12> W_;               // Grasp matrix
        Eigen::Matrix<double, 12, 12> Rbar_;           // Extended Rotation matrix
        Eigen::Matrix4d oTg1_;                         // Transformation matrix from object to grasp 1
        Eigen::Matrix4d oTg2_;                         // Transformation matrix from object to grasp 2
        Eigen::Matrix4d b1Tb2_;                        // Transformation matrix from base 1 to base 2 of robots (could be unknown)
        Eigen::Matrix4d bTb1_;                         // Transformation matrix from base to base 1  (well known)
        Eigen::Matrix<double, 6, 6> viscous_friction_; // Viscous friction matrix of the object-air
    };

} // namespace uclv::systems