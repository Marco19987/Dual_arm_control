#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <uclv_robot_ros_msgs/msg/matrix.hpp>

#include <realtime_tools/thread_priority.hpp>

/*
 * This node implements the inverse differential kinematics for a two robots system
 * using the absolute and relative coordinates described in the paper: Chiacchio96
 * Direct and Inverse Kinematics for Coordinated Motion of a Two-Manipulator System
 *
 *
 */

class CooperativeRobotsServer : public rclcpp::Node
{
public:
    CooperativeRobotsServer(const rclcpp::NodeOptions &options) : Node("cooperative_robots_server", options)
    {
        // Declare parameters
        declare_ros_parameters();

        initRealTime();

        int index = 0;
        std::string robot1_prefix = this->get_parameter("robot1_prefix").as_string();
        sub_jacobian_robot1_ = this->create_subscription<uclv_robot_ros_msgs::msg::Matrix>(
            robot1_prefix + "/jacobian", rclcpp::SensorDataQoS(),
            [this, index](const uclv_robot_ros_msgs::msg::Matrix::SharedPtr msg)
            { this->jacobianCallback(msg, index); });

        index = 1;
        std::string robot2_prefix = this->get_parameter("robot2_prefix").as_string();
        sub_jacobian_robot2_ = this->create_subscription<uclv_robot_ros_msgs::msg::Matrix>(
            robot2_prefix + "/jacobian", rclcpp::SensorDataQoS(),
            [this, index](const uclv_robot_ros_msgs::msg::Matrix::SharedPtr msg)
            { this->jacobianCallback(msg, index); });

        pub_joint_state_robot1_ =
            this->create_publisher<sensor_msgs::msg::JointState>(robot1_prefix + "/command/joint_vel_states", rclcpp::SensorDataQoS());

        pub_joint_state_robot2_ =
            this->create_publisher<sensor_msgs::msg::JointState>(robot2_prefix + "/command/joint_vel_states", rclcpp::SensorDataQoS());

        index = 0;
        sub_absolute_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "absolute_twist", rclcpp::SensorDataQoS(),
            [this, index](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
            { this->twistCallback(msg, index); });

        index = 1;
        sub_relative_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "relative_twist", rclcpp::SensorDataQoS(),
            [this, index](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
            { this->twistCallback(msg, index); });

        // Initialize variables
        absolute_twist_.setZero();
        relative_twist_.setZero();
    }

    void initRealTime()
    {
        if ((realtime_priority_ > 0) && realtime_tools::has_realtime_kernel())
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "set REALTIME to " << realtime_priority_);
            if (!realtime_tools::configure_sched_fifo(realtime_priority_))
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "SET REALTIME FAILED ");
                throw std::runtime_error("REALTIME REQUIRED, BUT FAILED TO SET");
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "SET REALTIME OK ");
        }
    }

    void jacobianCallback(const uclv_robot_ros_msgs::msg::Matrix::ConstSharedPtr msg, int index)
    {
        assert(msg->dim.size() == 2 && "Jacobian must be a matrix");
        assert(msg->dim[0] == 6 && "Jacobian must have 6 rows");

        if (index == 0)
        {
            jacobian_robot1_ = msg;
            robot1_joints_number_ = msg->dim[1];
        }
        else
        {
            jacobian_robot2_ = msg;
            robot2_joints_number_ = msg->dim[1];
        }
    }

    void twistCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg, int index)
    {
        if (index == 0)
        {
            absolute_twist_[0] = msg->twist.linear.x;
            absolute_twist_[1] = msg->twist.linear.y;
            absolute_twist_[2] = msg->twist.linear.z;
            absolute_twist_[3] = msg->twist.angular.x;
            absolute_twist_[4] = msg->twist.angular.y;
            absolute_twist_[5] = msg->twist.angular.z;
        }
        else
        {
            relative_twist_[0] = msg->twist.linear.x;
            relative_twist_[1] = msg->twist.linear.y;
            relative_twist_[2] = msg->twist.linear.z;
            relative_twist_[3] = msg->twist.angular.x;
            relative_twist_[4] = msg->twist.angular.y;
            relative_twist_[5] = msg->twist.angular.z;
        }
        compute_qdot(msg);
    }

    void rotate_jacobian(const uclv_robot_ros_msgs::msg::Matrix::ConstSharedPtr jacobian, const Eigen::Matrix<double, 4, 4> &T, Eigen::Matrix<double, 6, Eigen::Dynamic> &J_rotated)
    {
        Eigen::Matrix<double, 6, 6> Rext;
        Rext.block<3, 3>(0, 0) = T.block<3, 3>(0, 0);
        Rext.block<3, 3>(3, 3) = T.block<3, 3>(0, 0);
        if (jacobian->row_major)
        {
            Eigen::Map<const Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::RowMajor>> J(
                jacobian->data.data(), jacobian->dim[0], jacobian->dim[1]);
            J_rotated.resize(6, J.cols());
            J_rotated.setZero();
            J_rotated = Rext * J;
        }
        else
        {
            Eigen::Map<const Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::ColMajor>> J(
                jacobian->data.data(), jacobian->dim[0], jacobian->dim[1]);
            J_rotated.resize(6, J.cols());
            J_rotated.setZero();
            J_rotated = Rext * J;
        }
    }

    void compute_qdot(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
    {
        // check if jacobians are received
        std::cout << "Computing q_dot" << std::endl;

        if (jacobian_robot1_ && jacobian_robot2_)
        {
            std::cout << "Computing q_dot - STEP rotate jacobian 1" << std::endl;
            // rotate jacobian 1 to the base frame
            Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_robot1_base; // Jacobian robot 1 base frame rotated in the common base frame
            rotate_jacobian(jacobian_robot1_, bTb1_, jacobian_robot1_base);
            std::cout << "Jacobian 1: " << jacobian_robot1_base << std::endl;

            std::cout << "Computing q_dot - STEP rotate jacobian 2" << std::endl;

            // rotate jacobian 2 to the base frame
            Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_robot2_base; // Jacobian robot 2 base frame rotated in the common base frame
            Eigen::Matrix<double, 4, 4> bTb2 = bTb1_ * b1Tb2_;
            rotate_jacobian(jacobian_robot2_, bTb2, jacobian_robot2_base);

            std::cout << "Jacobian 2: " << jacobian_robot2_base << std::endl;

                        std::cout << "Computing q_dot - STEP absolute " << std::endl;


            // define absolute jacobian
            Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_absolute; // Ja = 0.5 * [J1, J2];
            jacobian_absolute.resize(6, jacobian_robot1_base.cols() + jacobian_robot2_base.cols());
            jacobian_absolute << 0.5*jacobian_robot1_base, 0.5*jacobian_robot2_base;


            std::cout << "Absolute jacobian: " << jacobian_absolute << std::endl;

                                    std::cout << "Computing q_dot - STEP relative " << std::endl;


            // define relative jacobian
            Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_relative; // Jr = [-J1 J2];
            jacobian_relative.resize(6, jacobian_robot1_base.cols() + jacobian_robot2_base.cols());
            jacobian_relative << -jacobian_robot1_base, jacobian_robot2_base;

            std::cout << "Relative jacobian: " << jacobian_relative << std::endl;

                                    std::cout << "Computing q_dot - STEP complete " << std::endl;


            // define complete jacobian
            Eigen::Matrix<double, 12, Eigen::Dynamic> jacobian_complete; // J = [Ja;Jr];
            jacobian_complete.resize(12, jacobian_absolute.cols());
            jacobian_complete << jacobian_absolute, jacobian_relative;

            std::cout << "Complete jacobian: " << jacobian_complete << std::endl;

                                    std::cout << "Computing q_dot - STEP twits " << std::endl;


            // define complete twist
            Eigen::Matrix<double, 12, 1> twist_complete; // twist = [v1;w1;v2;w2];
            twist_complete << absolute_twist_, relative_twist_;

            std::cout << "Complete twist: " << twist_complete << std::endl;

                                    std::cout << "Computing q_dot - STEP inverse kinematics " << std::endl;


            // solve inverse kinematics
            Eigen::Matrix<double, Eigen::Dynamic, 1> q_dot;
            q_dot.resize(jacobian_complete.cols());
            q_dot.setZero();
            q_dot = jacobian_complete.completeOrthogonalDecomposition().solve(twist_complete);

            std::cout << "q_dot: " << q_dot << std::endl;

                                    std::cout << "Computing q_dot - STEP vel control " << std::endl;


            // check velocity limits
            // if ((Eigen::Index)joint_vel_limits_robot1_.size() != robot1_joints_number_ || (Eigen::Index)joint_vel_limits_robot2_.size() != robot2_joints_number_)
            // {
            //     abort_inverse_kinematics(
            //         "VEL LIMITS SIZE MISMATCH q_size: " + std::to_string(q_dot.size()) +
            //         " -- joint_vel_limits_size: " + std::to_string(joint_vel_limits_robot1_.size()) +
            //         "-- joint_vel_limits_size: " + std::to_string(joint_vel_limits_robot2_.size()));
            // }

            //                         std::cout << "Computing q_dot - STEP vel control 2 " << std::endl;

            // for (int i = 0; i < q_dot.size(); i++)
            // {                  
            //      std::cout << "Computing q_dot - STEP vel control  iiii" << std::endl;

            //     if (i < robot1_joints_number_)
            //     {
            //                          std::cout << "Computing q_dot - STEP vel control  pppp" << std::endl;

            //         if (fabs(q_dot(i)) > joint_vel_limits_robot1_[i])
            //         {
            //             abort_inverse_kinematics("VEL LIMITS VIOLATED ROBOT 1");
            //         }
            //     }
            //     else
            //     {
            //                          std::cout << "Computing q_dot - STEP vel control  ooooo" << std::endl;

            //         if (fabs(q_dot(i)) > joint_vel_limits_robot2_[i-robot1_joints_number_])
            //         {
            //             abort_inverse_kinematics("VEL LIMITS VIOLATED ROBOT 2");
            //         }
            //     }
            // }

                                    std::cout << "Computing q_dot - STEP publishing " << std::endl;


            // publish result
            auto joint_state_robot1 = std::make_unique<sensor_msgs::msg::JointState>();
            joint_state_robot1->header = msg->header;

            joint_state_robot1->name = joint_names_robot1_;
            joint_state_robot1->velocity.resize(robot1_joints_number_);
            for (int i = 0; i < robot1_joints_number_; i++)
            {
                joint_state_robot1->velocity[i] = q_dot[i];
            }

            pub_joint_state_robot1_->publish(std::move(joint_state_robot1));

            auto joint_state_robot2 = std::make_unique<sensor_msgs::msg::JointState>();
            joint_state_robot2->header = msg->header;
            joint_state_robot2->name = joint_names_robot2_;
            joint_state_robot2->velocity.resize(robot2_joints_number_);
            for (int i = robot1_joints_number_; i < robot1_joints_number_ + robot2_joints_number_; i++)
            {
                joint_state_robot2->velocity[i-robot1_joints_number_] = q_dot[i];
            }

            pub_joint_state_robot2_->publish(std::move(joint_state_robot2));
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Jacobian not received yet");
        }
    }

    void abort_inverse_kinematics(const std::string &err_msg = "ABORTED")
    {
        if (pub_joint_state_robot1_ && pub_joint_state_robot2_)
        {
            // publish zero vel
            for (int i = 0; i < 5; i++)
            {
                std::cout << "Aborting inverse kinematics" << std::endl;
                auto joint_state = std::make_unique<sensor_msgs::msg::JointState>();
                joint_state->header.stamp = this->now();
                joint_state->name = joint_names_robot1_;
                joint_state->position.resize(robot1_joints_number_);
                joint_state->velocity.resize(robot1_joints_number_);
                joint_state->position = std::vector<double>(robot1_joints_number_, 0.0);
                pub_joint_state_robot1_->publish(std::move(joint_state));

                auto joint_state2 = std::make_unique<sensor_msgs::msg::JointState>();
                joint_state2->header.stamp = this->now();
                joint_state2->name = joint_names_robot2_;
                joint_state2->position.resize(robot2_joints_number_);
                joint_state2->velocity.resize(robot2_joints_number_);
                joint_state2->position = std::vector<double>(robot2_joints_number_, 0.0);
                pub_joint_state_robot2_->publish(std::move(joint_state2));
            }
        }
        RCLCPP_ERROR_STREAM(this->get_logger(), "inverse_diff_kinematics: " << err_msg);
        throw std::runtime_error("inverse_diff_kinematics: " + err_msg);
    }

    void declare_ros_parameters()
    {

        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        cb_handles_.clear();

        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.name = "realtime_priority";
            desc.description = "Realtime priority, if <0 (default) no realtime is explicitely set";
            desc.additional_constraints = "";
            desc.read_only = true;
            desc.integer_range.resize(1);
            desc.integer_range[0].from_value = INT32_MIN;
            desc.integer_range[0].to_value = 99;
            desc.integer_range[0].step = 0;
            realtime_priority_ = this->declare_parameter(desc.name, -1, desc);
        }

        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.name = "joint_vel_limits_robot1";
            desc.description = "Joint vel limits on each joint of robot1. It is a mandatory parameter";
            desc.additional_constraints = "";
            desc.read_only = false;
            this->declare_parameter(desc.name, std::vector<double>(), desc);
            cb_handles_.insert(
                {desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter &p)
                                                                      {
             joint_vel_limits_robot1_ = p.as_double_array();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p); })});
        }

        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.name = "joint_vel_limits_robot2";
            desc.description = "Joint vel limits on each joint of robot1. It is a mandatory parameter";
            desc.additional_constraints = "";
            desc.read_only = false;
            this->declare_parameter(desc.name, std::vector<double>(), desc);
            cb_handles_.insert(
                {desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter &p)
                                                                      {
             joint_vel_limits_robot2_ = p.as_double_array();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p); })});
        }

        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.name = "joint_names_robot1";
            desc.description = "Joint names used to fill the message for robot1";
            desc.additional_constraints = "";
            desc.read_only = false;
            desc.floating_point_range.resize(1);
            desc.floating_point_range[0].from_value = 0.0;
            desc.floating_point_range[0].to_value = INFINITY;
            desc.floating_point_range[0].step = 0;
            this->declare_parameter(desc.name, std::vector<std::string>(), desc);
            cb_handles_.insert(
                {desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter &p)
                                                                      {
             joint_names_robot1_ = p.as_string_array();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p); })});
        }
        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.name = "joint_names_robot2";
            desc.description = "Joint names used to fill the message for robot2";
            desc.additional_constraints = "";
            desc.read_only = false;
            desc.floating_point_range.resize(1);
            desc.floating_point_range[0].from_value = 0.0;
            desc.floating_point_range[0].to_value = INFINITY;
            desc.floating_point_range[0].step = 0;
            this->declare_parameter(desc.name, std::vector<std::string>(), desc);
            cb_handles_.insert(
                {desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter &p)
                                                                      {
             joint_names_robot2_ = p.as_string_array();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p); })});
        }
        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.name = "b1Tb2";
            desc.description = "Transformation from robot1 to robot2 base frames. It is a mandatory parameter. The pose is "
                               "given as [x, y, z, qw, qx, qy, qz] where x, y, z are the translation and qw, qx, qy, qz are the "
                               "quaternion";
            desc.additional_constraints = "";
            desc.read_only = false;
            this->declare_parameter(desc.name, std::vector<double>(), desc);
            cb_handles_.insert(
                {desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter &p)
                                                                      {
             std::vector<double> pose;
             pose.resize(6);
             pose = p.as_double_array();
             b1Tb2_ = Eigen::Matrix<double, 4, 4>::Identity();
             b1Tb2_.block<3, 1>(0, 3) = Eigen::Vector3d(pose[0], pose[1], pose[2]);
            Eigen::Quaterniond q(pose[3], pose[4], pose[5], pose[6]); // w x y z
            q.normalize();
            b1Tb2_.block<3, 3>(0, 0) = q.toRotationMatrix();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p); })});
        }
        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.name = "bTb1";
            desc.description = "Transformation from the reference base frame and the robot1 base frame. It is a mandatory parameter. The pose is "
                               "given as [x, y, z, qw, qx, qy, qz] where x, y, z are the translation and qw, qx, qy, qz are the "
                               "quaternion";
            desc.additional_constraints = "";
            desc.read_only = false;
            this->declare_parameter(desc.name, std::vector<double>(), desc);
            cb_handles_.insert(
                {desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter &p)
                                                                      {
             std::vector<double> pose;
             pose.resize(6);
             pose = p.as_double_array();
             bTb1_ = Eigen::Matrix<double, 4, 4>::Identity();
             bTb1_.block<3, 1>(0, 3) = Eigen::Vector3d(pose[0], pose[1], pose[2]);
            Eigen::Quaterniond q(pose[3], pose[4], pose[5], pose[6]); // w x y z
            q.normalize();
            bTb1_.block<3, 3>(0, 0) = q.toRotationMatrix();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p); })});
        }
        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.name = "robot1_prefix";
            desc.description = "Prefix for the robot1 joint names";
            desc.additional_constraints = "";
            desc.read_only = false;
            this->declare_parameter(desc.name, "robot1", desc);
            cb_handles_.insert(
                {desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter &p)
                                                                      {
             robot1_prefix_ = p.as_string();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p); })});
        }
        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.name = "robot2_prefix";
            desc.description = "Prefix for the robot2 joint names";
            desc.additional_constraints = "";
            desc.read_only = false;
            this->declare_parameter(desc.name, "robot2", desc);
            cb_handles_.insert(
                {desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter &p)
                                                                      {
             robot2_prefix_ = p.as_string();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p); })});
        }
    }

protected:
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::map<std::string, rclcpp::ParameterCallbackHandle::SharedPtr> cb_handles_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_absolute_twist_; // va twist
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_relative_twist_; // vr twist
    rclcpp::Subscription<uclv_robot_ros_msgs::msg::Matrix>::SharedPtr sub_jacobian_robot1_;
    rclcpp::Subscription<uclv_robot_ros_msgs::msg::Matrix>::SharedPtr sub_jacobian_robot2_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_robot1_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_robot2_;

    uclv_robot_ros_msgs::msg::Matrix::ConstSharedPtr jacobian_robot1_;
    uclv_robot_ros_msgs::msg::Matrix::ConstSharedPtr jacobian_robot2_;

    std::vector<double> joint_vel_limits_robot1_;
    std::vector<double> joint_vel_limits_robot2_;

    std::vector<std::string> joint_names_robot1_;
    std::vector<std::string> joint_names_robot2_;

    Eigen::Matrix<double, 4, 4> b1Tb2_; // transformation from robot1 to robot2 base frames
    Eigen::Matrix<double, 4, 4> bTb1_;  // transformation from the reference base frame and the robot1 base frame

    Eigen::Vector<double, 6> absolute_twist_;
    Eigen::Vector<double, 6> relative_twist_;

    std::string robot1_prefix_;
    std::string robot2_prefix_;

    int robot1_joints_number_;
    int robot2_joints_number_;
    int realtime_priority_ = -1;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<CooperativeRobotsServer>(options));
    rclcpp::shutdown();
    return 0;
}

// #include <rclcpp_components/register_node_macro.hpp>
// RCLCPP_COMPONENTS_REGISTER_NODE(uclv_robot_ros::CooperativeRobotsServer)
