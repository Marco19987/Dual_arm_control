#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <uclv_robot_ros_msgs/msg/matrix.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <realtime_tools/thread_priority.hpp>
#include "geometry_helper.hpp"

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
  CooperativeRobotsServer(const rclcpp::NodeOptions& options) : Node("cooperative_robots_server", options)
  {
    // Declare parameters
    declare_ros_parameters();

    initRealTime();

    int index = 0;
    std::string robot1_prefix = this->get_parameter("robot1_prefix").as_string();
    sub_jacobian_robot1_ = this->create_subscription<uclv_robot_ros_msgs::msg::Matrix>(
        robot1_prefix + "/jacobian", rclcpp::SensorDataQoS(),
        [this, index](const uclv_robot_ros_msgs::msg::Matrix::SharedPtr msg) { this->jacobianCallback(msg, index); });

    index = 1;
    std::string robot2_prefix = this->get_parameter("robot2_prefix").as_string();
    sub_jacobian_robot2_ = this->create_subscription<uclv_robot_ros_msgs::msg::Matrix>(
        robot2_prefix + "/jacobian", rclcpp::SensorDataQoS(),
        [this, index](const uclv_robot_ros_msgs::msg::Matrix::SharedPtr msg) { this->jacobianCallback(msg, index); });

    pub_joint_state_robot1_ = this->create_publisher<sensor_msgs::msg::JointState>(
        robot1_prefix + "/command/joint_vel_states", rclcpp::SensorDataQoS());

    pub_joint_state_robot2_ = this->create_publisher<sensor_msgs::msg::JointState>(
        robot2_prefix + "/command/joint_vel_states", rclcpp::SensorDataQoS());

    index = 0;
    sub_absolute_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "absolute_twist", rclcpp::SensorDataQoS(),
        [this, index](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { this->twistCallback(msg, index); });

    index = 1;
    sub_relative_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "relative_twist", rclcpp::SensorDataQoS(),
        [this, index](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { this->twistCallback(msg, index); });

    index = 0;
    sub_fkine_robot1_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        robot1_prefix + "/fkine", rclcpp::SensorDataQoS(),
        [this, index](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->fkineCallback(msg, index); });

    index = 1;
    sub_fkine_robot2_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        robot2_prefix + "/fkine", rclcpp::SensorDataQoS(),
        [this, index](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->fkineCallback(msg, index); });

    pub_absolute_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("absolute_pose", 1);
    pub_fkine_robot1_base_frame_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(robot1_prefix + "/fkine_base_frame", 1);
    pub_fkine_robot2_base_frame_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(robot2_prefix + "/fkine_base_frame", 1);

    // Initialize variables
    absolute_twist_.setZero();
    relative_twist_.setZero();
    fkine_robot1_.setZero();
    fkine_robot2_.setZero();
    previous_absolute_quaterion_.setIdentity();
    fk1_T_fk2_.setIdentity();
    bT_absolute_.setIdentity();
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
  void fkineCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg, int index)
  {
    if (index == 0)
    {
      fkine_robot1_[0] = msg->pose.position.x;
      fkine_robot1_[1] = msg->pose.position.y;
      fkine_robot1_[2] = msg->pose.position.z;
      fkine_robot1_[3] = msg->pose.orientation.w;
      fkine_robot1_[4] = msg->pose.orientation.x;
      fkine_robot1_[5] = msg->pose.orientation.y;
      fkine_robot1_[6] = msg->pose.orientation.z;
      fkine_robot1_read = true;
    }
    else
    {
      fkine_robot2_[0] = msg->pose.position.x;
      fkine_robot2_[1] = msg->pose.position.y;
      fkine_robot2_[2] = msg->pose.position.z;
      fkine_robot2_[3] = msg->pose.orientation.w;
      fkine_robot2_[4] = msg->pose.orientation.x;
      fkine_robot2_[5] = msg->pose.orientation.y;
      fkine_robot2_[6] = msg->pose.orientation.z;
      fkine_robot2_read = true;
    }

    if (fkine_robot1_read && fkine_robot2_read)
    {
      compute_cooperative_space_coordinates();
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

  void eigen_to_pose(const Eigen::Matrix<double, 4, 4>& T, geometry_msgs::msg::PoseStamped& pose)
  {
    pose.pose.position.x = T(0, 3);
    pose.pose.position.y = T(1, 3);
    pose.pose.position.z = T(2, 3);
    Eigen::Quaterniond q(T.block<3, 3>(0, 0));
    q.normalize();
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
  }

  void compute_cooperative_space_coordinates()
  {
    // transform fkine robot1 to the base frame
    Eigen::Quaterniond b1Qfk1(fkine_robot1_[3], fkine_robot1_[4], fkine_robot1_[5],
                              fkine_robot1_[6]);  // quaternion fkine 2 wrt base robot 2
    b1Qfk1.normalize();
    Eigen::Matrix<double, 4, 4> b1Tfk1;
    uclv::geometry_helper::pose_to_matrix(fkine_robot1_, b1Tfk1);
    Eigen::Matrix<double, 4, 4> bTfk1 = bTb1_ * b1Tfk1;
    Eigen::Quaterniond bQb1(bTb1_.block<3, 3>(0, 0));
    bQb1.normalize();
    Eigen::Quaterniond bQfk1 = bQb1 * b1Qfk1;

    // publish fkine robot1 in the base frame
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = base_frame_name_;
    eigen_to_pose(bTfk1, pose_msg);
    pub_fkine_robot1_base_frame_->publish(pose_msg);

    // transform fkine robot2 to the base frame
    Eigen::Quaterniond b2Qfk2(fkine_robot2_[3], fkine_robot2_[4], fkine_robot2_[5],
                              fkine_robot2_[6]);  // quaternion fkine 2 wrt base robot 2
    b2Qfk2.normalize();
    Eigen::Matrix<double, 4, 4> b2Tfk2;
    uclv::geometry_helper::pose_to_matrix(fkine_robot2_, b2Tfk2);
    Eigen::Matrix<double, 4, 4> bTfk2 = bTb1_ * b1Tb2_ * b2Tfk2;
    Eigen::Quaterniond bQb2(bTb1_.block<3, 3>(0, 0) * b1Tb2_.block<3, 3>(0, 0));
    bQb2.normalize();
    Eigen::Quaterniond bQfk2 = bQb2 * b2Qfk2;

    // publish fkine robot2 in the base frame
    eigen_to_pose(bTfk2, pose_msg);
    pub_fkine_robot2_base_frame_->publish(pose_msg);

    // compute the mean between the two poses
    Eigen::Vector<double, 3> b_p_absolute = (bTfk1.block<3, 1>(0, 3) + bTfk2.block<3, 1>(0, 3)) / 2;

    // Eigen::Matrix3d bRfk1 = bTfk1.block<3, 3>(0, 0);
    // Eigen::Matrix3d bRfk2 = bTfk2.block<3, 3>(0, 0);
    // Eigen::Matrix3d fk1Rfk2 = bRfk1.transpose() * bRfk2;

    Eigen::AngleAxisd angleAxis(bQfk1.inverse() * bQfk2);

    // Eigen::AngleAxisd angleAxis(fk1Rfk2);
    double fk1_theta_fk2 = angleAxis.angle();
    Eigen::Vector3d fk1_r_fk2 = angleAxis.axis();

    // std::cout << "ANGLE AXIS Variables" << std::endl;
    // std::cout << "fk1_theta_fk2: " << fk1_theta_fk2 << std::endl;
    // std::cout << "fk1_r_fk2: " << fk1_r_fk2.transpose() << std::endl;

    // Eigen::Matrix3d fk1_R_fk2_half = Eigen::AngleAxisd(fk1_theta_fk2 / 2.0, fk1_r_fk2).toRotationMatrix();
    // Eigen::Matrix3d bR_absolute = bRfk1 * fk1_R_fk2_half;

    Eigen::Quaterniond bQ_absolute = bQfk1 * Eigen::Quaterniond(Eigen::AngleAxisd(fk1_theta_fk2 / 2.0, fk1_r_fk2));

    // publish absolute frame
    // pose_msg.header.frame_id = base_frame_name_;

    // pose_msg.pose.position.x = b_p_absolute(0);
    // pose_msg.pose.position.y = b_p_absolute(1);
    // pose_msg.pose.position.z = b_p_absolute(2);

    // Eigen::Quaterniond mean_orientation(bR_absolute);
    // Eigen::Quaterniond mean_orientation(bQ_absolute);
    // uclv::geometry_helper::quaternion_continuity(mean_orientation, previous_absolute_quaterion_, mean_orientation);
    // previous_absolute_quaterion_ = mean_orientation;
    // pose_msg.pose.orientation.w = mean_orientation.w();
    // pose_msg.pose.orientation.x = mean_orientation.x();
    // pose_msg.pose.orientation.y = mean_orientation.y();
    // pose_msg.pose.orientation.z = mean_orientation.z();

    // publish the new pose
    // pub_absolute_pose_->publish(pose_msg);

    // save bT_absolute_
    bT_absolute_.block<3, 3>(0, 0) = bQ_absolute.toRotationMatrix();
    bT_absolute_.block<3, 1>(0, 3) = b_p_absolute;
    eigen_to_pose(bT_absolute_, pose_msg);
    pub_absolute_pose_->publish(pose_msg);

    // compute fk1_T_fk2_
    Eigen::Quaterniond fk1_Q_fk2(bQfk1.inverse() * bQfk2);
    fk1_Q_fk2.normalize();
    fk1_T_fk2_.block<3, 3>(0, 0) = fk1_Q_fk2.toRotationMatrix();
    fk1_T_fk2_.block<3, 1>(0, 3) = bTfk2.block<3, 1>(0, 3) - bTfk1.block<3, 1>(0, 3);
  }

  void rotate_jacobian(const uclv_robot_ros_msgs::msg::Matrix::ConstSharedPtr jacobian,
                       const Eigen::Matrix<double, 4, 4>& T, Eigen::Matrix<double, 6, Eigen::Dynamic>& J_rotated)
  {
    Eigen::Matrix<double, 6, 6> Rext;
    Rext.setZero();
    Rext.block<3, 3>(0, 0) = T.block<3, 3>(0, 0);
    Rext.block<3, 3>(3, 3) = T.block<3, 3>(0, 0);

    if (jacobian->row_major)
    {
      Eigen::Map<const Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::RowMajor>> J(jacobian->data.data(),
                                                                                    jacobian->dim[0], jacobian->dim[1]);
      J_rotated.resize(6, J.cols());
      J_rotated.setZero();
      J_rotated = Rext * J;
    }
    else
    {
      Eigen::Map<const Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::ColMajor>> J(jacobian->data.data(),
                                                                                    jacobian->dim[0], jacobian->dim[1]);
      J_rotated.resize(6, J.cols());
      J_rotated.setZero();
      J_rotated = Rext * J;
    }
  }

  void compute_qdot(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    // check if jacobians are received
    if (jacobian_robot1_ && jacobian_robot2_)
    {
      // rotate jacobian 1 to the base frame
      Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_robot1_base;  // Jacobian robot 1 base frame rotated in the
                                                                      // common base frame
      rotate_jacobian(jacobian_robot1_, bTb1_, jacobian_robot1_base);

      // rotate jacobian 2 to the base frame
      Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_robot2_base;  // Jacobian robot 2 base frame rotated in the
                                                                      // common base frame
      Eigen::Matrix<double, 4, 4> bTb2;
      bTb2 = Eigen::Matrix<double, 4, 4>::Identity();
      bTb2 << bTb1_ * b1Tb2_;
      rotate_jacobian(jacobian_robot2_, bTb2, jacobian_robot2_base);

      // define absolute jacobian
      Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_absolute;  // Ja = 0.5 * [J1, J2];
      jacobian_absolute.resize(6, jacobian_robot1_base.cols() + jacobian_robot2_base.cols());
      jacobian_absolute << 0.5 * jacobian_robot1_base, 0.5 * jacobian_robot2_base;

      // define relative jacobian
      Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_relative;  // Jr = [-J1 J2];
      jacobian_relative.resize(6, jacobian_robot1_base.cols() + jacobian_robot2_base.cols());
      jacobian_relative << -jacobian_robot1_base, jacobian_robot2_base;

      // define complete jacobian
      Eigen::Matrix<double, 12, Eigen::Dynamic> jacobian_complete;  // J = [Ja;Jr];
      jacobian_complete.resize(12, jacobian_absolute.cols());
      jacobian_complete << jacobian_absolute, jacobian_relative;

      // define complete twist
      Eigen::Matrix<double, 12, 1> twist_complete;  // twist = [v1;w1;v2;w2];
      twist_complete << absolute_twist_, relative_twist_;

      // add foward action to the relative twist
      if (hold_robots_relative_pose_)
      {
        // add to the relative linear twist skew(absolute_angular_twist)*fk1_p_fk2
        Eigen::Matrix<double, 3, 3> skew_angular_twist;
        uclv::geometry_helper::skew(absolute_twist_.block<3, 1>(3, 0), skew_angular_twist);
        Eigen::Vector<double, 3> fk1_p_fk2 = fk1_T_fk2_.block<3, 1>(0, 3);
        twist_complete.block<3, 1>(6, 0) += skew_angular_twist * fk1_p_fk2;
      }

      // solve inverse kinematics
      Eigen::Matrix<double, Eigen::Dynamic, 1> q_dot;
      q_dot.resize(jacobian_complete.cols());
      q_dot.setZero();
      q_dot = jacobian_complete.completeOrthogonalDecomposition().solve(twist_complete);

      // exploit redundancy to minimize the the function W(q) = (q-qdes)^T * (q-qdes)
      // check if the size of q1_desired_ and q2_desired_ are the same as the size of robot1_joints_number_ and
      // robot2_joints_number_
      if ((Eigen::Index)q1_desired_.size() != robot1_joints_number_ ||
          (Eigen::Index)q2_desired_.size() != robot2_joints_number_)
      {
        abort_inverse_kinematics("DESIRED JOINTS SIZE MISMATCH q_size: " + std::to_string(q_dot.size()) +
                                 " -- q1_desired_size: " + std::to_string(q1_desired_.size()) +
                                 "-- q2_desired_size: " + std::to_string(q2_desired_.size()));
      }
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> null_space_projector;
      null_space_projector.resize(jacobian_complete.cols(), jacobian_complete.cols());
      null_space_projector.setIdentity();
      null_space_projector -= jacobian_complete.completeOrthogonalDecomposition().pseudoInverse() * jacobian_complete;
      Eigen::Matrix<double, Eigen::Dynamic, 1> q_desired;
      q_desired.resize(jacobian_complete.cols());
      q_desired.setZero();
      q_desired << q1_desired_, q2_desired_;

      q_dot += -null_space_projector * abs(secondary_task_weight_) * (q_dot - q_desired);

      // check velocity limits
      if ((Eigen::Index)joint_vel_limits_robot1_.size() != robot1_joints_number_ ||
          (Eigen::Index)joint_vel_limits_robot2_.size() != robot2_joints_number_)
      {
        abort_inverse_kinematics("VEL LIMITS SIZE MISMATCH q_size: " + std::to_string(q_dot.size()) +
                                 " -- joint_vel_limits_size: " + std::to_string(joint_vel_limits_robot1_.size()) +
                                 "-- joint_vel_limits_size: " + std::to_string(joint_vel_limits_robot2_.size()));
      }

      for (int i = 0; i < q_dot.size(); i++)
      {
        if (i < robot1_joints_number_)
        {
          if (fabs(q_dot(i)) > joint_vel_limits_robot1_[i])
          {
            std::cout << "q_dot: " << q_dot(i) << " joint_vel_limits_robot1_[i]: " << joint_vel_limits_robot1_[i]
                      << std::endl;
            abort_inverse_kinematics("VEL LIMITS VIOLATED ROBOT 1");
          }
        }
        else
        {
          if (fabs(q_dot(i)) > joint_vel_limits_robot2_[i - robot1_joints_number_])
          {
            std::cout << "q_dot: " << q_dot(i) << " joint_vel_limits_robot2_[i - robot1_joints_number_]: "
                      << joint_vel_limits_robot2_[i - robot1_joints_number_] << std::endl;
            abort_inverse_kinematics("VEL LIMITS VIOLATED ROBOT 2");
          }
        }
      }

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
      for (int i = 0; i < robot2_joints_number_; i++)
      {
        joint_state_robot2->velocity[i] = q_dot[i + robot2_joints_number_];
      }

      pub_joint_state_robot2_->publish(std::move(joint_state_robot2));
    }
    else
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Jacobian not received yet");
    }
  }

  void abort_inverse_kinematics(const std::string& err_msg = "ABORTED")
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
          { desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter& p) {
             joint_vel_limits_robot1_ = p.as_double_array();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p);
           }) });
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "joint_vel_limits_robot2";
      desc.description = "Joint vel limits on each joint of robot1. It is a mandatory parameter";
      desc.additional_constraints = "";
      desc.read_only = false;
      this->declare_parameter(desc.name, std::vector<double>(), desc);
      cb_handles_.insert(
          { desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter& p) {
             joint_vel_limits_robot2_ = p.as_double_array();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p);
           }) });
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
          { desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter& p) {
             joint_names_robot1_ = p.as_string_array();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p);
           }) });
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
          { desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter& p) {
             joint_names_robot2_ = p.as_string_array();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p);
           }) });
    }
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "b1Tb2";
      desc.description =
          "Transformation from robot1 to robot2 base frames. It is a mandatory parameter. The pose is "
          "given as [x, y, z, qw, qx, qy, qz] where x, y, z are the translation and qw, qx, qy, qz are the "
          "quaternion";
      desc.additional_constraints = "";
      desc.read_only = false;
      this->declare_parameter(desc.name, std::vector<double>(), desc);
      cb_handles_.insert(
          { desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter& p) {
             std::vector<double> pose;
             pose.resize(6);
             pose = p.as_double_array();
             b1Tb2_ = Eigen::Matrix<double, 4, 4>::Identity();
             b1Tb2_.block<3, 1>(0, 3) = Eigen::Vector3d(pose[0], pose[1], pose[2]);
             Eigen::Quaterniond q(pose[3], pose[4], pose[5], pose[6]);  // w x y z
             q.normalize();
             b1Tb2_.block<3, 3>(0, 0) = q.toRotationMatrix();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p);
           }) });
    }
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "bTb1";
      desc.description =
          "Transformation from the reference base frame and the robot1 base frame. It is a mandatory parameter. The "
          "pose is "
          "given as [x, y, z, qw, qx, qy, qz] where x, y, z are the translation and qw, qx, qy, qz are the "
          "quaternion";
      desc.additional_constraints = "";
      desc.read_only = false;
      this->declare_parameter(desc.name, std::vector<double>(), desc);
      cb_handles_.insert(
          { desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter& p) {
             std::vector<double> pose;
             pose.resize(6);
             pose = p.as_double_array();
             bTb1_ = Eigen::Matrix<double, 4, 4>::Identity();
             bTb1_.block<3, 1>(0, 3) = Eigen::Vector3d(pose[0], pose[1], pose[2]);
             Eigen::Quaterniond q(pose[3], pose[4], pose[5], pose[6]);  // w x y z
             q.normalize();
             bTb1_.block<3, 3>(0, 0) = q.toRotationMatrix();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p);
           }) });
    }
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "robot1_prefix";
      desc.description = "Prefix for the robot1 joint names";
      desc.additional_constraints = "";
      desc.read_only = false;
      this->declare_parameter(desc.name, "robot1", desc);
      cb_handles_.insert(
          { desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter& p) {
             robot1_prefix_ = p.as_string();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p);
           }) });
    }
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "robot2_prefix";
      desc.description = "Prefix for the robot2 joint names";
      desc.additional_constraints = "";
      desc.read_only = false;
      this->declare_parameter(desc.name, "robot2", desc);
      cb_handles_.insert(
          { desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter& p) {
             robot2_prefix_ = p.as_string();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p);
           }) });
    }
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "hold_robots_relative_pose";
      desc.description = "True if you want to hold the relative pose of the robots";
      desc.additional_constraints = "";
      desc.read_only = false;
      this->declare_parameter(desc.name, true, desc);
      cb_handles_.insert(
          { desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter& p) {
             hold_robots_relative_pose_ = p.as_bool();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p);
           }) });
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "base_frame_name";
      desc.description = "Base frame name";
      desc.additional_constraints = "";
      desc.read_only = false;
      this->declare_parameter(desc.name, "base_frame", desc);
      cb_handles_.insert(
          { desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter& p) {
             base_frame_name_ = p.as_string();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p);
           }) });
    }
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "q1_desired";
      desc.description = "desired joint configuration secondary objective robot 1";
      desc.additional_constraints = "";
      desc.read_only = false;
      this->declare_parameter(desc.name, std::vector<double>(), desc);
      cb_handles_.insert(
          { desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter& p) {
             std::vector<double> q_desired = p.as_double_array();
             // resize q1_desired_ to length of q_desired
             q1_desired_.resize(q_desired.size());
             for (u_int16_t i = 0; i < q_desired.size(); i++)
             {
               q1_desired_[i] = q_desired[i];
             }
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p);
           }) });
    }
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "q2_desired";
      desc.description = "desired joint configuration secondary objective robot 2";
      desc.additional_constraints = "";
      desc.read_only = false;
      this->declare_parameter(desc.name, std::vector<double>(), desc);
      cb_handles_.insert(
          { desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter& p) {
             std::vector<double> q_desired = p.as_double_array();
             // resize q2_desired_ to length of q_desired
             q2_desired_.resize(q_desired.size());
             for (u_int16_t i = 0; i < q_desired.size(); i++)
             {
               q2_desired_[i] = q_desired[i];
             }
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p);
           }) });
    }
    // secondary_task_weight_
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "secondary_task_weight";
      desc.description = "secondary task weight";
      desc.additional_constraints = "";
      desc.read_only = false;
      this->declare_parameter(desc.name, 0.01, desc);
      cb_handles_.insert(
          { desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter& p) {
             secondary_task_weight_ = p.as_double();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p);
           }) });
    }
  }

protected:
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::map<std::string, rclcpp::ParameterCallbackHandle::SharedPtr> cb_handles_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_absolute_twist_;  // va twist
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_relative_twist_;  // vr twist
  rclcpp::Subscription<uclv_robot_ros_msgs::msg::Matrix>::SharedPtr sub_jacobian_robot1_;
  rclcpp::Subscription<uclv_robot_ros_msgs::msg::Matrix>::SharedPtr sub_jacobian_robot2_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_fkine_robot1_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_fkine_robot2_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_robot1_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_robot2_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_fkine_robot1_base_frame_;  // publish fkine
                                                                                               // transfromed in the
                                                                                               // common base frame
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_fkine_robot2_base_frame_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_absolute_pose_;

  uclv_robot_ros_msgs::msg::Matrix::ConstSharedPtr jacobian_robot1_;
  uclv_robot_ros_msgs::msg::Matrix::ConstSharedPtr jacobian_robot2_;

  std::vector<double> joint_vel_limits_robot1_;
  std::vector<double> joint_vel_limits_robot2_;

  std::vector<std::string> joint_names_robot1_;
  std::vector<std::string> joint_names_robot2_;

  Eigen::Matrix<double, 4, 4> b1Tb2_;  // transformation from robot1 to robot2 base frames
  Eigen::Matrix<double, 4, 4> bTb1_;   // transformation from the reference base frame and the robot1 base frame

  Eigen::Vector<double, 6> absolute_twist_;
  Eigen::Vector<double, 6> relative_twist_;

  Eigen::Vector<double, 7> fkine_robot1_;
  Eigen::Vector<double, 7> fkine_robot2_;
  Eigen::Matrix<double, 4, 4> bT_absolute_;  // transformation from the reference base frame to the absolute frame
  Eigen::Matrix<double, 4, 4> fk1_T_fk2_;    // relative pose between robots expressend in the robot 1 fkine frame

  std::string robot1_prefix_;
  std::string robot2_prefix_;

  int robot1_joints_number_;
  int robot2_joints_number_;
  int realtime_priority_ = -1;

  bool hold_robots_relative_pose_;  // if true, the robots will hold the relative pose adding a forward term to the
                                    // relative twist
  bool fkine_robot1_read = false;
  bool fkine_robot2_read = false;

  std::string base_frame_name_;

  Eigen::Quaterniond previous_absolute_quaterion_;

  double secondary_task_weight_ = 0;

  Eigen::Vector<double, Eigen::Dynamic> q1_desired_;  // desired joint configuration secondary objective robot 1
  Eigen::Vector<double, Eigen::Dynamic> q2_desired_;  // desired joint configuration secondary objective robot 2;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<CooperativeRobotsServer>(options));
  rclcpp::shutdown();
  return 0;
}

// #include <rclcpp_components/register_node_macro.hpp>
// RCLCPP_COMPONENTS_REGISTER_NODE(uclv_robot_ros::CooperativeRobotsServer)
