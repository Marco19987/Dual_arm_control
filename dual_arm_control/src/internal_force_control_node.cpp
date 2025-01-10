#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <eigen3/Eigen/Geometry>
#include "geometry_helper.hpp"

// this node reads the wrench measures from two robots, computes the internal wrench applied to the object and computes
// the relative twist to regulate it to the desired one
// the grasp matrix is computed online by reading the robot poses and the object pose
// [IMPORTANT]: it is assumed that the read wrenhes are expressed in the robot fkine frames

// % grasp matrix
// Wg1 = [eye(3), zeros(3); -skew(opg1)', eye(3)];
// Wg2 = [eye(3), zeros(3); -skew(opg2)', eye(3)];
// W = [Wg1,Wg2];

// % Rbar matrix
// Rbar = blkdiag(oRg1,oRg1,oRg2,oRg2);
// oh_wrenches(:,i) = Rbar*h_wrenches(:,i);
// oh_int(:,i) = (eye(12) - pinv(W)*W)*oh_wrenches(:,i);
// bRo_i = quat2rotm(x_obj(4:7,i)');
// bRobar = blkdiag(bRo_i,bRo_i);
// vrd_int_wrench(:,i) = bRobar*10*Kc*(-(oh_int(7:end,i)-oh_int(1:6,i)));

class InternalForceControlNode : public rclcpp::Node
{
public:
  InternalForceControlNode() : Node("internal_force_control_node"), control_active_(false)
  {
    // declare parameters
    this->declare_parameter("sample_time", 0.02);
    this->get_parameter("sample_time", sample_time_);

    this->declare_parameter("force_control_gain_diag_vector",
                            std::vector<double>({ 1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1 }));
    std::vector<double> force_control_gain_diag_vector;
    this->get_parameter("force_control_gain_diag_vector", force_control_gain_diag_vector);
    force_control_gain_matrix_.setZero();
    force_control_gain_matrix_.diagonal() << force_control_gain_diag_vector[0], force_control_gain_diag_vector[1],
        force_control_gain_diag_vector[2], force_control_gain_diag_vector[3], force_control_gain_diag_vector[4],
        force_control_gain_diag_vector[5];

    this->declare_parameter("robot1_prefix", "robot1");
    robot1_prefix = this->get_parameter("robot1_prefix").as_string();

    this->declare_parameter("robot2_prefix", "robot2");
    robot2_prefix = this->get_parameter("robot2_prefix").as_string();

    // declare subscribers and publishers
    sub_object_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object_pose", rclcpp::SensorDataQoS(),
        std::bind(&InternalForceControlNode::objectPoseCallback, this, std::placeholders::_1));

    int index = 0;
    sub_fkine_robot1_base_frame_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        robot1_prefix + "/fkine_base_frame", rclcpp::SensorDataQoS(),
        [this, index](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->fkineCallback(msg, index); });

    index = 1;
    sub_fkine_robot2_base_frame_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        robot2_prefix + "/fkine_base_frame", rclcpp::SensorDataQoS(),
        [this, index](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->fkineCallback(msg, index); });

    index = 0;
    sub_wrench_robot1_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        robot1_prefix + "/wrench", rclcpp::SensorDataQoS(),
        [this, index](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) { this->wrenchCallback(msg, index); });

    index = 1;
    sub_wrench_robot2_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        robot2_prefix + "/wrench", rclcpp::SensorDataQoS(),
        [this, index](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) { this->wrenchCallback(msg, index); });

    sub_desired_internal_wrench_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/desired_internal_wrench", rclcpp::SensorDataQoS(),
        std::bind(&InternalForceControlNode::desiredInternalWrenchPoseCallback, this, std::placeholders::_1));

    pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("relative_twist", 1);

    pub_internal_wrench_1_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("internal_wrench_1", 1);
    pub_internal_wrench_2_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("internal_wrench_2", 1);

    pub_object_wrench_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("object_wrench", 1);

    // create service
    service_ = this->create_service<std_srvs::srv::SetBool>(
        "activate_force_control", std::bind(&InternalForceControlNode::activateControlCallback, this,
                                            std::placeholders::_1, std::placeholders::_2));

    // initialize variables
    bTo_.setIdentity();
    bTfkine1_.setIdentity();
    bTfkine2_.setIdentity();
    internal_wrench_.setZero();
    desired_internal_wrench_.setZero();
    fkine1_h_fkine1_.setZero();
    fkine2_h_fkine2_.setZero();
    relative_twist_.setZero();
  }

private:
  void timerCallback()
  {
    if (object_pose_read_ && fkine_robot1_read && fkine_robot2_read && robot1_wrench_read && robot2_wrench_read)
    {
      // compute oTfkine1 and oTfkine2
      Eigen::Matrix<double, 4, 4> oTfkine1 = bTo_.inverse() * bTfkine1_;
      Eigen::Matrix<double, 4, 4> oTfkine2 = bTo_.inverse() * bTfkine2_;

      Eigen::Matrix3d skew_o_p_fkine1;
      uclv::geometry_helper::skew(oTfkine1.block<3, 1>(0, 3), skew_o_p_fkine1);

      Eigen::Matrix3d skew_o_p_fkine2;
      uclv::geometry_helper::skew(oTfkine2.block<3, 1>(0, 3), skew_o_p_fkine2);

      Eigen::Matrix<double, 12, 12> Rbar;
      Rbar.setZero();
      Rbar.block<3, 3>(0, 0) = oTfkine1.block<3, 3>(0, 0);
      Rbar.block<3, 3>(3, 3) = oTfkine1.block<3, 3>(0, 0);
      Rbar.block<3, 3>(6, 6) = oTfkine2.block<3, 3>(0, 0);
      Rbar.block<3, 3>(9, 9) = oTfkine2.block<3, 3>(0, 0);

      // compute the grasp matrix
      Eigen::Matrix<double, 6, 6> Wg1;
      Wg1.setZero();
      Wg1.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
      Wg1.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
      Wg1.block<3, 3>(3, 0) = -skew_o_p_fkine1.transpose();
      Eigen::Matrix<double, 6, 6> Wg2;
      Wg2.setZero();
      Wg2.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
      Wg2.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
      Wg2.block<3, 3>(3, 0) = -skew_o_p_fkine2.transpose();

      Eigen::Matrix<double, 6, 12> W;
      W.setZero();
      W.block<6, 6>(0, 0) = Wg1;
      W.block<6, 6>(0, 6) = Wg2;

      // compute the internal wrench
      Eigen::Matrix<double, 12, 1> o_h_internal_wrench;
      Eigen::Matrix<double, 12, 6> W_pinv = W.completeOrthogonalDecomposition().pseudoInverse();
      Eigen::Matrix<double, 12, 1> measured_wrench;
      measured_wrench << fkine1_h_fkine1_, fkine2_h_fkine2_;
      o_h_internal_wrench = (Eigen::Matrix<double, 12, 12>::Identity() - W_pinv * W) * Rbar * measured_wrench;

      // compute the relative twist
      Eigen::Matrix<double, 6, 6> bRo_bar;  // bRo_bar = [bRo, -Skew(bpo)*bRo; 0, bRo]
      Eigen::Matrix<double, 3, 3> skew_b_p_o; // skew(b_p_o)
      uclv::geometry_helper::skew(bTo_.block<3, 1>(0, 3), skew_b_p_o);
      bRo_bar.setZero();
      bRo_bar.block<3, 3>(0, 0) = bTo_.block<3, 3>(0, 0);
      bRo_bar.block<3, 3>(3, 3) = bTo_.block<3, 3>(0, 0);
      // bRo_bar.block<3, 3>(0, 3) = -skew_b_p_o*bTo_.block<3, 3>(0, 0);

      relative_twist_ = bRo_bar * force_control_gain_matrix_ *
                        (-o_h_internal_wrench.block<6, 1>(6, 0) + o_h_internal_wrench.block<6, 1>(0, 0));

      // publish messages
      geometry_msgs::msg::TwistStamped twist_msg;
      twist_msg.header.stamp = this->now();
      twist_msg.twist.linear.x = relative_twist_(0);
      twist_msg.twist.linear.y = relative_twist_(1);
      twist_msg.twist.linear.z = relative_twist_(2);
      twist_msg.twist.angular.x = relative_twist_(3);
      twist_msg.twist.angular.y = relative_twist_(4);
      twist_msg.twist.angular.z = relative_twist_(5);
      pub_twist_->publish(twist_msg);

      geometry_msgs::msg::WrenchStamped wrench_msg;
      wrench_msg.header.stamp = this->now();
      wrench_msg.wrench.force.x = o_h_internal_wrench(0);
      wrench_msg.wrench.force.y = o_h_internal_wrench(1);
      wrench_msg.wrench.force.z = o_h_internal_wrench(2);
      wrench_msg.wrench.torque.x = o_h_internal_wrench(3);
      wrench_msg.wrench.torque.y = o_h_internal_wrench(4);
      wrench_msg.wrench.torque.z = o_h_internal_wrench(5);
      pub_internal_wrench_1_->publish(wrench_msg);

      wrench_msg.wrench.force.x = o_h_internal_wrench(6);
      wrench_msg.wrench.force.y = o_h_internal_wrench(7);
      wrench_msg.wrench.force.z = o_h_internal_wrench(8);
      wrench_msg.wrench.torque.x = o_h_internal_wrench(9);
      wrench_msg.wrench.torque.y = o_h_internal_wrench(10);
      wrench_msg.wrench.torque.z = o_h_internal_wrench(11);
      pub_internal_wrench_2_->publish(wrench_msg);

      Eigen::Matrix<double, 6, 1> object_wrench;
      object_wrench = W * Rbar * measured_wrench;

      wrench_msg.wrench.force.x = object_wrench(0);
      wrench_msg.wrench.force.y = object_wrench(1);
      wrench_msg.wrench.force.z = object_wrench(2);
      wrench_msg.wrench.torque.x = object_wrench(3);
      wrench_msg.wrench.torque.y = object_wrench(4);
      wrench_msg.wrench.torque.z = object_wrench(5);
      pub_object_wrench_->publish(wrench_msg);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for pose messages or control activation...");
    }
  }

  void desiredInternalWrenchPoseCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    desired_internal_wrench_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x,
        msg->wrench.torque.y, msg->wrench.torque.z;
  }

  void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg, int index)
  {
    if (index == 0)
    {
      fkine1_h_fkine1_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x,
          msg->wrench.torque.y, msg->wrench.torque.z;
      robot1_wrench_read = true;
    }
    else
    {
      fkine2_h_fkine2_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x,
          msg->wrench.torque.y, msg->wrench.torque.z;
      robot2_wrench_read = true;
    }
  }

  void fkineCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int index)
  {
    if (index == 0)
    {
      convertPoseToMatrix(msg, bTfkine1_);
      fkine_robot1_read = true;
    }
    else
    {
      convertPoseToMatrix(msg, bTfkine2_);
      fkine_robot2_read = true;
    }
  }

  void objectPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    convertPoseToMatrix(msg, bTo_);
    object_pose_read_ = true;
  }

  void convertPoseToMatrix(const geometry_msgs::msg::PoseStamped::SharedPtr msg, Eigen::Matrix<double, 4, 4>& matrix)
  {
    Eigen::Vector<double, 7> pose;
    pose << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->pose.orientation.w,
        msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
    uclv::geometry_helper::pose_to_matrix(pose, matrix);
  }

  void activateControlCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                               std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    control_active_ = request->data;
    if (!control_active_)
    {
      reset_server();
    }
    else
    {
      timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(sample_time_ * 1000)),
                                       std::bind(&InternalForceControlNode::timerCallback, this));
    }
    response->success = true;
    response->message = control_active_ ? "Control activated" : "Control deactivated";
  }

  void reset_server()
  {
    relative_twist_.setZero();
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = this->now();
    twist_msg.twist.linear.x = relative_twist_(0);
    twist_msg.twist.linear.y = relative_twist_(1);
    twist_msg.twist.linear.z = relative_twist_(2);
    twist_msg.twist.angular.x = relative_twist_(3);
    twist_msg.twist.angular.y = relative_twist_(4);
    twist_msg.twist.angular.z = relative_twist_(5);
    for (int i = 0; i < 10; i++)
      pub_twist_->publish(twist_msg);
    timer_->cancel();
    object_pose_read_ = false;
    fkine_robot1_read = false;
    fkine_robot2_read = false;
    control_active_ = false;
    robot1_wrench_read = false;
    robot2_wrench_read = false;
    bTo_.setIdentity();
    bTfkine1_.setIdentity();
    bTfkine2_.setIdentity();
    relative_twist_.setZero();
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_object_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_fkine_robot1_base_frame_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_fkine_robot2_base_frame_;

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_desired_internal_wrench_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench_robot1_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench_robot2_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_internal_wrench_1_;  // measured internal wrench robot 1 in
                                                                                         // the object frame
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_internal_wrench_2_;  // measured internal wrench in robot 2
                                                                                         // in the object frame                                                                                  
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_object_wrench_;  // overall wrench applied to the
                                                                                       // object

  // transformation matrices
  Eigen::Matrix<double, 4, 4> bTfkine1_;
  Eigen::Matrix<double, 4, 4> bTfkine2_;
  Eigen::Matrix<double, 4, 4> bTo_;

  // relative_twist
  Eigen::Matrix<double, 6, 1> relative_twist_;

  // wrench variables
  Eigen::Matrix<double, 6, 1> internal_wrench_;
  Eigen::Matrix<double, 6, 1> desired_internal_wrench_;
  Eigen::Matrix<double, 6, 1> fkine1_h_fkine1_;  // wrench robot 1 in the robot 1 fkine frame
  Eigen::Matrix<double, 6, 1> fkine2_h_fkine2_;  // wrench robot 2 in the robot 2 fkine frame

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // parameters
  double sample_time_;
  Eigen::Matrix<double, 6, 6> force_control_gain_matrix_;

  // other variables
  bool object_pose_read_ = false;
  bool control_active_ = false;
  bool fkine_robot1_read = false;
  bool fkine_robot2_read = false;
  bool robot1_wrench_read = false;
  bool robot2_wrench_read = false;
  std::string robot1_prefix;
  std::string robot2_prefix;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InternalForceControlNode>());
  rclcpp::shutdown();
  return 0;
}