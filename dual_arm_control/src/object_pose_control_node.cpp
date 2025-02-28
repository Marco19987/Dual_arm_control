#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <eigen3/Eigen/Geometry>
#include "geometry_helper.hpp"

class ObjectPoseControlNode : public rclcpp::Node
{
public:
  ObjectPoseControlNode() : Node("object_pose_control"), control_active_(false)
  {
    // declare parameters
    this->declare_parameter("sample_time", 0.02);
    this->get_parameter("sample_time", sample_time_);

    this->declare_parameter("control_gain_diag_vector", std::vector<double>({ 1e-1, 1e-1, 1e-1, 1e-2, 1e-2, 1e-2 }));
    std::vector<double> control_gain_diag_vector;
    this->get_parameter("control_gain_diag_vector", control_gain_diag_vector);
    control_gain_matrix_.setZero();
    control_gain_matrix_.diagonal() << control_gain_diag_vector[0], control_gain_diag_vector[1],
        control_gain_diag_vector[2], control_gain_diag_vector[3], control_gain_diag_vector[4],
        control_gain_diag_vector[5];

    // declare subscribers and publishers
    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);
    sub_object_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object_pose", qos,
        std::bind(&ObjectPoseControlNode::objectPoseCallback, this, std::placeholders::_1));
    sub_desired_object_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/desired_object_pose", qos,
        std::bind(&ObjectPoseControlNode::desiredObjectPoseCallback, this, std::placeholders::_1));
    sub_absolute_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/absolute_pose", qos,
        std::bind(&ObjectPoseControlNode::absolutePoseCallback, this, std::placeholders::_1));

    pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("absolute_twist", qos);

    // create service
    service_ = this->create_service<std_srvs::srv::SetBool>(
        "activate_object_control",
        std::bind(&ObjectPoseControlNode::activateControlCallback, this, std::placeholders::_1, std::placeholders::_2));

    // initialize variables
    bTa_.setIdentity();
    bTo_.setIdentity();
    bTo_d_.setIdentity();
    absolute_twist_.setZero();
  }

private:
  void timerCallback()
  {
    if (object_pose_read_ && desired_object_pose_read_ && absolute_pose_read_)
    {
      Eigen::Matrix<double, 4, 4> oTa = bTo_.inverse() * bTa_;
      Eigen::Matrix<double, 4, 4> bTa_d = bTo_d_ * oTa;

      Eigen::Matrix<double, 3, 3> aRa_d = bTa_.block<3, 3>(0, 0).transpose() * bTa_d.block<3, 3>(0, 0);
      Eigen::AngleAxisd angleAxis(aRa_d);
      Eigen::Vector3d a_error_aa_d = angleAxis.angle() * angleAxis.axis();   // error a-a_d in the absolute frame
      Eigen::Vector3d b_error_aa_d = bTa_.block<3, 3>(0, 0) * a_error_aa_d;  // error a-a_d in the base frame

      absolute_twist_.block<3, 1>(0, 0) =
          control_gain_matrix_.block<3, 3>(0, 0) * (bTa_d.block<3, 1>(0, 3) - bTa_.block<3, 1>(0, 3));
      absolute_twist_.block<3, 1>(3, 0) = control_gain_matrix_.block<3, 3>(3, 3) * b_error_aa_d;

      // publish control
      geometry_msgs::msg::TwistStamped twist_msg;
      twist_msg.header.stamp = this->now();
      twist_msg.header.frame_id = base_frame_name_;
      twist_msg.twist.linear.x = absolute_twist_(0);
      twist_msg.twist.linear.y = absolute_twist_(1);
      twist_msg.twist.linear.z = absolute_twist_(2);
      twist_msg.twist.angular.x = absolute_twist_(3);
      twist_msg.twist.angular.y = absolute_twist_(4);
      twist_msg.twist.angular.z = absolute_twist_(5);
      pub_twist_->publish(twist_msg);
    }
    else
    {
      // RCLCPP_INFO(this->get_logger(), "Waiting for pose messages ...");
    }
  }

  void objectPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    base_frame_name_ = msg->header.frame_id;
    convertPoseToMatrix(msg, bTo_);
    object_pose_read_ = true;
  }

  void desiredObjectPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    convertPoseToMatrix(msg, bTo_d_);
    desired_object_pose_read_ = true;
  }

  void absolutePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    convertPoseToMatrix(msg, bTa_);
    absolute_pose_read_ = true;
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
                                       std::bind(&ObjectPoseControlNode::timerCallback, this));
    }
    response->success = true;
    response->message = control_active_ ? "Control activated" : "Control deactivated";
  }

  void reset_server()
  {
    absolute_twist_.setZero();
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = base_frame_name_;
    twist_msg.twist.linear.x = absolute_twist_(0);
    twist_msg.twist.linear.y = absolute_twist_(1);
    twist_msg.twist.linear.z = absolute_twist_(2);
    twist_msg.twist.angular.x = absolute_twist_(3);
    twist_msg.twist.angular.y = absolute_twist_(4);
    twist_msg.twist.angular.z = absolute_twist_(5);
    for (int i = 0; i < 10; i++)
      pub_twist_->publish(twist_msg);
    timer_->cancel();
    object_pose_read_ = false;
    desired_object_pose_read_ = false;
    absolute_pose_read_ = false;
    control_active_ = false;
    bTa_.setIdentity();
    bTo_.setIdentity();
    bTo_d_.setIdentity();
    absolute_twist_.setZero();
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_object_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_desired_object_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_absolute_pose_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

  // transformation matrices
  Eigen::Matrix<double, 4, 4> bTa_;
  Eigen::Matrix<double, 4, 4> bTo_;
  Eigen::Matrix<double, 4, 4> bTo_d_;

  // absolute_twist
  Eigen::Matrix<double, 6, 1> absolute_twist_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // parameters
  double sample_time_;
  Eigen::Matrix<double, 6, 6> control_gain_matrix_;

  // other variables
  std::string base_frame_name_;
  bool object_pose_read_ = false;
  bool desired_object_pose_read_ = false;
  bool absolute_pose_read_ = false;
  bool control_active_ = false;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectPoseControlNode>());
  rclcpp::shutdown();
  return 0;
}