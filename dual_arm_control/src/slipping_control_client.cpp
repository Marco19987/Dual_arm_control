// This node implements a dummy service server that is used to 
// use the slipping control client in a ROS1 environment from a ROS2 environment.

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <uclv_slipping_control_common/Slipping_Control_Client.h>

using std::placeholders::_1;
using std::placeholders::_2;

class SlippingControlClientNode : public rclcpp::Node
{
public:
  SlippingControlClientNode()
  : Node("slipping_client_ros2_server")
  {
    gripper_active_ = true;
    one_ls_ = true;
    wait_for_servers_ = true;
    grasp_force_ = 3.0;

    slipping_client_ = std::make_shared<Slipping_Control_Client>(shared_from_this(), gripper_active_, one_ls_, wait_for_servers_);

    homing_service_ = this->create_service<std_srvs::srv::SetBool>(
      "slipping_client_bridge_homing_srv",
      std::bind(&SlippingControlClientNode::homing_cb, this, _1, _2)
    );
    RCLCPP_INFO(this->get_logger(), "Ready to execute homing commands.");

    grasping_service_ = this->create_service<std_srvs::srv::SetBool>(
      "slipping_client_bridge_grasping_srv",
      std::bind(&SlippingControlClientNode::grasping_cb, this, _1, _2)
    );
    RCLCPP_INFO(this->get_logger(), "Ready to execute grasping commands.");

    pivoting_service_ = this->create_service<std_srvs::srv::SetBool>(
      "slipping_client_bridge_pivoting_srv",
      std::bind(&SlippingControlClientNode::pivoting_cb, this, _1, _2)
    );
    RCLCPP_INFO(this->get_logger(), "Ready to execute pivoting commands.");

    slipping_avoidance_service_ = this->create_service<std_srvs::srv::SetBool>(
      "slipping_client_bridge_slipping_avoidance_srv",
      std::bind(&SlippingControlClientNode::slipping_avoidance_cb, this, _1, _2)
    );
    RCLCPP_INFO(this->get_logger(), "Ready to execute slipping avoidance commands.");
  }

private:
  bool gripper_active_;
  bool one_ls_;
  bool wait_for_servers_;
  double grasp_force_;
  std::shared_ptr<Slipping_Control_Client> slipping_client_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr homing_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr grasping_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pivoting_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr slipping_avoidance_service_;

  void homing_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                 std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    RCLCPP_INFO(this->get_logger(), "Homing request: %d", req->data);
    slipping_client_->home(true, false);
    res->success = true;
    res->message = "Homing done";
  }

  void grasping_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    RCLCPP_INFO(this->get_logger(), "Grasping request: %d", req->data);
    slipping_client_->grasp(grasp_force_, 0.0, true);
    res->success = true;
    res->message = "Grasping done";
  }

  void pivoting_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    RCLCPP_INFO(this->get_logger(), "Pivoting request: %d", req->data);
    slipping_client_->gripper_pivoting(true);
    res->success = true;
    res->message = "Pivoting done";
  }

  void slipping_avoidance_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    RCLCPP_INFO(this->get_logger(), "Slipping avoidance request: %d", req->data);
    slipping_client_->slipping_avoidance(true, true);
    res->success = true;
    res->message = "Slipping avoidance done";
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlippingControlClientNode>());
  rclcpp::shutdown();
  return 0;
}