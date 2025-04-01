#include "rclcpp/rclcpp.hpp"
#include "dual_arm_control_interfaces/srv/ekf_service.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

class EKFClient : public rclcpp::Node
{
public:
  EKFClient() : Node("ekf_client")
  {
    client_ = this->create_client<dual_arm_control_interfaces::srv::EKFService>("ekf_service");
    timer_ = this->create_wall_timer(500ms, std::bind(&EKFClient::send_request, this));
  }

private:
  void send_request()
  {
    auto request = std::make_shared<dual_arm_control_interfaces::srv::EKFService::Request>();
    // Fill in the request data here

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("dual_arm_control");
    request->yaml_file_path.data = package_share_directory + "/config/config.yaml";
    request->object_name.data = "resin_block_1";

    request->object_pose.pose.position.x = 0.8;
    request->object_pose.pose.position.y = 0.0;
    request->object_pose.pose.position.z = 0.1;
    request->object_pose.pose.orientation.x = 0.0;
    request->object_pose.pose.orientation.y = 0.0;
    request->object_pose.pose.orientation.z = 0.0;
    request->object_pose.pose.orientation.w = 1.0;

    request->object_twist.twist.linear.x = 0.0;
    request->object_twist.twist.linear.y = 0.0;
    request->object_twist.twist.linear.z = 0.0;
    request->object_twist.twist.angular.x = 0.0;
    request->object_twist.twist.angular.y = 0.0;
    request->object_twist.twist.angular.z = 0.0;

    request->robots_relative_transform.pose.position.x = 0.0;
    request->robots_relative_transform.pose.position.y = 0.0;
    request->robots_relative_transform.pose.position.z = 0.0;
    request->robots_relative_transform.pose.orientation.x = 0.0;
    request->robots_relative_transform.pose.orientation.y = 0.0;
    request->robots_relative_transform.pose.orientation.z = 1.0;
    request->robots_relative_transform.pose.orientation.w = 0.0;




    while (!client_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto result = client_->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Service call succeeded");
      // Process the result here
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service ekf");
    }
  }

  rclcpp::Client<dual_arm_control_interfaces::srv::EKFService>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFClient>());
  rclcpp::shutdown();
  return 0;
}