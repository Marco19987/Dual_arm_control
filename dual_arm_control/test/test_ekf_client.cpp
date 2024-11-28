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

    request->yaml_file_path.data = "path/to/your/yaml/file";
    // pass the path considering that the yaml is in the share folder

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("dual_arm_control");
    request->yaml_file_path.data = package_share_directory + "/config.yaml";
    request->object_name.data = "object_name";

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