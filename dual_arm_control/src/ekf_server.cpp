/** This node implement the EKF for the robots_object_system system
 * The service takes as input the path to a yaml file containing the parameters of the objects and the topics where to
 * subscribe and read the poses, and the object name to be tracked. When a new service request is received, the node
 * subscribes to the topics and starts the EKF for the requested object.
 *
 *
 *
 *
 * */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "dual_arm_control_interfaces/srv/ekf_service.hpp"

#include "../include/robots_object_system.hpp"
#include "../include/robots_object_system_ext.hpp"

#include <eigen3/Eigen/Geometry>
#include <chrono>
#include <memory>
#include <yaml-cpp/yaml.h>

class EKFServer : public rclcpp::Node
{
public:
  EKFServer() : Node("ekf_server")
  {
    // Define reetrant cb group
    reentrant_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    options_cb_group.callback_group = reentrant_cb_group_;

    // Create the pose subscriber
    // subscribe_to_pose_topic("/dope/pose_" + object_name_);

    // // Create the depth subscriber
    // std::string camera_topic = "/camera/aligned_depth_to_color/image_raw";
    // depth_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
    //     camera_topic, rclcpp::SensorDataQoS(), std::bind(&EKFServer::depth_callback, this, std::placeholders::_1),
    //     options_cb_group);

    // // Create the depth optimization client
    // client = this->create_client<depth_optimization_interfaces::srv::DepthOptimize>(
    //     "/depth_optimize", rmw_qos_profile_services_default, reentrant_cb_group_);

    // Create the service server
    server_ = this->create_service<dual_arm_control_interfaces::srv::EKFService>(
        "ekf_service",
        std::bind(&EKFServer::handle_service_request, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "EKFService service server ready!");
  }

private:
  rclcpp::Service<dual_arm_control_interfaces::srv::EKFService>::SharedPtr server_;
  rclcpp::CallbackGroup::SharedPtr
      reentrant_cb_group_;  // see https://docs.ros.org/en/foxy/How-To-Guides/Using-callback-groups.html
  rclcpp::SubscriptionOptions options_cb_group;

  void handle_service_request(const std::shared_ptr<dual_arm_control_interfaces::srv::EKFService::Request> request,
                              std::shared_ptr<dual_arm_control_interfaces::srv::EKFService::Response> response)
  {
    // read yaml file
    read_yaml_file(request->yaml_file_path.data.c_str(), request->object_name.data.c_str());

    // initialize subscribers to the topics and publishers

    // initialize the EKF

    // start the EKF

    // return the response
  }

  void read_yaml_file(const std::string& yaml_file_path, const std::string& object_name)
  {
    // read the yaml file
    RCLCPP_INFO(this->get_logger(), "Loading Configuration from %s", yaml_file_path.c_str());
    YAML::Node config = YAML::LoadFile(yaml_file_path);

    // read b1Tb2
    Eigen::Matrix<double, 4, 4> b1Tb2;
    std::vector<double> translation = config["b1Tb2"]["translation"].as<std::vector<double>>();
    b1Tb2.block<3,1>(0,3) = Eigen::Vector3d(translation[0], translation[1], translation[2]);

    std::cout << "b1Tb2: \n" << b1Tb2 << std::endl; 
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto ekf_server = std::make_shared<EKFServer>();
  executor.add_node(ekf_server);

  RCLCPP_INFO(ekf_server->get_logger(), "Starting ekf_server node, shut down with CTRL-C");
  executor.spin();
  RCLCPP_INFO(ekf_server->get_logger(), "Keyboard interrupt, shutting down.\n");

  rclcpp::shutdown();
  return 0;
}