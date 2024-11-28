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
    // declare parameters
    this->declare_parameter<std::string>("robot_1_prefix", "robot_1");
    this->get_parameter("robot_1_prefix", this->robot_1_prefix_);

    this->declare_parameter<std::string>("robot_2_prefix", "robot_2");
    this->get_parameter("robot_2_prefix", this->robot_2_prefix_);

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
    RCLCPP_INFO(this->get_logger(), "Loading Configuration from %s\n", yaml_file_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Object Selected %s", object_name.c_str());
    YAML::Node config = YAML::LoadFile(yaml_file_path);

    // check if the object is in the yaml file
    if (!config[object_name])
    {
      RCLCPP_ERROR(this->get_logger(), "Object %s not found in the YAML file", object_name.c_str());
      return;
    }

    // read gravity vector
    Eigen::Matrix<double, 3, 1> bg;
    bg.setZero();
    std::vector<double> gravity = config["gravity_vector"].as<std::vector<double>>();
    bg << gravity[0], gravity[1], gravity[2];
    std::cout << "Gravity Vector: \n" << bg << std::endl;

    // read b1Tb2
    Eigen::Matrix<double, 4, 4> b1Tb2;
    b1Tb2.setIdentity();
    read_transform(config["b1Tb2"], b1Tb2);
    std::cout << "b1Tb2: \n" << b1Tb2 << std::endl;

    // read bTb1
    Eigen::Matrix<double, 4, 4> bTb1;
    bTb1.setIdentity();
    read_transform(config["bTb1"], bTb1);
    std::cout << "bTb1: \n" << bTb1 << std::endl;

    YAML::Node object_node = config[object_name];

    // read viscous friction
    Eigen::Matrix<double, 6, 6> viscous_friction_matrix;
    read_viscous_friction(object_node, viscous_friction_matrix);

    // read Bm
    Eigen::Matrix<double, 6, 6> Bm;
    read_inertia_matrix(object_node, Bm);

    // read oTg1
    Eigen::Matrix<double, 4, 4> oTg1;
    oTg1.setIdentity();
    read_transform(object_node["oTg1"], oTg1);
    std::cout << "oTg1: \n" << oTg1 << std::endl;

    // read oTg2
    Eigen::Matrix<double, 4, 4> oTg2;
    oTg2.setIdentity();
    read_transform(object_node["oTg2"], oTg2);
    std::cout << "oTg2: \n" << oTg2 << std::endl;

    // read names of frames published
    std::vector<std::string> frame_names;
    for (const auto& transformation : object_node["aruco_transforms"])
    {
      frame_names.push_back(transformation["name"].as<std::string>());
      RCLCPP_INFO(this->get_logger(), "Frame: %s", transformation["name"].as<std::string>().c_str());
    }

    // instantiate the subscribers to the pose topics
    int num_frames = frame_names.size();
    int index = 0;
    for (const auto& frame_name : frame_names)
    {
      pose_subscribers_.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/" + object_name + "/" + frame_name + "/" + this->robot_1_prefix_ + "/pose", 1,
          [this, index](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->pose_callback(msg, index); }));

      pose_subscribers_.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/" + object_name + "/" + frame_name + "/" + this->robot_2_prefix_ + "/pose", 1,
          [this, num_frames, index](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            this->pose_callback(msg, num_frames + index);
          }));

      index++;
    }
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const int& index)
  {
    RCLCPP_INFO(this->get_logger(), "Received pose from %d", index);
  }

  void read_inertia_matrix(const YAML::Node& object, Eigen::Matrix<double, 6, 6>& Bm)
  {
    if (object["inertia_matrix"])
    {
      std::vector<double> inertia_matrix = object["inertia_matrix"].as<std::vector<double>>();
      if (inertia_matrix.size() == 6)
      {
        Bm.setZero();
        Bm.diagonal() << inertia_matrix[0], inertia_matrix[1], inertia_matrix[2], inertia_matrix[3], inertia_matrix[4],
            inertia_matrix[5];
        std::cout << "Inertia Matrix: \n" << Bm << std::endl;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Inertia matrix vector size is not 6");
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Inertia matrix not found in the YAML file");
    }
  }

  void read_viscous_friction(const YAML::Node& object, Eigen::Matrix<double, 6, 6>& viscous_friction_matrix)
  {
    if (object["viscous_friction"])
    {
      std::vector<double> viscous_friction = object["viscous_friction"].as<std::vector<double>>();
      if (viscous_friction.size() == 6)
      {
        viscous_friction_matrix.setZero();
        viscous_friction_matrix.diagonal() << viscous_friction[0], viscous_friction[1], viscous_friction[2],
            viscous_friction[3], viscous_friction[4], viscous_friction[5];
        std::cout << "viscous_friction: \n" << viscous_friction_matrix << std::endl;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "viscous_friction vector size is not 6");
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "viscous_friction not found in the YAML file");
    }
  }

  void read_transform(const YAML::Node& node, Eigen::Matrix<double, 4, 4>& T)
  {
    std::vector<double> translation = node["translation"].as<std::vector<double>>();
    std::vector<double> quaternion = node["quaternion"].as<std::vector<double>>();
    // swap order of quaternion from x y z w to w x y z
    std::swap(quaternion[0], quaternion[3]);

    T.block<3, 1>(0, 3) = Eigen::Vector3d(translation[0], translation[1], translation[2]);
    Eigen::Quaterniond q(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
  }

  rclcpp::Service<dual_arm_control_interfaces::srv::EKFService>::SharedPtr server_;
  rclcpp::CallbackGroup::SharedPtr
      reentrant_cb_group_;  // see https://docs.ros.org/en/foxy/How-To-Guides/Using-callback-groups.html
  rclcpp::SubscriptionOptions options_cb_group;

  // define vecor of subscribers to poseStamped
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subscribers_;

  // strings to attach at the topic name to subscribe
  std::string robot_1_prefix_;
  std::string robot_2_prefix_;

  
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