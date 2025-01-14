#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <uclv_robot_ros/utils.hpp>
#include <uclv_robot_ros/CartesianTrajectoryClient.hpp>
#include <uclv_robot_ros/JointTrajectoryClient.hpp>

#include "dual_arm_control_interfaces/srv/ekf_service.hpp"

#include <eigen3/Eigen/Geometry>
#include <chrono>
#include <memory>
#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

/*
    demo node for cooperative robots taks execution
    - use_force_control: bool, use force control or not
    - use_object_pose_control: bool, use object pose control or not
*/

void print_joint_positions(const std::vector<double>& q)
{
  for (const auto& q_i : q)
  {
    std::cout << q_i << " ";
  }
  std::cout << std::endl;
}
auto call_ekf_service(const std::shared_ptr<rclcpp::Node>& node,
                      const std::shared_ptr<rclcpp::Client<dual_arm_control_interfaces::srv::EKFService>>& ekf_client,
                      const std::string& obj_name, const std::string& yaml_path)
{
  RCLCPP_INFO(node->get_logger(), "Calling EKF service to get object pose");
  auto request = std::make_shared<dual_arm_control_interfaces::srv::EKFService::Request>();

  request->object_name.data = obj_name;
  request->yaml_file_path.data = yaml_path;

  request->object_pose.pose.position.x = -0.037;
  request->object_pose.pose.position.y = 0.0135;
  request->object_pose.pose.position.z = 0.196;
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

  request->transform_error.pose.position.x = 0.0;
  request->transform_error.pose.position.y = 0.0;
  request->transform_error.pose.position.z = 0.0;
  request->transform_error.pose.orientation.x = 0.0;
  request->transform_error.pose.orientation.y = 0.0;
  request->transform_error.pose.orientation.z = 0.0;
  request->transform_error.pose.orientation.w = 1.0;

  while (!ekf_client->wait_for_service(std::chrono::seconds(2)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }

  auto result = ekf_client->async_send_request(request);
  return result;
}

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    RCLCPP_ERROR(rclcpp::get_logger("cooperative_robots_demo"),
                 "Usage: demo_node <use_force_control> <use_object_pose_control> <use_ekf>");
    return 1;
  }

  bool use_force_control = argv[1];
  bool use_object_pose_control = argv[2];
  bool use_ekf = argv[3];

  std::cout << "use_force_control: " << use_force_control << std::endl;
  std::cout << "use_object_pose_control: " << use_object_pose_control << std::endl;
  std::cout << "use_ekf: " << use_ekf << std::endl;

  // parameters

  const std::string joint_state_topic_robot1 = "/robot1/joint_states";
  const std::string joint_state_topic_robot2 = "/robot2/joint_states";
  const std::string obj_pose_topic = "/ekf/object_pose";

  //-----------

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cooperative_robots_demo");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // READ YAML FILE WITH OBJECT INFORMATION
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("dual_arm_control");
  std::string obj_yaml_path = package_share_directory + "/config/config.yaml";
  RCLCPP_INFO(node->get_logger(), "Loading Configuration from %s\n", obj_yaml_path.c_str());
  YAML::Node obj_yaml = YAML::LoadFile(obj_yaml_path.c_str());

  // READ YAML FILE WITH TASK INFORMATION
  std::string task_yaml_path = package_share_directory + "/config/task.yaml";
  RCLCPP_INFO(node->get_logger(), "Loading Configuration from %s\n", task_yaml_path.c_str());
  YAML::Node task_yaml = YAML::LoadFile(task_yaml_path.c_str());

  std::vector<double> q_robot1_home = task_yaml["q_robot1_home"].as<std::vector<double>>();
  std::vector<double> q_robot2_home = task_yaml["q_robot2_home"].as<std::vector<double>>();

  std::cout << "\nq_robot1_home: ";
  print_joint_positions(q_robot1_home);

  std::cout << "\nq_robot2_home: ";
  print_joint_positions(q_robot2_home);

  std::vector<std::string> objects_task_list;
  std::cout << "\nObjects in the task: " << std::endl;
  for (const auto& obj : task_yaml["objects"])
  {
    objects_task_list.push_back(obj["object"]["name"].as<std::string>());
    std::cout << "-> " << obj["object"]["name"].as<std::string>() << std::endl;
  }
  if (objects_task_list.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "No objects in the task list");
    return 1;
  }

  // START COOPERATIVE TASK

  // 1. MOVE ROBOTS TO INITIAL JOINT POSITION (HOME)
  //   double duration = 7.0;
  //   uclv::ros::JointTrajectoryClient joint_client_robot1(node, "/robot1/generate_joint_trajectory");

  //   RCLCPP_INFO(node->get_logger(), "Executing go_to robot 1 home");
  //   joint_client_robot1.goTo(joint_state_topic_robot1, q_robot1_home, rclcpp::Duration::from_seconds(duration),
  //                            rclcpp::Time(0), true);

  //   RCLCPP_INFO(node->get_logger(), "Executing go_to robot 2 home");
  //   uclv::ros::JointTrajectoryClient joint_client_robot2(node, "/robot2/generate_joint_trajectory");
  //   joint_client_robot2.goTo(joint_state_topic_robot2, q_robot2_home, rclcpp::Duration::from_seconds(duration),
  //                            rclcpp::Time(0), true);

  // iterate over the objects in the task list

  auto ekf_client = node->create_client<dual_arm_control_interfaces::srv::EKFService>("ekf_service");

  for (const auto& obj_name : objects_task_list)
  {
    std::cout << "PROCESSING Object: " << obj_name << std::endl;
    // 2. CALL EKF SERVICE TO GET OBJECT POSE

    if (use_ekf)
    {
      call_ekf_service(node, ekf_client, obj_name, obj_yaml_path);
    }

    // wait some time to get the object pose
    rclcpp::sleep_for(std::chrono::seconds(2));

    // read object pose from the topic
    auto object_pose = uclv::ros::waitForMessage<geometry_msgs::msg::PoseStamped>(obj_pose_topic, node);

    //
  }

  //   uclv::ros::CartesianTrajectoryClient cartesian_client(node);

  //   RCLCPP_INFO(node->get_logger(), "Executing go_to");

  //   geometry_msgs::msg::Pose target_pose;
  //   target_pose.position.x = px;
  //   target_pose.position.y = py;
  //   target_pose.position.z = pz;
  //   target_pose.orientation.w = qw;
  //   target_pose.orientation.x = qx;
  //   target_pose.orientation.y = qy;
  //   target_pose.orientation.z = qz;

  //   // read initial pose from the cartesian topic using wait_for_message
  //   const std::string cartesian_topic_const = cartesian_topic;
  //   auto initial_pose = uclv::ros::waitForMessage<geometry_msgs::msg::PoseStamped>(cartesian_topic_const, node);

  //   cartesian_client.goTo(initial_pose->pose, target_pose, rclcpp::Duration::from_seconds(duration));

  rclcpp::shutdown();
  return 0;
}
