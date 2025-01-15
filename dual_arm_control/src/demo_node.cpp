#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <uclv_robot_ros/utils.hpp>
#include <uclv_robot_ros/CartesianTrajectoryClient.hpp>
#include <uclv_robot_ros/JointTrajectoryClient.hpp>

#include "dual_arm_control_interfaces/srv/ekf_service.hpp"

#include "uclv_aruco_detection_interfaces/srv/pose_service.hpp"

#include <std_srvs/srv/set_bool.hpp>

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
void eigen_matrix_to_pose_msg(Eigen::Matrix<double, 4, 4>& T, geometry_msgs::msg::Pose& pose)
{
  pose.position.x = T(0, 3);
  pose.position.y = T(1, 3);
  pose.position.z = T(2, 3);
  Eigen::Quaterniond q(T.block<3, 3>(0, 0));
  q.normalize();
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
}

void pose_msg_to_eigen_matrix(const geometry_msgs::msg::Pose& pose, Eigen::Matrix<double, 4, 4>& T)
{
  T.setIdentity();
  T.block<3, 1>(0, 3) = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  T.block<3, 3>(0, 0) = q.toRotationMatrix();
}

void read_transform(const YAML::Node& node, Eigen::Matrix<double, 4, 4>& T)
{
  std::vector<double> translation = node["translation"].as<std::vector<double>>();
  std::vector<double> quaternion = node["quaternion"].as<std::vector<double>>();
  // swap order of quaternion from x y z w to w x y z
  std::vector<double> quaternion_swap = { quaternion[3], quaternion[0], quaternion[1], quaternion[2] };

  T.block<3, 1>(0, 3) = Eigen::Vector3d(translation[0], translation[1], translation[2]);
  Eigen::Quaterniond q(quaternion_swap[0], quaternion_swap[1], quaternion_swap[2], quaternion_swap[3]);
  T.block<3, 3>(0, 0) = q.toRotationMatrix();
}

void wait_for_enter()
{
  std::cout << "Press ENTER to continue..." << std::endl;
  std::cin.ignore();
}

void fill_pose(geometry_msgs::msg::Pose& pose, double px, double py, double pz, double qw, double qx, double qy,
               double qz)
{
  pose.position.x = px;
  pose.position.y = py;
  pose.position.z = pz;
  pose.orientation.w = qw;
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;
}
void fill_twist(geometry_msgs::msg::Twist& twist, double vx, double vy, double vz, double wx, double wy, double wz)
{
  twist.linear.x = vx;
  twist.linear.y = vy;
  twist.linear.z = vz;
  twist.angular.x = wx;
  twist.angular.y = wy;
  twist.angular.z = wz;
}

void print_pose_stamped(geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  // print heaer
  std::cout << "Header: " << std::endl;
  std::cout << "Frame ID: " << pose->header.frame_id << std::endl;
  std::cout << "Stamp: " << pose->header.stamp.sec << " " << pose->header.stamp.nanosec << std::endl;
  // print pose
  std::cout << "Pose: " << std::endl;
  std::cout << "Position: " << pose->pose.position.x << " " << pose->pose.position.y << " " << pose->pose.position.z
            << std::endl;
  std::cout << "Orientation: " << pose->pose.orientation.x << " " << pose->pose.orientation.y << " "
            << pose->pose.orientation.z << " " << pose->pose.orientation.w << std::endl;
}

void print_joint_positions(const std::vector<double>& q)
{
  for (const auto& q_i : q)
  {
    std::cout << q_i << " ";
  }
  std::cout << std::endl;
}

template <typename ServiceT>
auto call_service(rclcpp::executors::SingleThreadedExecutor& executor,
                  const std::shared_ptr<rclcpp::Client<ServiceT>>& client,
                  typename ServiceT::Request::SharedPtr request, typename ServiceT::Response::SharedPtr response)

{
  std::cout << "Calling service" << std::endl;

  while (!client->wait_for_service(std::chrono::seconds(2)))
  {
    if (!rclcpp::ok())
    {
      std::cerr << "Interrupted while waiting for the service. Exiting." << std::endl;
      throw std::runtime_error("Interrupted while waiting for the service. Exiting.");
    }
    std::cout << "Service not available, waiting again..." << std::endl;
  }

  auto result = client->async_send_request(request);

  // Wait for the result using the executor if provided.
  while (rclcpp::ok())
  {
    if (result.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      if (result.get()->success)
      {
        std::cout << "Service call succeeded" << std::endl;
        // Process the result here
        return result;
      }
      else
      {
        std::cerr << "Failed to call service" << std::endl;
        throw std::runtime_error("Failed to call service");
      }
    }
  }
  throw std::runtime_error("Service call interrupted");
}

bool stringToBool(const std::string& str)
{
  return str == "true" || str == "1";
}

int main(int argc, char** argv)
{
  if (argc < 5)
  {
    RCLCPP_ERROR(rclcpp::get_logger("cooperative_robots_demo"),
                 "Usage: demo_node <use_force_control> <use_object_pose_control> <use_ekf> <use_estimated_b1Tb2>");
    return 1;
  }

  bool use_force_control = stringToBool(argv[1]);
  bool use_object_pose_control = stringToBool(argv[2]);
  bool use_ekf = stringToBool(argv[3]);
  bool use_estimated_b1Tb2 = stringToBool(argv[4]);

  std::cout << "use_force_control: " << use_force_control << std::endl;
  std::cout << "use_object_pose_control: " << use_object_pose_control << std::endl;
  std::cout << "use_ekf: " << use_ekf << std::endl;
  std::cout << "use_estimated_b1Tb2: " << use_estimated_b1Tb2 << std::endl;

  // init ros
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cooperative_robots_demo");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // PARAMETERS ---------------------------------
  const std::string fkine_robot1_topic = "/robot1/fkine";
  const std::string fkine_robot2_topic = "/robot2/fkine";
  const std::string joint_state_topic_robot1 = "/robot1/joint_states";
  const std::string joint_state_topic_robot2 = "/robot2/joint_states";
  const std::string obj_pose_topic = "/ekf/object_pose";
  const std::string filtered_b1Tb2_topic = "/ekf/b1Tb2_filtered";
  const std::string package_share_directory = ament_index_cpp::get_package_share_directory("dual_arm_control");
  const std::string obj_yaml_path = package_share_directory + "/config/config.yaml";
  const std::string task_yaml_path = package_share_directory + "/config/task.yaml";

  double duration_home_robots = 7.0;
  double duration_pregrasp = 5.0;
  double duration_grasp = 5.0;
  
  Eigen::Vector3d pregrasp_offset_single_robot(0.0, 0.0, -0.1);  // offset from object pose for pregrasp pose
  Eigen::Vector3d offset_from_initial_pose(0.0, 0.0, 0.1);  // (base frame) offset from initial pose for intermediate
                                                            // poses in the cartesian trajectory
  Eigen::Vector3d offset_from_final_pose(0.0, 0.0, 0.1);  // (base frame) offset from final pose for intermediate poses
                                                          // in the cartesian trajectory

  //---------------------------------------------

  // ROS objects --------------------------------
  auto ekf_client = node->create_client<dual_arm_control_interfaces::srv::EKFService>("ekf_service");
  auto aurco_pose_conversion_client_camera1 =
      node->create_client<uclv_aruco_detection_interfaces::srv::PoseService>("/camera_1/pose_conversion_service");
  auto aurco_pose_conversion_client_camera2 =
      node->create_client<uclv_aruco_detection_interfaces::srv::PoseService>("/camera_2/pose_conversion_service");
  auto activate_joint_integrator_client_robot1 = node->create_client<std_srvs::srv::SetBool>("/robot1/joint_integrator/set_running");
  auto activate_joint_integrator_client_robot2 = node->create_client<std_srvs::srv::SetBool>("/robot2/joint_integrator/set_running");

  // Objects to store ----------------------------
  uclv::ros::JointTrajectoryClient joint_client_robot1(node, "/robot1/generate_joint_trajectory");
  uclv::ros::JointTrajectoryClient joint_client_robot2(node, "/robot2/generate_joint_trajectory");
  uclv::ros::CartesianTrajectoryClient cartesian_client_robot1(node, "/robot1/generate_cartesian_trajectory");
  uclv::ros::CartesianTrajectoryClient cartesian_client_robot2(node, "/robot2/generate_cartesian_trajectory");


  //---------------------------------------------

  // READ YAML FILE WITH OBJECT INFORMATION
  RCLCPP_INFO(node->get_logger(), "Loading Configuration from %s\n", obj_yaml_path.c_str());
  YAML::Node obj_yaml = YAML::LoadFile(obj_yaml_path.c_str());

  // READ YAML FILE WITH TASK INFORMATION
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

  // read b1Tb2
  Eigen::Matrix<double, 4, 4> b1Tb2;
  b1Tb2.setIdentity();
  read_transform(obj_yaml["b1Tb2"], b1Tb2);
  std::cout << "b1Tb2: \n" << b1Tb2 << std::endl;

  // read bTb1
  Eigen::Matrix<double, 4, 4> bTb1;
  bTb1.setIdentity();
  read_transform(obj_yaml["bTb1"], bTb1);
  std::cout << "bTb1: \n" << bTb1 << std::endl;

  // ############################### START COOPERATIVE TASK ################################################

  // 1. MOVE ROBOTS TO INITIAL JOINT POSITION (HOME)
  RCLCPP_INFO(node->get_logger(), "Executing go_to robot 1 home");

  // the first call to goTo may fail, so we need to retry
  bool success = false;
  while (!success)
  {
    try
    {
      joint_client_robot1.goTo(joint_state_topic_robot1, q_robot1_home,
                               rclcpp::Duration::from_seconds(duration_home_robots), rclcpp::Time(0), true);
      success = true;
    }
    catch (const std::runtime_error& e)
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to move robot 1 to home position: %s", e.what());
      RCLCPP_INFO(node->get_logger(), "Retrying to move robot 1 to home position...");
      rclcpp::sleep_for(std::chrono::seconds(2));
    }
  }

  RCLCPP_INFO(node->get_logger(), "Executing go_to robot 2 home");
  joint_client_robot2.goTo(joint_state_topic_robot2, q_robot2_home,
                           rclcpp::Duration::from_seconds(duration_home_robots), rclcpp::Time(0), true);

  // iterate over the objects in the task list
  for (const auto& obj_name : objects_task_list)
  {
    std::cout << "PROCESSING Object: " << obj_name << std::endl;

    // 2. CALL EKF SERVICE TO GET OBJECT POSE
    if (use_ekf)
    {
      // call aruco conversion services
      auto request_aruco = std::make_shared<uclv_aruco_detection_interfaces::srv::PoseService::Request>();
      request_aruco->object_name.data = obj_name;
      request_aruco->yaml_file_path.data = obj_yaml_path;
      std::cout << "Calling aruco conversion service camera 1" << std::endl;
      call_service(executor, aurco_pose_conversion_client_camera1, request_aruco,
                   uclv_aruco_detection_interfaces::srv::PoseService::Response::SharedPtr());
      std::cout << "Calling aruco conversion service camera 2" << std::endl;
      call_service(executor, aurco_pose_conversion_client_camera2, request_aruco,
                   uclv_aruco_detection_interfaces::srv::PoseService::Response::SharedPtr());

      // call EKF service
      auto request = std::make_shared<dual_arm_control_interfaces::srv::EKFService::Request>();
      request->object_name.data = obj_name;
      request->yaml_file_path.data = obj_yaml_path;
      fill_pose(request->object_pose.pose, -0.37, 0.0135, 0.196, 1.0, 0.0, 0.0, 0.0);
      fill_twist(request->object_twist.twist, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      fill_pose(request->transform_error.pose, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);

      std::cout << "Calling EKF service" << std::endl;
      call_service(executor, ekf_client, request, dual_arm_control_interfaces::srv::EKFService::Response::SharedPtr());
    }

    // wait some time to get the object pose
    // rclcpp::sleep_for(std::chrono::seconds(2));

    // wait for ENTER to continue
    wait_for_enter();

    // if use_estimated_b1Tb2 is true, then read the estimated b1Tb2 from the topic
    if (use_estimated_b1Tb2)
    {
      // read estimated b1Tb2 from the topic
      auto estimated_b1Tb2_ptr = uclv::ros::waitForMessage<geometry_msgs::msg::PoseStamped>(filtered_b1Tb2_topic, node);
      auto estimated_b1Tb2 = std::const_pointer_cast<geometry_msgs::msg::PoseStamped>(estimated_b1Tb2_ptr);

      pose_msg_to_eigen_matrix(estimated_b1Tb2->pose, b1Tb2);
      std::cout << "Estimated b1Tb2: \n" << b1Tb2 << std::endl;
    }

    // read object pose from the topic
    auto object_pose_ptr = uclv::ros::waitForMessage<geometry_msgs::msg::PoseStamped>(obj_pose_topic, node);
    auto object_pose = std::const_pointer_cast<geometry_msgs::msg::PoseStamped>(object_pose_ptr);

    Eigen::Matrix<double, 4, 4> bTo = Eigen::Matrix<double, 4, 4>::Identity();
    pose_msg_to_eigen_matrix(object_pose->pose, bTo);
    std::cout << "Object pose: \n" << bTo << std::endl;

    // 3. MOVE ROBOTS TO GRASP POSES

    // read predefined grasp poses from the object yaml file

    // read oTg1
    Eigen::Matrix<double, 4, 4> oTg1;
    oTg1.setIdentity();
    read_transform(obj_yaml[obj_name]["oTg1"], oTg1);
    std::cout << "oTg1: \n" << oTg1 << std::endl;

    // read oTg2
    Eigen::Matrix<double, 4, 4> oTg2;
    oTg2.setIdentity();
    read_transform(obj_yaml[obj_name]["oTg2"], oTg2);
    std::cout << "oTg2: \n" << oTg2 << std::endl;

    // compute b1Tg1 and b2Tg2
    Eigen::Matrix<double, 4, 4> b1Tg1 = bTb1.inverse() * bTo * oTg1;
    std::cout << "b1Tg1: \n" << b1Tg1 << std::endl;

    Eigen::Matrix<double, 4, 4> b2Tg2 = b1Tb2.inverse() * bTb1.inverse() * bTo * oTg2;
    std::cout << "b2Tg2: \n" << b2Tg2 << std::endl;

    // define pre-grasp poses
    Eigen::Matrix<double, 4, 4> g1Tg1_pregrasp = Eigen::Matrix<double, 4, 4>::Identity();
    g1Tg1_pregrasp.block<3, 1>(0, 3) = pregrasp_offset_single_robot;
    Eigen::Matrix<double, 4, 4> b1Tg1_pregrasp = b1Tg1 * g1Tg1_pregrasp;
    std::cout << "b1Tg1_pregrasp: \n" << b1Tg1_pregrasp << std::endl;

    Eigen::Matrix<double, 4, 4> g2Tg2_pregrasp = Eigen::Matrix<double, 4, 4>::Identity();
    g2Tg2_pregrasp.block<3, 1>(0, 3) = pregrasp_offset_single_robot;
    Eigen::Matrix<double, 4, 4> b2Tg2_pregrasp = b2Tg2 * g2Tg2_pregrasp;
    std::cout << "b2Tg2_pregrasp: \n" << b2Tg2_pregrasp << std::endl;

    // Single Cartesian Movement

    // activate joint integrators
    std::shared_ptr<std_srvs::srv::SetBool::Request> request_activate_joint_integrator =
        std::make_shared<std_srvs::srv::SetBool::Request>();
    request_activate_joint_integrator->data = true;
    call_service(executor, activate_joint_integrator_client_robot1, request_activate_joint_integrator,
                 std_srvs::srv::SetBool::Response::SharedPtr());

    call_service(executor, activate_joint_integrator_client_robot2, request_activate_joint_integrator,
                  std_srvs::srv::SetBool::Response::SharedPtr()); 

    // move robot 1 to pregrasp pose
    std::cout << "Moving robot 1 to pregrasp pose" << std::endl;
    geometry_msgs::msg::Pose target_pose;
    eigen_matrix_to_pose_msg(b1Tg1_pregrasp, target_pose);
    cartesian_client_robot1.goTo(fkine_robot1_topic,target_pose, rclcpp::Duration::from_seconds(duration_pregrasp));

  

    // 4. COOPERATIVE MANIPULATION

    // read final_pose bTo_final
    Eigen::Matrix<double, 4, 4> bTo_final;
    bTo_final.setIdentity();
    read_transform(task_yaml["objects"][objects_task_list.size() - 1]["object"]["bTo_final"], bTo_final);

    // compute intermediate poses for cartesian trajectory
    Eigen::Matrix<double, 4, 4> bTo_second = bTo;
    bTo_second.block<3, 1>(0, 3) = bTo_second.block<3, 1>(0, 3) + offset_from_initial_pose;
    Eigen::Matrix<double, 4, 4> bTo_third = bTo_final;
    bTo_third.block<3, 1>(0, 3) = bTo_third.block<3, 1>(0, 3) + offset_from_final_pose;
  }

  

  rclcpp::shutdown();
  return 0;
}
