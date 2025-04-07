#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <uclv_robot_ros/utils.hpp>
#include <uclv_robot_ros/CartesianTrajectoryClient.hpp>
#include <uclv_robot_ros/JointTrajectoryClient.hpp>

#include <uclv_robot_ros_msgs/action/cartesian_trajectory.hpp>
#include <uclv_robot_ros_msgs/srv/set_end_effector.hpp>

#include "dual_arm_control_interfaces/srv/ekf_service.hpp"

#include "uclv_aruco_detection_interfaces/srv/pose_service.hpp"

#include <std_srvs/srv/set_bool.hpp>

#include <rcl_interfaces/srv/set_parameters.hpp>

#include <eigen3/Eigen/Geometry>
#include <chrono>
#include <memory>
#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

/*
    demo node for cooperative robots taks execution
    - use_internal_force_control: bool, use force control or not
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
auto call_service(const std::shared_ptr<rclcpp::Client<ServiceT>>& client,
                  typename ServiceT::Request::SharedPtr request, typename ServiceT::Response::SharedPtr response,
                  bool wait = true)

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

  if (!wait)
  {
    return result;
  }

  // Wait for the result using the executor if provided.
  auto start_time = std::chrono::steady_clock::now();
  auto timeout = std::chrono::seconds(10);  // Set a timeout duration

  while (rclcpp::ok())
  {
    if (result.wait_for(std::chrono::seconds(1)) == std::future_status::ready)
    {
      auto response = result.get();
      if (response)
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

    if (std::chrono::steady_clock::now() - start_time > timeout)
    {
      std::cerr << "Service call timed out" << std::endl;
      throw std::runtime_error("Service call timed out");
    }
  }
  throw std::runtime_error("Service call interrupted");
}

bool stringToBool(const std::string& str)
{
  return str == "true" || str == "1";
}

void set_activate_status(const std::shared_ptr<rclcpp::Client<std_srvs::srv::SetBool>>& client, bool activate,
                         bool wait = true)
{
  std::shared_ptr<std_srvs::srv::SetBool::Request> request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = activate;

  call_service(client, request, std_srvs::srv::SetBool::Response::SharedPtr(), wait);
}

int main(int argc, char** argv)
{
  if (argc < 6)
  {
    RCLCPP_ERROR(rclcpp::get_logger("cooperative_robots_demo"),
                 "Usage: demo_node <use_internal_force_control> <use_object_pose_control> <use_ekf> "
                 "<use_estimated_b1Tb2> <use_pivoting>");
    return 1;
  }

  bool use_internal_force_control = stringToBool(argv[1]);
  bool use_object_pose_control = stringToBool(argv[2]);
  bool use_ekf = stringToBool(argv[3]);
  bool use_estimated_b1Tb2 = stringToBool(argv[4]);
  bool use_pivoting = stringToBool(argv[5]);

  std::cout << "use_internal_force_control: " << use_internal_force_control << std::endl;
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
  const std::string fkine_prepivoting_robot1_topic = "/robot1/fkine_pre_pivoting_joint";
  const std::string fkine_prepivoting_robot2_topic = "/robot2/fkine_pre_pivoting_joint";
  const std::string joint_state_topic_robot1 = "/robot1/joint_states";
  const std::string joint_state_topic_robot2 = "/robot2/joint_states";
  const std::string obj_pose_topic = "/ekf/object_pose";
  const std::string filtered_b1Tb2_topic = "/ekf/b1Tb2_filtered";
  const std::string package_share_directory = ament_index_cpp::get_package_share_directory("dual_arm_control");
  const std::string obj_yaml_path = package_share_directory + "/config/config.yaml";
  const std::string task_yaml_path = package_share_directory + "/config/task.yaml";

  const std::string slipping_client_bridge_homing_srv_robot1_name = "/iiwa/wsg50/slipping_client_bridge_homing_srv";
  const std::string slipping_client_bridge_grasping_srv_robot1_name = "/iiwa/wsg50/slipping_client_bridge_grasping_srv";
  const std::string slipping_client_bridge_pivoting_srv_robot1_name = "/iiwa/wsg50/slipping_client_bridge_pivoting_srv";

  const std::string slipping_client_bridge_homing_srv_robot2_name = "/yaskawa/wsg32/slipping_client_bridge_homing_srv";
  const std::string slipping_client_bridge_grasping_srv_robot2_name =
      "/yaskawa/wsg32/slipping_client_bridge_grasping_srv";
  const std::string slipping_client_bridge_pivoting_srv_robot2_name =
      "/yaskawa/wsg32/slipping_client_bridge_pivoting_srv";

  double duration_home_robots = 7.0;
  double duration_pregrasp = 10.0;
  double duration_grasp = 10.0;
  double duration_cooperative_segments = 10.0;

  Eigen::Vector3d pregrasp_offset_single_robot(0.0, -0.1, -0.0);  // offset from object pose for pregrasp pose
  Eigen::Vector3d offset_from_initial_pose(0.0, 0.0, 0.1);  // (base frame) offset from initial pose for intermediate
                                                            // poses in the cartesian trajectory
  Eigen::Vector3d offset_from_final_pose(0.0, 0.0, 0.1);  // (base frame) offset from final pose for intermediate poses
                                                          // in the cartesian trajectory
  Eigen::Vector3d postgrasp_offset(0.0, -0.1, -0.0);  // (base frame) offset for robots movement after cooperative task

  //---------------------------------------------

  // ROS objects --------------------------------
  auto ekf_client = node->create_client<dual_arm_control_interfaces::srv::EKFService>("ekf_service");
  auto ekf_client_object_grasped = node->create_client<std_srvs::srv::SetBool>("/ekf_set_object_grasped");
  auto aurco_pose_conversion_client_camera1 =
      node->create_client<uclv_aruco_detection_interfaces::srv::PoseService>("/robot1/pose_conversion_service");
  auto aurco_pose_conversion_client_camera2 =
      node->create_client<uclv_aruco_detection_interfaces::srv::PoseService>("/robot2/pose_conversion_service");
  auto activate_joint_integrator_client_robot1 =
      node->create_client<std_srvs::srv::SetBool>("/robot1/joint_integrator/set_running");
  auto activate_joint_integrator_client_robot2 =
      node->create_client<std_srvs::srv::SetBool>("/robot2/joint_integrator/set_running");
  auto actvate_internal_force_control_client = node->create_client<std_srvs::srv::SetBool>("/activate_force_control");
  auto activate_object_pose_control_client = node->create_client<std_srvs::srv::SetBool>("/activate_object_control");
  auto set_end_effector_camera_client_robot1 =
      node->create_client<uclv_robot_ros_msgs::srv::SetEndEffector>("/robot1/camera/set_end_effector");
  auto set_end_effector_camera_client_robot2 =
      node->create_client<uclv_robot_ros_msgs::srv::SetEndEffector>("/robot2/camera/set_end_effector");
  auto parameters_client_cooperative_space_node =
      node->create_client<rcl_interfaces::srv::SetParameters>("/cooperative_robots_server/set_parameters");
  auto set_end_effector_client_robot1 =
      node->create_client<uclv_robot_ros_msgs::srv::SetEndEffector>("/robot1/set_end_effector");
  auto set_end_effector_client_robot2 =
      node->create_client<uclv_robot_ros_msgs::srv::SetEndEffector>("/robot2/set_end_effector");
  auto set_end_effector_client_robot1_tactile =
      node->create_client<uclv_robot_ros_msgs::srv::SetEndEffector>("/robot1/tactile_sensor/set_end_effector");
  auto set_end_effector_client_robot2_tactile =
      node->create_client<uclv_robot_ros_msgs::srv::SetEndEffector>("/robot2/tactile_sensor/set_end_effector");

  auto slipping_client_bridge_homing_srv_robot1 =
      node->create_client<std_srvs::srv::SetBool>(slipping_client_bridge_homing_srv_robot1_name);
  auto slipping_client_bridge_grasping_srv_robot1 =
      node->create_client<std_srvs::srv::SetBool>(slipping_client_bridge_grasping_srv_robot1_name);
  auto slipping_client_bridge_pivoting_srv_robot1 =
      node->create_client<std_srvs::srv::SetBool>(slipping_client_bridge_pivoting_srv_robot1_name);

  auto slipping_client_bridge_homing_srv_robot2 =
      node->create_client<std_srvs::srv::SetBool>(slipping_client_bridge_homing_srv_robot2_name);
  auto slipping_client_bridge_grasping_srv_robot2 =
      node->create_client<std_srvs::srv::SetBool>(slipping_client_bridge_grasping_srv_robot2_name);
  auto slipping_client_bridge_pivoting_srv_robot2 =
      node->create_client<std_srvs::srv::SetBool>(slipping_client_bridge_pivoting_srv_robot2_name);

  // Objects to store ----------------------------
  uclv::ros::JointTrajectoryClient joint_client_robot1(node, "/robot1/generate_joint_trajectory");
  uclv::ros::JointTrajectoryClient joint_client_robot2(node, "/robot2/generate_joint_trajectory");
  uclv::ros::CartesianTrajectoryClient cartesian_client_robot1(node, "/robot1/generate_cartesian_trajectory");
  uclv::ros::CartesianTrajectoryClient cartesian_client_robot2(node, "/robot2/generate_cartesian_trajectory");
  uclv::ros::CartesianTrajectoryClient cooperative_cartesian_client(node, "/generate_cartesian_trajectory");
  uclv::ros::CartesianTrajectoryClient cooperative_cartesian_client_direct(node,
                                                                           "/cooperative_utils/"
                                                                           "generate_cartesian_trajectory");

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

  // read camera wrt last link
  Eigen::Matrix<double, 4, 4> prepivot1Tcamera;
  prepivot1Tcamera.setIdentity();
  read_transform(task_yaml["prepivot1Tcamera"], prepivot1Tcamera);
  std::cout << "prepivot1Tcamera: \n" << prepivot1Tcamera << std::endl;

  // read camera wrt last link
  Eigen::Matrix<double, 4, 4> prepivot2Tcamera;
  prepivot2Tcamera.setIdentity();
  read_transform(task_yaml["prepivot2Tcamera"], prepivot2Tcamera);
  std::cout << "prepivot2Tcamera: \n" << prepivot2Tcamera << std::endl;

  // set end effector camera 1
  uclv_robot_ros_msgs::srv::SetEndEffector::Request::SharedPtr request_end_effector =
      std::make_shared<uclv_robot_ros_msgs::srv::SetEndEffector::Request>();
  eigen_matrix_to_pose_msg(prepivot1Tcamera, request_end_effector->flange_pose_ee);
  call_service(set_end_effector_camera_client_robot1, request_end_effector,
               uclv_robot_ros_msgs::srv::SetEndEffector::Response::SharedPtr());

  // set end effector camera 2
  eigen_matrix_to_pose_msg(prepivot2Tcamera, request_end_effector->flange_pose_ee);
  call_service(set_end_effector_camera_client_robot2, request_end_effector,
               uclv_robot_ros_msgs::srv::SetEndEffector::Response::SharedPtr());

  // read end effector robot1
  Eigen::Matrix<double, 4, 4> pivotinglink1Tee1;
  pivotinglink1Tee1.setIdentity();
  read_transform(task_yaml["pivotinglink1Tee1"], pivotinglink1Tee1);
  std::cout << "pivotinglink1Tee1: \n" << pivotinglink1Tee1 << std::endl;

  // read end effector robot2
  Eigen::Matrix<double, 4, 4> pivotinglink2Tee2;
  pivotinglink2Tee2.setIdentity();
  read_transform(task_yaml["pivotinglink2Tee2"], pivotinglink2Tee2);
  std::cout << "pivotinglink2Tee2: \n" << pivotinglink2Tee2 << std::endl;

  // set end effector robot1
  eigen_matrix_to_pose_msg(pivotinglink1Tee1, request_end_effector->flange_pose_ee);
  call_service(set_end_effector_client_robot1, request_end_effector,
               uclv_robot_ros_msgs::srv::SetEndEffector::Response::SharedPtr());

  // set end effector camera 2
  eigen_matrix_to_pose_msg(pivotinglink2Tee2, request_end_effector->flange_pose_ee);
  call_service(set_end_effector_client_robot2, request_end_effector,
               uclv_robot_ros_msgs::srv::SetEndEffector::Response::SharedPtr());

  // set tactile sensors end effector
  Eigen::Matrix<double, 4, 4> prepivot1Ttactile;
  prepivot1Ttactile.setIdentity();
  read_transform(task_yaml["prepivot1Ttactile"], prepivot1Ttactile);
  std::cout << "prepivot1Ttactile: \n" << prepivot1Ttactile << std::endl;

  // read tactile sensor wrt the pre-pivoting joint
  Eigen::Matrix<double, 4, 4> prepivot2Ttactile;
  prepivot2Ttactile.setIdentity();
  read_transform(task_yaml["prepivot2Ttactile"], prepivot2Ttactile);
  std::cout << "prepivot2Ttactile: \n" << prepivot2Ttactile << std::endl;

  // set end effector robot1
  eigen_matrix_to_pose_msg(prepivot1Ttactile, request_end_effector->flange_pose_ee);
  call_service(set_end_effector_client_robot2_tactile, request_end_effector,
               uclv_robot_ros_msgs::srv::SetEndEffector::Response::SharedPtr());

  // set end effector camera 2
  eigen_matrix_to_pose_msg(prepivot2Ttactile, request_end_effector->flange_pose_ee);
  call_service(set_end_effector_client_robot1_tactile, request_end_effector,
               uclv_robot_ros_msgs::srv::SetEndEffector::Response::SharedPtr());

  // home grippers
  if (use_pivoting)
  {
    std::cout << "Slipping control homing robot 1" << std::endl;
    set_activate_status(slipping_client_bridge_homing_srv_robot1, true, false);
    wait_for_enter();
    std::cout << "Slipping control homing robot 2" << std::endl;
    set_activate_status(slipping_client_bridge_homing_srv_robot2, true, false);
  }

  // ############################### START COOPERATIVE TASK ################################################

  // 1. MOVE ROBOTS TO INITIAL JOINT POSITION (HOME)
  RCLCPP_INFO(node->get_logger(), "Executing go_to robot 1 home");
  wait_for_enter();
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  // the first call to goTo may fail, so we need to retry
  bool success = false;
  while (!success)
  {
    if (!rclcpp::ok())
    {
      return 1;
    }
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
  wait_for_enter();

  joint_client_robot2.goTo(joint_state_topic_robot2, q_robot2_home,
                           rclcpp::Duration::from_seconds(duration_home_robots), rclcpp::Time(0), true);

  // iterate over the objects in the task list
  int object_index = -1;
  for (const auto& obj_name : objects_task_list)
  {
    std::cout << "PROCESSING Object: " << obj_name << std::endl;
    object_index++;

    // 2. CALL EKF SERVICE TO GET OBJECT POSE
    if (use_ekf)
    {
      // call aruco conversion services
      auto request_aruco = std::make_shared<uclv_aruco_detection_interfaces::srv::PoseService::Request>();
      request_aruco->object_name.data = obj_name;
      request_aruco->yaml_file_path.data = obj_yaml_path;
      std::cout << "Calling aruco conversion service camera 1" << std::endl;
      call_service(aurco_pose_conversion_client_camera1, request_aruco,
                   uclv_aruco_detection_interfaces::srv::PoseService::Response::SharedPtr());
      std::cout << "Calling aruco conversion service camera 2" << std::endl;
      call_service(aurco_pose_conversion_client_camera2, request_aruco,
                   uclv_aruco_detection_interfaces::srv::PoseService::Response::SharedPtr());

      // call EKF service
      auto request = std::make_shared<dual_arm_control_interfaces::srv::EKFService::Request>();
      request->object_name.data = obj_name;
      request->yaml_file_path.data = obj_yaml_path;
      fill_pose(request->object_pose.pose, 0.8, 0.0, 0.5, 0.0, 0.0, 0.7, -0.7);
      fill_twist(request->object_twist.twist, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      fill_pose(request->fkine_robot1.pose, 0.77, 0.06, 0.44, 0.69, -0.69,-0.21, -0.21);
      fill_pose(request->fkine_robot2.pose, 0.009, 0.52, 0.43, 0.65, -0.27, -0.64, 0.27);
      fill_pose(request->robots_relative_transform.pose, 0.0, 0*1.63, 0.0, 0.707, 0.0, 0.0, -0.707);

      std::cout << "Calling EKF service" << std::endl;
      call_service(ekf_client, request, dual_arm_control_interfaces::srv::EKFService::Response::SharedPtr());
    }

    // wait some time to get the object pose
    // rclcpp::sleep_for(std::chrono::seconds(2));

    // wait for ENTER to continue
    wait_for_enter();

    // if use_estimated_b1Tb2 is true, then read the estimated b1Tb2 from the topic
    if (use_estimated_b1Tb2 && use_ekf)
    {
      // read estimated b1Tb2 from the topic
      auto estimated_b1Tb2_ptr = uclv::ros::waitForMessage<geometry_msgs::msg::PoseStamped>(filtered_b1Tb2_topic, node);
      auto estimated_b1Tb2 = std::const_pointer_cast<geometry_msgs::msg::PoseStamped>(estimated_b1Tb2_ptr);

      pose_msg_to_eigen_matrix(estimated_b1Tb2->pose, b1Tb2);
      std::cout << "Estimated b1Tb2: \n" << b1Tb2 << std::endl;

      // set the b1Tb2 transform of the cooperative space node
      rcl_interfaces::srv::SetParameters::Request::SharedPtr request_cooperative_space_node =
          std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

      std::vector<double> b1Tb2_vector;
      b1Tb2_vector.push_back(estimated_b1Tb2->pose.position.x);
      b1Tb2_vector.push_back(estimated_b1Tb2->pose.position.y);
      b1Tb2_vector.push_back(estimated_b1Tb2->pose.position.z);
      b1Tb2_vector.push_back(estimated_b1Tb2->pose.orientation.w);
      b1Tb2_vector.push_back(estimated_b1Tb2->pose.orientation.x);
      b1Tb2_vector.push_back(estimated_b1Tb2->pose.orientation.y);
      b1Tb2_vector.push_back(estimated_b1Tb2->pose.orientation.z);
      request_cooperative_space_node->parameters.push_back(rclcpp::Parameter("b1Tb2", b1Tb2_vector).to_parameter_msg());
      call_service(parameters_client_cooperative_space_node, request_cooperative_space_node,
                   rcl_interfaces::srv::SetParameters::Response::SharedPtr());
    }

    // read object pose from the topic
    auto object_pose_ptr = uclv::ros::waitForMessage<geometry_msgs::msg::PoseStamped>(obj_pose_topic, node);
    auto object_pose = std::const_pointer_cast<geometry_msgs::msg::PoseStamped>(object_pose_ptr);

    Eigen::Matrix<double, 4, 4> bTo = Eigen::Matrix<double, 4, 4>::Identity();
    pose_msg_to_eigen_matrix(object_pose->pose, bTo);
    std::cout << "Object pose: \n" << bTo << std::endl;

    // 3. MOVE ROBOTS TO GRASP POSES

    // read desired robot configurations for the secondary task
    std::vector<double> qdesired_robot1 =
        task_yaml["objects"][object_index]["object"]["qdes_robot1"].as<std::vector<double>>();
    std::vector<double> qdesired_robot2 =
        task_yaml["objects"][object_index]["object"]["qdes_robot2"].as<std::vector<double>>();

    std::cout << "\n desired robot1 joint configuration: ";
    print_joint_positions(qdesired_robot1);

    std::cout << "\n desired robot2 joint configuration: ";
    print_joint_positions(qdesired_robot2);

    // set the parameter for the cooperative robots server
    rcl_interfaces::srv::SetParameters::Request::SharedPtr request_cooperative_space_node =
        std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

    request_cooperative_space_node->parameters.push_back(
        rclcpp::Parameter("q1_desired", qdesired_robot1).to_parameter_msg());
    request_cooperative_space_node->parameters.push_back(
        rclcpp::Parameter("q2_desired", qdesired_robot2).to_parameter_msg());
    call_service(parameters_client_cooperative_space_node, request_cooperative_space_node,
                 rcl_interfaces::srv::SetParameters::Response::SharedPtr());

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
    set_activate_status(activate_joint_integrator_client_robot1, true);
    set_activate_status(activate_joint_integrator_client_robot2, true);

    // move robot 1 to pregrasp pose
    std::cout << "Moving robot 1 to pregrasp pose" << std::endl;
    wait_for_enter();
    geometry_msgs::msg::Pose target_pose;
    eigen_matrix_to_pose_msg(b1Tg1_pregrasp, target_pose);
    cartesian_client_robot1.goTo(fkine_robot1_topic, target_pose, rclcpp::Duration::from_seconds(duration_pregrasp));

    // move robot 1 to grasp pose
    std::cout << "Moving robot 1 to grasp pose" << std::endl;
    wait_for_enter();
    eigen_matrix_to_pose_msg(b1Tg1, target_pose);
    cartesian_client_robot1.goTo(fkine_robot1_topic, target_pose, rclcpp::Duration::from_seconds(duration_grasp));

    // move robot 2 to pregrasp pose
    std::cout << "Moving robot 2 to pregrasp pose" << std::endl;
    wait_for_enter();
    eigen_matrix_to_pose_msg(b2Tg2_pregrasp, target_pose);
    cartesian_client_robot2.goTo(fkine_robot2_topic, target_pose, rclcpp::Duration::from_seconds(duration_pregrasp));

    // move robot 2 to grasp pose
    std::cout << "Moving robot 2 to grasp pose" << std::endl;
    wait_for_enter();
    eigen_matrix_to_pose_msg(b2Tg2, target_pose);
    cartesian_client_robot2.goTo(fkine_robot2_topic, target_pose, rclcpp::Duration::from_seconds(duration_grasp));

    // deactivate joints integrators to avoid undesired robots movements
    set_activate_status(activate_joint_integrator_client_robot1, false);
    set_activate_status(activate_joint_integrator_client_robot2, false);

    // 4. COOPERATIVE MANIPULATION

    // grasp object and start pivoting
    if (use_pivoting)
    {
      std::cout << "PRESS ENTER TO CLOSE GRIPPERS" << std::endl;
      wait_for_enter();

      set_activate_status(slipping_client_bridge_grasping_srv_robot1, true, false);
      set_activate_status(slipping_client_bridge_grasping_srv_robot2, true, false);

      set_activate_status(slipping_client_bridge_pivoting_srv_robot1, true, false);
      set_activate_status(slipping_client_bridge_pivoting_srv_robot2, true, false);

      std::cout << "Pivoting mode active" << std::endl;
    }

    // first cooperative movement: bring up the object without controls
    // generate first trajectory
    Eigen::Matrix<double, 4, 4> bTo_up;
    bTo_up = bTo;
    bTo_up.block<3, 1>(0, 3) = bTo_up.block<3, 1>(0, 3) + offset_from_initial_pose / 4.0;

    auto goal_msg = uclv_robot_ros_msgs::action::CartesianTrajectory::Goal();
    goal_msg.trajectory.header.stamp = rclcpp::Time(0);
    goal_msg.trajectory.points.resize(2);
    goal_msg.trajectory.points[0].time_from_start = rclcpp::Duration(0, 0);
    eigen_matrix_to_pose_msg(bTo, goal_msg.trajectory.points[0].pose);

    goal_msg.trajectory.points[1].time_from_start = rclcpp::Duration::from_seconds(duration_cooperative_segments * 2);
    eigen_matrix_to_pose_msg(bTo_up, goal_msg.trajectory.points[1].pose);

    // activate joint integrators
    set_activate_status(activate_joint_integrator_client_robot1, true);
    set_activate_status(activate_joint_integrator_client_robot2, true);

    // execute cooperative cartesian trajectory
    std::cout << "PRESS ENTER TO START COOPERATIVE MOVEMENT" << std::endl;
    wait_for_enter();

    cooperative_cartesian_client_direct.goTo(goal_msg);  // the twist is published directly to the robot

    // deactivate and activate integrators to avoid undesired robots movements
    std::cout << "JOINT INTEGRATORS DEACTIVATION" << std::endl;
    set_activate_status(activate_joint_integrator_client_robot1, false);
    set_activate_status(activate_joint_integrator_client_robot2, false);

    // advertise ekf set object grasped
    if (use_ekf)
    { 
      std::cout << "PRESS ENTER TO SET OBJECT GRASPED" << std::endl;   
      wait_for_enter();
      set_activate_status(ekf_client_object_grasped, true, true);
    }

    std::cout << "ACTIVATE joint integrators before force control and pose control" << std::endl;
    wait_for_enter();

    set_activate_status(activate_joint_integrator_client_robot1, true);
    set_activate_status(activate_joint_integrator_client_robot2, true);

    // activate internal force control
    if (use_internal_force_control)
    {
      // activate internal force control
      std::cout << "PRESS ENTER TO ACTIVATE FORCE CONTROL" << std::endl;
      wait_for_enter();
      set_activate_status(actvate_internal_force_control_client, true);
    }

    // activate object pose control
    if (use_object_pose_control)
    {
      // activate object pose control
      std::cout << "PRESS ENTER TO ACTIVATE OBJECT POSE CONTROL" << std::endl;
      wait_for_enter();
      set_activate_status(activate_object_pose_control_client, true);
    }

    // compute cooperative trajectory

    // read again object pose from the topic
    object_pose_ptr = uclv::ros::waitForMessage<geometry_msgs::msg::PoseStamped>(obj_pose_topic, node);
    object_pose = std::const_pointer_cast<geometry_msgs::msg::PoseStamped>(object_pose_ptr);

    bTo = Eigen::Matrix<double, 4, 4>::Identity();
    pose_msg_to_eigen_matrix(object_pose->pose, bTo);
    std::cout << "Object pose: \n" << bTo << std::endl;

    // read final_pose bTo_final
    Eigen::Matrix<double, 4, 4> bTo_final;
    bTo_final.setIdentity();
    read_transform(task_yaml["objects"][object_index]["object"]["bTo_final"], bTo_final);

    // compute intermediate poses for cartesian trajectory
    Eigen::Matrix<double, 4, 4> bTo_second = bTo;
    bTo_second.block<3, 1>(0, 3) = bTo_second.block<3, 1>(0, 3) + offset_from_initial_pose;
    Eigen::Matrix<double, 4, 4> bTo_third = bTo_final;
    bTo_third.block<3, 1>(0, 3) = bTo_third.block<3, 1>(0, 3) + offset_from_final_pose;

    // print bTo, bTo_second, bTo_third, bTo_final
    std::cout << "bTo: \n" << bTo << std::endl;
    std::cout << "bTo_second: \n" << bTo_second << std::endl;
    std::cout << "bTo_third: \n" << bTo_third << std::endl;
    std::cout << "bTo_final: \n" << bTo_final << std::endl;

    // fill Goal message for cooperative cartesian trajectory
    goal_msg = uclv_robot_ros_msgs::action::CartesianTrajectory::Goal();
    goal_msg.trajectory.header.stamp = rclcpp::Time(0);
    goal_msg.trajectory.points.resize(5);
    goal_msg.trajectory.points[0].time_from_start = rclcpp::Duration(0, 0);
    eigen_matrix_to_pose_msg(bTo, goal_msg.trajectory.points[0].pose);

    goal_msg.trajectory.points[1].time_from_start = rclcpp::Duration::from_seconds(duration_cooperative_segments);
    eigen_matrix_to_pose_msg(bTo_second, goal_msg.trajectory.points[1].pose);

    goal_msg.trajectory.points[2].time_from_start = rclcpp::Duration::from_seconds(2 * duration_cooperative_segments);
    eigen_matrix_to_pose_msg(bTo_third, goal_msg.trajectory.points[2].pose);

    goal_msg.trajectory.points[3].time_from_start = rclcpp::Duration::from_seconds(3 * duration_cooperative_segments);
    eigen_matrix_to_pose_msg(bTo_final, goal_msg.trajectory.points[3].pose);

    goal_msg.trajectory.points[4].time_from_start =
        rclcpp::Duration::from_seconds(4 * duration_cooperative_segments);  // hold the position for a while
    eigen_matrix_to_pose_msg(bTo_final, goal_msg.trajectory.points[4].pose);

    std::cout << "PRESS ENTER TO START COOPERATIVE MOVEMENT" << std::endl;
    wait_for_enter();

    // execute cooperative cartesian trajectory
    if (use_object_pose_control)
      cooperative_cartesian_client.goTo(goal_msg);
    else
      cooperative_cartesian_client_direct.goTo(goal_msg);  // the twist is published directly to the robot

    // deactivate and activate integrators to avoid undesired robots movements
    std::cout << "JOINT INTEGRATORS DEACTIVATION" << std::endl;
    set_activate_status(activate_joint_integrator_client_robot1, false);
    set_activate_status(activate_joint_integrator_client_robot2, false);

    // deactivate internal force control
    if (use_internal_force_control)
    {
      std::cout << "FORCE CONTROL DEACTIVATION" << std::endl;
      set_activate_status(actvate_internal_force_control_client, false);
    }

    // deactivate object pose control
    if (use_object_pose_control)
    {
      std::cout << "OBJECT POSE CONTROL DEACTIVATION" << std::endl;
      set_activate_status(activate_object_pose_control_client, false);
    }

    // home grippers
    if (use_pivoting)
    {
      std::cout << "homing grippers" << std::endl;
      wait_for_enter();
      set_activate_status(slipping_client_bridge_homing_srv_robot1, true, false);
      set_activate_status(slipping_client_bridge_homing_srv_robot2, true, false);

      if (use_ekf)
        set_activate_status(ekf_client_object_grasped, false, true);
    }

    // 5. MOVE ROBOTS TO HOME POSITION
    std::cout << "PRESS ENTER TO MOVE ROBOTS TO HOME POSITION" << std::endl;
    wait_for_enter();
    set_activate_status(activate_joint_integrator_client_robot1, true);
    set_activate_status(activate_joint_integrator_client_robot2, true);

    // movement post-grasp robot 1
    Eigen::Matrix<double, 4, 4> fkine1_T_fkine1postgrasp;
    fkine1_T_fkine1postgrasp.setIdentity();
    fkine1_T_fkine1postgrasp.block<3, 1>(0, 3) = postgrasp_offset;
    eigen_matrix_to_pose_msg(fkine1_T_fkine1postgrasp, target_pose);

    cartesian_client_robot1.goTo_EE(fkine_prepivoting_robot1_topic, target_pose,
                                    rclcpp::Duration::from_seconds(duration_grasp));
    wait_for_enter();

    // movement post-grasp robot 2
    Eigen::Matrix<double, 4, 4> fkine2_T_fkine2postgrasp;
    fkine2_T_fkine2postgrasp.setIdentity();
    fkine2_T_fkine2postgrasp.block<3, 1>(0, 3) = postgrasp_offset;
    eigen_matrix_to_pose_msg(fkine2_T_fkine2postgrasp, target_pose);

    cartesian_client_robot2.goTo_EE(fkine_prepivoting_robot2_topic, target_pose,
                                    rclcpp::Duration::from_seconds(duration_grasp));
    wait_for_enter();

    // deactivate joints integrators
    set_activate_status(activate_joint_integrator_client_robot1, false);
    set_activate_status(activate_joint_integrator_client_robot2, false);

    // move the robots to home position
    joint_client_robot1.goTo(joint_state_topic_robot1, q_robot1_home,
                             rclcpp::Duration::from_seconds(duration_home_robots), rclcpp::Time(0), true);
    wait_for_enter();
    joint_client_robot2.goTo(joint_state_topic_robot2, q_robot2_home,
                             rclcpp::Duration::from_seconds(duration_home_robots), rclcpp::Time(0), true);
    wait_for_enter();
  }

  rclcpp::shutdown();
  return 0;
}
