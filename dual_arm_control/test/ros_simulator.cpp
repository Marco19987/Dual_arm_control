
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "dual_arm_control_interfaces/srv/ekf_service.hpp"

#include "../include/robots_object_system.hpp"
#include "../include/robots_object_system_ext.hpp" // see this file to understande the system
#include "../include/geometry_helper.hpp"
#include <uclv_systems_lib/observers/ekf.hpp>
#include <uclv_systems_lib/discretization/forward_euler.hpp>

#include <eigen3/Eigen/Geometry>
#include <chrono>
#include <memory>
#include <yaml-cpp/yaml.h>

class SimulatorRobotsObject : public rclcpp::Node
{
public:
  SimulatorRobotsObject() : Node("simulator_robots_object")
  {
    // declare parameters
    this->declare_parameter<double>("sample_time", 0.05);
    this->get_parameter("sample_time", this->sample_time_);

    this->declare_parameter<std::string>("robot_1_prefix", "robot_1");
    this->get_parameter("robot_1_prefix", this->robot_1_prefix_);

    this->declare_parameter<std::string>("robot_2_prefix", "robot_2");
    this->get_parameter("robot_2_prefix", this->robot_2_prefix_);

    // initialize wrench publishers
    wrench_robot1_pub_ =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>("/" + this->robot_1_prefix_ + "/wrench", 1);
    wrench_robot2_pub_ =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>("/" + this->robot_2_prefix_ + "/wrench", 1);

    // initialize pose publisher
    object_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/object_pose", 1);
    object_twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/object_twist", 1);

    // define yaml file path
    std::string yaml_file_path = "/home/mdesimone/dual_arm_ws/src/dual_arm_control/dual_arm_control/config/config.yaml";
    //std::string yaml_file_path = "/home/marco/dual_arm_ws/src/dual_arm_control/dual_arm_control/config/config.yaml";
    //std::string yaml_file_path = "/home/mdesimone/cooperative_robots_ws/src/dual_arm_control/dual_arm_control/config/config.yaml";

    std::string object_name = "resin_block_1";

    // control input and initial state
    x0_.resize(20, 1);
    x0_ << -0.037, 0.0135, 0.196, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    u_.resize(12, 1);
    u_ << 0.0, 0.0, 0.001, 0, 0, 0.0, -0.0, -0.0, 0.001, 0, 0, 0;

    // read the yaml file
    read_yaml_file(yaml_file_path, object_name);

    // create the timer
    timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(sample_time_ * 1000)),
                                     std::bind(&SimulatorRobotsObject::timer_callback, this));
  }

private:
  void timer_callback()
  {
    x0_ = discretized_system_ptr_->get_state();
    discretized_system_ptr_->step(u_);

    // ensure the quaternion continuity
    Eigen::Matrix<double, 20, 1> x = discretized_system_ptr_->get_state();
    Eigen::Matrix<double, 4, 1> q;

    uclv::geometry_helper::quaternion_continuity(x.block<4, 1>(3, 0), x0_.block<4, 1>(3, 0), q);
    Eigen::Quaterniond q_(q(0), q(1), q(2), q(3));
    q_.normalize();
    x.block<4, 1>(3, 0) << q_.w(), q_.vec();

    // std::cout << "x: \n"
    //           << x.transpose() << std::endl;
    discretized_system_ptr_->set_state(x);

    y_ = discretized_system_ptr_->get_output();

    // publish the measures
    for (int i = 0; i < 2 * num_frames_; i++)
    {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = this->now();
      pose_msg.pose.position.x = y_(i * 7);
      pose_msg.pose.position.y = y_(i * 7 + 1);
      pose_msg.pose.position.z = y_(i * 7 + 2);
      pose_msg.pose.orientation.w = y_(i * 7 + 3);
      pose_msg.pose.orientation.x = y_(i * 7 + 4);
      pose_msg.pose.orientation.y = y_(i * 7 + 5);
      pose_msg.pose.orientation.z = y_(i * 7 + 6);
      pose_publishers_[i]->publish(pose_msg);
    }

    // publish wrench
    for (int i = 0; i < 2; i++)
    {
      geometry_msgs::msg::WrenchStamped wrench_msg;
      wrench_msg.header.stamp = this->now();
      wrench_msg.wrench.force.x = u_(i * 6);
      wrench_msg.wrench.force.y = u_(i * 6 + 1);
      wrench_msg.wrench.force.z = u_(i * 6 + 2);
      wrench_msg.wrench.torque.x = u_(i * 6 + 3);
      wrench_msg.wrench.torque.y = u_(i * 6 + 4);
      wrench_msg.wrench.torque.z = u_(i * 6 + 5);
      if (i == 0)
      {
        wrench_robot1_pub_->publish(wrench_msg);
      }
      else
      {
        wrench_robot2_pub_->publish(wrench_msg);
      }

      // publish pose
      geometry_msgs::msg::PoseStamped object_pose_msg;
      object_pose_msg.header.stamp = this->now();
      object_pose_msg.pose.position.x = x(0);
      object_pose_msg.pose.position.y = x(1);
      object_pose_msg.pose.position.z = x(2);
      object_pose_msg.pose.orientation.w = x(3);
      object_pose_msg.pose.orientation.x = x(4);
      object_pose_msg.pose.orientation.y = x(5);
      object_pose_msg.pose.orientation.z = x(6);
      object_pose_publisher_->publish(object_pose_msg);

      // publish twist
      geometry_msgs::msg::TwistStamped object_twist_msg;
      object_twist_msg.header.stamp = this->now();
      object_twist_msg.twist.linear.x = x(7);
      object_twist_msg.twist.linear.y = x(8);
      object_twist_msg.twist.linear.z = x(9);
      object_twist_msg.twist.angular.x = x(10);
      object_twist_msg.twist.angular.y = x(11);
      object_twist_msg.twist.angular.z = x(12);
      object_twist_publisher_->publish(object_twist_msg);
    }
  }

  void read_yaml_file(const std::string &yaml_file_path, const std::string &object_name)
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
    std::cout << "Gravity Vector: \n"
              << bg << std::endl;

    // read b1Tb2
    Eigen::Matrix<double, 4, 4> b1Tb2;
    b1Tb2.setIdentity();
    read_transform(config["b1Tb2"], b1Tb2);
    std::cout << "b1Tb2: \n"
              << b1Tb2 << std::endl;

    // read bTb1
    Eigen::Matrix<double, 4, 4> bTb1;
    bTb1.setIdentity();
    read_transform(config["bTb1"], bTb1);
    std::cout << "bTb1: \n"
              << bTb1 << std::endl;

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
    std::cout << "oTg1: \n"
              << oTg1 << std::endl;

    // read oTg2
    Eigen::Matrix<double, 4, 4> oTg2;
    oTg2.setIdentity();
    read_transform(object_node["oTg2"], oTg2);
    std::cout << "oTg2: \n"
              << oTg2 << std::endl;

    // read names of frames published
    std::vector<std::string> frame_names;
    for (const auto &transformation : object_node["aruco_transforms"])
    {
      frame_names.push_back(transformation["name"].as<std::string>());
      RCLCPP_INFO(this->get_logger(), "Frame: %s", transformation["name"].as<std::string>().c_str());
    }

    // instantiate the subscribers to the pose topics
    int num_frames = frame_names.size();
    for (const auto &frame_name : frame_names)
    {
      pose_publishers_.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>(
          this->robot_1_prefix_ + "/" + object_name + "/" + frame_name + "/pose", 1));
    }

    for (const auto &frame_name : frame_names)
    {
      pose_publishers_.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>(
          this->robot_2_prefix_ + "/" + object_name + "/" + frame_name + "/pose", 1));
    }

    num_frames_ = num_frames;

    // create the system
    Eigen::Matrix<double, 13, 1> x0_system; x0_system.setZero();
    robots_object_system_ptr_ = std::make_shared<uclv::systems::RobotsObjectSystem>(
        x0_system, Bm, bg, oTg1, oTg2, b1Tb2, bTb1, viscous_friction_matrix, num_frames_);

    // create the extended system
    robots_object_system_ext_ptr_ =
        std::make_shared<uclv::systems::RobotsObjectSystemExt>(x0_, robots_object_system_ptr_);

    // discretize the system
    discretized_system_ptr_ = std::make_shared<uclv::systems::ForwardEuler<20, 12, Eigen::Dynamic>>(
        robots_object_system_ext_ptr_, sample_time_, x0_);

    discretized_system_ptr_->display();

    // resize the output variable
    y_.resize(num_frames_ * 14, 1);
  }

  void read_inertia_matrix(const YAML::Node &object, Eigen::Matrix<double, 6, 6> &Bm)
  {
    if (object["inertia_matrix"])
    {
      std::vector<double> inertia_matrix = object["inertia_matrix"].as<std::vector<double>>();
      if (inertia_matrix.size() == 6)
      {
        Bm.setZero();
        Bm.diagonal() << inertia_matrix[0], inertia_matrix[1], inertia_matrix[2], inertia_matrix[3], inertia_matrix[4],
            inertia_matrix[5];
        std::cout << "Inertia Matrix: \n"
                  << Bm << std::endl;
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

  void read_viscous_friction(const YAML::Node &object, Eigen::Matrix<double, 6, 6> &viscous_friction_matrix)
  {
    if (object["viscous_friction"])
    {
      std::vector<double> viscous_friction = object["viscous_friction"].as<std::vector<double>>();
      if (viscous_friction.size() == 6)
      {
        viscous_friction_matrix.setZero();
        viscous_friction_matrix.diagonal() << viscous_friction[0], viscous_friction[1], viscous_friction[2],
            viscous_friction[3], viscous_friction[4], viscous_friction[5];
        std::cout << "viscous_friction: \n"
                  << viscous_friction_matrix << std::endl;
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

  void read_transform(const YAML::Node &node, Eigen::Matrix<double, 4, 4> &T)
  {
    std::vector<double> translation = node["translation"].as<std::vector<double>>();
    std::vector<double> quaternion = node["quaternion"].as<std::vector<double>>();
    // swap order of quaternion from x y z w to w x y z
    std::vector<double> quaternion_swap = { quaternion[3], quaternion[0], quaternion[1], quaternion[2] };

    T.block<3, 1>(0, 3) = Eigen::Vector3d(translation[0], translation[1], translation[2]);
    Eigen::Quaterniond q(quaternion_swap[0], quaternion_swap[1], quaternion_swap[2], quaternion_swap[3]);
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
  }

  // subscribers to poseStamped
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_publishers_;

  // state publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr object_twist_publisher_;

  // subscribers to WrenchStamped
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_robot1_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_robot2_pub_;

  // timer for ekf update
  rclcpp::TimerBase::SharedPtr timer_;

  // strings to attach at the topic name to subscribe
  std::string robot_1_prefix_;
  std::string robot_2_prefix_;

  // define systems for EKF
  uclv::systems::RobotsObjectSystem::SharedPtr robots_object_system_ptr_;
  uclv::systems::RobotsObjectSystemExt::SharedPtr robots_object_system_ext_ptr_;
  uclv::systems::ForwardEuler<20, 12, Eigen::Dynamic>::SharedPtr discretized_system_ptr_;

  Eigen::Matrix<double, 20, 1> x0_;            // initial state
  Eigen::Matrix<double, Eigen::Dynamic, 1> y_; // variable to store the pose measures
  Eigen::Matrix<double, 12, 1> u_;             // variable to store the force measures

  double sample_time_; // sample time for the filter
  int num_frames_;     // number of frames measuring the object
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulatorRobotsObject>());
  rclcpp::shutdown();
  return 0;
}