
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "dual_arm_control_interfaces/srv/ekf_service.hpp"

#include "../include/robots_spring_object_system.hpp"
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
    this->declare_parameter<double>("sample_time", 0.002);
    this->get_parameter("sample_time", this->sample_time_);

    this->declare_parameter<double>("publishing_sample_time", 0.01);
    this->get_parameter("publishing_sample_time", this->publishing_sample_time);

    this->declare_parameter<std::string>("robot_1_prefix", "robot1");
    this->get_parameter("robot_1_prefix", this->robot_1_prefix_);

    this->declare_parameter<std::string>("robot_2_prefix", "robot2");
    this->get_parameter("robot_2_prefix", this->robot_2_prefix_);

    // initialize wrench publishers
    wrench_robot1_pub_ =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>("/" + this->robot_1_prefix_ + "/wrench", 1);
    wrench_robot2_pub_ =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>("/" + this->robot_2_prefix_ + "/wrench", 1);

    // initialize wrench publishers
    fkine_robot1_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/" + this->robot_1_prefix_ + "/fkine", 1);
    fkine_robot2_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/" + this->robot_2_prefix_ + "/fkine", 1);

    // initialize pose publisher
    object_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/object_pose", 1);
    object_twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/object_twist", 1);
    robot1_fkine_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(this->robot_1_prefix_ + "/fkine", 1);
    robot2_fkine_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(this->robot_2_prefix_ + "/fkine", 1);

    // define yaml file path
    // std::string yaml_file_path =
    // "/home/mdesimone/dual_arm_ws/src/dual_arm_control/dual_arm_control/config/config.yaml"; std::string
    // yaml_file_path = "/home/marco/dual_arm_ws/src/dual_arm_control/dual_arm_control/config/config.yaml";
    std::string yaml_file_path =
        "/home/mdesimone/cooperative_robots_ws/src/dual_arm_control/dual_arm_control/config/config.yaml";

    std::string object_name = "resin_block_1";

    // control input and initial state
    x0_.resize(27, 1);
    x0_ << 0.8, 0.01, 0.15, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.8000, 0.1100, 0.1500, 1, 0, 0, 0, -0.0900, 0.8300, 0.1500,
        0.7071, 0, 0, -0.7071;
    u_.resize(12, 1);
    u_ << 0.0, 0.0, 0.0, 0, 0, 0.0, -0.0, -0.0, 0.0, 0, 0, 0;

    // read the yaml file
    read_yaml_file(yaml_file_path, object_name);

    // create the timer
    timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(sample_time_ * 1000)),
                                     std::bind(&SimulatorRobotsObject::timer_callback, this));
    // create the timer
    publish_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(publishing_sample_time * 1000)),
                                             std::bind(&SimulatorRobotsObject::publish_timer_callback, this));

    cmd_twist_robot1_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/" + this->robot_1_prefix_ + "/cmd_twist", 1,
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { this->cmd_twist_callback(msg, 0); });
    cmd_twist_robot2_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/" + this->robot_2_prefix_ + "/cmd_twist", 1,
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { this->cmd_twist_callback(msg, 1); });
  }

private:
  void cmd_twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg, int robot)
  {
    u_(robot * 6) = msg->twist.linear.x;
    u_(robot * 6 + 1) = msg->twist.linear.y;
    u_(robot * 6 + 2) = msg->twist.linear.z;
    u_(robot * 6 + 3) = msg->twist.angular.x;
    u_(robot * 6 + 4) = msg->twist.angular.y;
    u_(robot * 6 + 5) = msg->twist.angular.z;
  }

  void publish_timer_callback()
  {
    Eigen::Matrix<double, 27, 1> x = discretized_system_ptr_->get_state();
    y_ = discretized_system_ptr_->get_output();

    auto timestamp = this->now();

    // publish the measures
    for (int i = 0; i < 2 * num_frames_; i++)
    {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = timestamp;
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
    int start_position_wrenchs = num_frames_ * 2 * 7;
    int sign_wrench = 1; // change sign wrench to be consistent with the sign convention of the sensors
    for (int i = 0; i < 2; i++)
    {
      geometry_msgs::msg::WrenchStamped wrench_msg;
      wrench_msg.header.stamp = timestamp;
      wrench_msg.wrench.force.x = sign_wrench*y_(start_position_wrenchs + i * 6);
      wrench_msg.wrench.force.y = sign_wrench*y_(start_position_wrenchs + i * 6 + 1);
      wrench_msg.wrench.force.z = sign_wrench*y_(start_position_wrenchs + i * 6 + 2);
      wrench_msg.wrench.torque.x = sign_wrench*y_(start_position_wrenchs + i * 6 + 3);
      wrench_msg.wrench.torque.y = sign_wrench*y_(start_position_wrenchs + i * 6 + 4);
      wrench_msg.wrench.torque.z = sign_wrench*y_(start_position_wrenchs + i * 6 + 5);
      if (i == 0)
      {
        wrench_robot1_pub_->publish(wrench_msg);
      }
      else
      {
        wrench_robot2_pub_->publish(wrench_msg);
      }
    }

    // publish fkine
    int start_position_fkine = num_frames_ * 2 * 7 + 12;
    geometry_msgs::msg::PoseStamped fkine_msg;
    fkine_msg.header.stamp = timestamp;
    fkine_msg.header.frame_id = this->robot_1_prefix_;
    fkine_msg.pose.position.x = y_(start_position_fkine);
    fkine_msg.pose.position.y = y_(start_position_fkine + 1);
    fkine_msg.pose.position.z = y_(start_position_fkine + 2);
    fkine_msg.pose.orientation.w = y_(start_position_fkine + 3);
    fkine_msg.pose.orientation.x = y_(start_position_fkine + 4);
    fkine_msg.pose.orientation.y = y_(start_position_fkine + 5);
    fkine_msg.pose.orientation.z = y_(start_position_fkine + 6);
    fkine_robot1_pub_->publish(fkine_msg);
    fkine_msg.header.frame_id = this->robot_2_prefix_;
    fkine_msg.pose.position.x = y_(start_position_fkine + 7);
    fkine_msg.pose.position.y = y_(start_position_fkine + 8);
    fkine_msg.pose.position.z = y_(start_position_fkine + 9);
    fkine_msg.pose.orientation.w = y_(start_position_fkine + 10);
    fkine_msg.pose.orientation.x = y_(start_position_fkine + 11);
    fkine_msg.pose.orientation.y = y_(start_position_fkine + 12);
    fkine_msg.pose.orientation.z = y_(start_position_fkine + 13);
    fkine_robot2_pub_->publish(fkine_msg);

    // publish pose
    geometry_msgs::msg::PoseStamped object_pose_msg;
    object_pose_msg.header.stamp = timestamp;
    object_pose_msg.header.frame_id = "world";
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
    object_twist_msg.header.stamp = timestamp;
    object_twist_msg.twist.linear.x = x(7);
    object_twist_msg.twist.linear.y = x(8);
    object_twist_msg.twist.linear.z = x(9);
    object_twist_msg.twist.angular.x = x(10);
    object_twist_msg.twist.angular.y = x(11);
    object_twist_msg.twist.angular.z = x(12);
    object_twist_publisher_->publish(object_twist_msg);

    // publish the robot fkine
    geometry_msgs::msg::PoseStamped robot1_fkine_msg;
    robot1_fkine_msg.header.stamp = timestamp;
    robot1_fkine_msg.header.frame_id = this->robot_1_prefix_;
    robot1_fkine_msg.pose.position.x = x(13);
    robot1_fkine_msg.pose.position.y = x(14);
    robot1_fkine_msg.pose.position.z = x(15);
    robot1_fkine_msg.pose.orientation.w = x(16);
    robot1_fkine_msg.pose.orientation.x = x(17);
    robot1_fkine_msg.pose.orientation.y = x(18);
    robot1_fkine_msg.pose.orientation.z = x(19);
    robot1_fkine_pub_->publish(robot1_fkine_msg);

    geometry_msgs::msg::PoseStamped robot2_fkine_msg;
    robot2_fkine_msg.header.stamp = timestamp;
    robot2_fkine_msg.header.frame_id = this->robot_2_prefix_;
    robot2_fkine_msg.pose.position.x = x(20);
    robot2_fkine_msg.pose.position.y = x(21);
    robot2_fkine_msg.pose.position.z = x(22);
    robot2_fkine_msg.pose.orientation.w = x(23);
    robot2_fkine_msg.pose.orientation.x = x(24);
    robot2_fkine_msg.pose.orientation.y = x(25);
    robot2_fkine_msg.pose.orientation.z = x(26);
    robot2_fkine_pub_->publish(robot2_fkine_msg);
  }

  void timer_callback()
  {
    x0_ = discretized_system_ptr_->get_state();
    discretized_system_ptr_->step(u_);

    // ensure the quaternion continuity
    Eigen::Matrix<double, 27, 1> x = discretized_system_ptr_->get_state();
    Eigen::Matrix<double, 4, 1> q;

    uclv::geometry_helper::quaternion_continuity(x.block<4, 1>(3, 0), x0_.block<4, 1>(3, 0), q);
    Eigen::Quaterniond q_(q(0), q(1), q(2), q(3));
    q_.normalize();
    x.block<4, 1>(3, 0) << q_.w(), q_.vec();

    // std::cout << "x: \n"
    //           << x.transpose() << std::endl;
    discretized_system_ptr_->set_state(x);
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
    Eigen::Matrix<double, 4, 4> b2Tb1 = b1Tb2.inverse();

    // read bTb1
    Eigen::Matrix<double, 4, 4> bTb1;
    bTb1.setIdentity();
    read_transform(config["bTb1"], bTb1);
    std::cout << "bTb1: \n" << bTb1 << std::endl;

    YAML::Node object_node = config[object_name];

    // read viscous friction
    Eigen::Matrix<double, 6, 6> viscous_friction_matrix;
    read_matrix_6x6(config, viscous_friction_matrix, "viscous_friction");

    // read spring stiffness K_1
    Eigen::Matrix<double, 6, 6> K_1_matrix;
    read_matrix_6x6(config, K_1_matrix, "K_1");

    // read spring stiffness K_2
    Eigen::Matrix<double, 6, 6> K_2_matrix;
    read_matrix_6x6(config, K_2_matrix, "K_2");

    // read spring damping B_1
    Eigen::Matrix<double, 6, 6> B_1_matrix;
    read_matrix_6x6(config, B_1_matrix, "B_1");

    // read spring damping B_2
    Eigen::Matrix<double, 6, 6> B_2_matrix;
    read_matrix_6x6(config, B_2_matrix, "B_2");

    // read Bm
    Eigen::Matrix<double, 6, 6> Bm;
    read_matrix_6x6(object_node, Bm, "inertia_matrix");

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
    for (const auto& frame_name : frame_names)
    {
      pose_publishers_.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>(
          this->robot_1_prefix_ + "/" + object_name + "/" + frame_name + "/pose", 1));
    }

    for (const auto& frame_name : frame_names)
    {
      pose_publishers_.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>(
          this->robot_2_prefix_ + "/" + object_name + "/" + frame_name + "/pose", 1));
    }

    num_frames_ = num_frames;

    // create the system
    robots_object_system_ptr_ = std::make_shared<uclv::systems::RobotsSpringObjectSystem>(
        x0_, Bm, bg, oTg1, oTg2, b2Tb1, bTb1, viscous_friction_matrix, K_1_matrix, B_1_matrix, K_2_matrix, B_2_matrix,
        num_frames_);

    // discretize the system
    discretized_system_ptr_ = std::make_shared<uclv::systems::ForwardEuler<27, 12, Eigen::Dynamic>>(
        robots_object_system_ptr_, sample_time_, x0_);

    discretized_system_ptr_->display();

    // resize the output variable
    y_.resize(num_frames_ * 14 + 12 + 14, 1);
  }

  void read_matrix_6x6(const YAML::Node& object, Eigen::Matrix<double, 6, 6>& matrix,
                       const std::string& name = "viscous_friction")
  {
    if (object[name])
    {
      std::vector<double> matrix_diag = object[name].as<std::vector<double>>();
      if (matrix_diag.size() == 6)
      {
        matrix.setZero();
        matrix.diagonal() << matrix_diag[0], matrix_diag[1], matrix_diag[2], matrix_diag[3], matrix_diag[4],
            matrix_diag[5];
        std::cout << name << ": \n" << matrix << std::endl;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "%s vector size is not 6", name.c_str());
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "%s not found in the YAML file", name.c_str());
    }
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

  // subscribers to poseStamped
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_publishers_;

  // state publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr object_twist_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot1_fkine_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot2_fkine_pub_;

  // subscribers to WrenchStamped
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_robot1_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_robot2_pub_;

  // subscribers to fkine
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fkine_robot1_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fkine_robot2_pub_;

  // subscribers to WrenchStamped for external commands
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_twist_robot1_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_twist_robot2_sub_;

  // timer for ekf update
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // strings to attach at the topic name to subscribe
  std::string robot_1_prefix_;
  std::string robot_2_prefix_;

  // define systems for EKF
  uclv::systems::RobotsSpringObjectSystem::SharedPtr robots_object_system_ptr_;
  uclv::systems::ForwardEuler<27, 12, Eigen::Dynamic>::SharedPtr discretized_system_ptr_;

  Eigen::Matrix<double, 27, 1> x0_;             // initial state
  Eigen::Matrix<double, Eigen::Dynamic, 1> y_;  // variable to store the pose measures
  Eigen::Matrix<double, 12, 1> u_;              // variable to store the force measures

  double sample_time_;            // sample time for the filter
  double publishing_sample_time;  // sample time for the publishing
  int num_frames_;                // number of frames measuring the object
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulatorRobotsObject>());
  rclcpp::shutdown();
  return 0;
}