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
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "dual_arm_control_interfaces/srv/ekf_service.hpp"

#include "../include/robots_object_system.hpp"
#include "../include/robots_object_system_ext.hpp" // see this file to understande the system
#include <uclv_systems_lib/observers/ekf.hpp>
#include <uclv_systems_lib/discretization/forward_euler.hpp>

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
    this->declare_parameter<double>("sample_time", 0.05);
    this->get_parameter("sample_time", this->sample_time_);

    this->declare_parameter<std::string>("robot_1_prefix", "robot_1");
    this->get_parameter("robot_1_prefix", this->robot_1_prefix_);

    this->declare_parameter<std::string>("robot_2_prefix", "robot_2");
    this->get_parameter("robot_2_prefix", this->robot_2_prefix_);

    // initialize covariance matrices W and V
    W_ << Eigen::Matrix<double, 20, 20>::Identity() * 1;
    Eigen::Matrix<double, 20, 1> W_diag;
    W_diag.setZero();
    // W_diag.block<7, 1>(0, 0) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    // W_diag.block<6, 1>(7, 0) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    // W_diag.block<7, 1>(13, 0) << 0.0000001, 0.0000001, 0.0000001, 0.00000000001, 0.00000000001, 0.00000000001, 0.00000000001;
    // W_ = W_diag.asDiagonal();
    W_.block<13,13>(0,0) = W_.block<13,13>(0,0) * 1e-6;
    W_.block<3,3>(13,13) = W_.block<3,3>(13,13) * 1e-7;
    W_.block<4,4>(16,16) = W_.block<4,4>(16,16) * 1e-9;


    V_single_measure_ << Eigen::Matrix<double, 7, 7>::Identity() * 1;
    V_single_measure_.block<3, 3>(0, 0) = V_single_measure_.block<3, 3>(0, 0) * 1e-4;
    V_single_measure_.block<4, 4>(3, 3) = V_single_measure_.block<4, 4>(3, 3) * 1e-6;

    // initialize publishers
    object_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ekf/object_pose", 1);
    object_twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/ekf/object_twist", 1);
    transform_error_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ekf/transform_error", 1);

    // initialize wrench subscribers
    int index = 0;
    wrench_robot1_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/" + this->robot_1_prefix_ + "/wrench", 1,
        [this, index](const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
        { this->wrench_callback(msg, index); });
    index++;
    wrench_robot2_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/" + this->robot_2_prefix_ + "/wrench", 1,
        [this, index](const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
        { this->wrench_callback(msg, index); });

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
    RCLCPP_INFO(this->get_logger(), "Received request");
    // save initial state
    save_initial_state(request, x0_);

    // read yaml file
    read_yaml_file(request->yaml_file_path.data.c_str(), request->object_name.data.c_str());

    // discretize the system
    discretized_system_ptr_ = std::make_shared<uclv::systems::ForwardEuler<20, 12, Eigen::Dynamic>>(
        robots_object_system_ext_ptr_, sample_time_, x0_);

    // initialize the EKF
    V_.resize(num_frames_ * 14, num_frames_ * 14);
    V_.setIdentity();
    for (int i = 0; i < 2 * num_frames_; i++)
    {
      V_.block<7, 7>(i * 7, i * 7) = V_single_measure_;
    }

    ekf_ptr =
        std::make_shared<uclv::systems::ExtendedKalmanFilter<20, 12, Eigen::Dynamic>>(discretized_system_ptr_, W_, V_);
    ekf_ptr->set_state(this->x0_);

    // start the estimation
    timer_ =
        this->create_wall_timer(std::chrono::seconds((int)(sample_time_)), std::bind(&EKFServer::ekf_callback, this));

    // return the response
    response->success = true;
  }

  void ekf_callback()
  {
    if(messsage_read==false)
    {
      return;
    }

    // std::cout << "EKF Callback" << std::endl;
    // std::cout << "--------------!\n"
    //           << std::endl;
    auto start = std::chrono::high_resolution_clock::now();

    std::cout << "\n y_ measured" << this->y_.transpose() << std::endl;

    std::cout << "\n"
              << std::endl;

    std::cout << "u_ measured \n " << u_.transpose() << std::endl;

    // Eigen::Matrix<double, 20, 1> x_old = ekf_ptr->get_state();

    ekf_ptr->kf_apply(u_, y_, W_, V_);
    x_hat_k_k = ekf_ptr->get_state();

    // Eigen::Matrix<double, 4, 1> qtmp;
    // uclv::geometry_helper::quaternion_continuity(x_hat_k_k.block<4, 1>(3, 0), x_old.block<4, 1>(3, 0), qtmp);
    // Eigen::Quaterniond q_(qtmp(0), qtmp(1), qtmp(2), qtmp(3));
    // q_.normalize();
    // x_hat_k_k.block<4, 1>(3, 0) << q_.w(), q_.vec();

    Eigen::Quaterniond q(x_hat_k_k(3), x_hat_k_k(4), x_hat_k_k(5), x_hat_k_k(6));
    q.normalize();
    x_hat_k_k.block<4, 1>(3, 0) << q.w(), q.vec();
    ekf_ptr->set_state(x_hat_k_k);
    // y_hat_k = ekf->get_output();

    std::cout << "x_hat_k_k: " << x_hat_k_k.transpose() << std::endl;

    // publish the object pose
    geometry_msgs::msg::PoseStamped object_pose_msg;
    object_pose_msg.header.stamp = this->now();
    object_pose_msg.pose.position.x = x_hat_k_k(0);
    object_pose_msg.pose.position.y = x_hat_k_k(1);
    object_pose_msg.pose.position.z = x_hat_k_k(2);
    object_pose_msg.pose.orientation.w = x_hat_k_k(3);
    object_pose_msg.pose.orientation.x = x_hat_k_k(4);
    object_pose_msg.pose.orientation.y = x_hat_k_k(5);
    object_pose_msg.pose.orientation.z = x_hat_k_k(6);
    object_pose_publisher_->publish(object_pose_msg);

    // publish the object twist
    geometry_msgs::msg::TwistStamped object_twist_msg;
    object_twist_msg.header.stamp = this->now();
    object_twist_msg.twist.linear.x = x_hat_k_k(7);
    object_twist_msg.twist.linear.y = x_hat_k_k(8);
    object_twist_msg.twist.linear.z = x_hat_k_k(9);
    object_twist_msg.twist.angular.x = x_hat_k_k(10);
    object_twist_msg.twist.angular.y = x_hat_k_k(11);
    object_twist_msg.twist.angular.z = x_hat_k_k(12);
    object_twist_publisher_->publish(object_twist_msg);

    // publish the transform error
    geometry_msgs::msg::PoseStamped transform_error_msg;
    transform_error_msg.header.stamp = this->now();
    transform_error_msg.pose.position.x = x_hat_k_k(13);
    transform_error_msg.pose.position.y = x_hat_k_k(14);
    transform_error_msg.pose.position.z = x_hat_k_k(15);
    transform_error_msg.pose.orientation.w = x_hat_k_k(16);
    transform_error_msg.pose.orientation.x = x_hat_k_k(17);
    transform_error_msg.pose.orientation.y = x_hat_k_k(18);
    transform_error_msg.pose.orientation.z = x_hat_k_k(19);
    transform_error_publisher_->publish(transform_error_msg);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    RCLCPP_INFO(this->get_logger(), "EKF callback execution time: %f seconds", elapsed.count());
  }

  void save_initial_state(const std::shared_ptr<dual_arm_control_interfaces::srv::EKFService::Request> request,
                          Eigen::Matrix<double, 20, 1> &x0)
  {
    x0.block<7, 1>(0, 0) << request->object_pose.pose.position.x, request->object_pose.pose.position.y,
        request->object_pose.pose.position.z, request->object_pose.pose.orientation.w,
        request->object_pose.pose.orientation.x, request->object_pose.pose.orientation.y,
        request->object_pose.pose.orientation.z;

    x0.block<6, 1>(7, 0) << request->object_twist.twist.linear.x, request->object_twist.twist.linear.y,
        request->object_twist.twist.linear.z, request->object_twist.twist.angular.x,
        request->object_twist.twist.angular.y, request->object_twist.twist.angular.z;

    x0.block<7, 1>(13, 0) << request->transform_error.pose.position.x, request->transform_error.pose.position.y,
        request->transform_error.pose.position.z, request->transform_error.pose.orientation.w,
        request->transform_error.pose.orientation.x, request->transform_error.pose.orientation.y,
        request->transform_error.pose.orientation.z;

    std::cout << "Initial State: \n"
              << x0.transpose() << std::endl;
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
    int index = 0;
    for (const auto &frame_name : frame_names)
    {
      pose_subscribers_.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/" + object_name + "/" + frame_name + "/" + this->robot_1_prefix_ + "/pose", 1,
          [this, index](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
          { this->pose_callback(msg, index); }));

      pose_subscribers_.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/" + object_name + "/" + frame_name + "/" + this->robot_2_prefix_ + "/pose", 1,
          [this, num_frames, index](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
          {
            this->pose_callback(msg, num_frames + index);
          }));

      index++;
    }
    num_frames_ = num_frames;

    // create the system
    Eigen::Matrix<double, 13, 1> x0_system;
    robots_object_system_ptr_ = std::make_shared<uclv::systems::RobotsObjectSystem>(
        x0_system, Bm, bg, oTg1, oTg2, b1Tb2, bTb1, viscous_friction_matrix, num_frames_);

    // create the extended system
    robots_object_system_ext_ptr_ =
        std::make_shared<uclv::systems::RobotsObjectSystemExt>(x0_, robots_object_system_ptr_);

    // resize the output variable
    y_.resize(num_frames_ * 14, 1);
    y_.setZero();
    // initialize with unitquaternion the elements corresponding to the orientation
    for (int i = 0; i < num_frames_; i++)
    {
      y_.block<4, 1>(i * 7 + 3, 0) << 1, 0, 0, 0;
      y_.block<4, 1>((i + num_frames_) * 7 + 3, 0) << 1, 0, 0, 0;
    }
  }

  void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg, const int &index)
  {
    RCLCPP_INFO(this->get_logger(), "Received wrench from %d", index);
    this->u_.block<6, 1>(index * 6, 0) << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
        msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const int &index)
  {
    RCLCPP_INFO(this->get_logger(), "Received pose from %d", index);
    this->y_.block<7, 1>(index * 7, 0) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
        msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;

    if(messsage_read == false)
    {
      messsage_read = true;
    }
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
    std::swap(quaternion[0], quaternion[3]);

    T.block<3, 1>(0, 3) = Eigen::Vector3d(translation[0], translation[1], translation[2]);
    Eigen::Quaterniond q(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
  }

  rclcpp::Service<dual_arm_control_interfaces::srv::EKFService>::SharedPtr server_;
  rclcpp::CallbackGroup::SharedPtr
      reentrant_cb_group_; // see https://docs.ros.org/en/foxy/How-To-Guides/Using-callback-groups.html
  rclcpp::SubscriptionOptions options_cb_group_;

  // subscribers to poseStamped
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subscribers_;

  // subscribers to WrenchStamped
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_robot1_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_robot2_sub_;

  // timer for ekf update
  rclcpp::TimerBase::SharedPtr timer_;

  // publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr object_twist_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr transform_error_publisher_;

  // strings to attach at the topic name to subscribe
  std::string robot_1_prefix_;
  std::string robot_2_prefix_;

  // define systems for EKF
  uclv::systems::RobotsObjectSystem::SharedPtr robots_object_system_ptr_;
  uclv::systems::RobotsObjectSystemExt::SharedPtr robots_object_system_ext_ptr_;
  uclv::systems::ForwardEuler<20, 12, Eigen::Dynamic>::SharedPtr discretized_system_ptr_;
  uclv::systems::ExtendedKalmanFilter<20, 12, Eigen::Dynamic>::SharedPtr ekf_ptr;

  Eigen::Matrix<double, 20, 1> x0_;                         // filter initial state
  Eigen::Matrix<double, 20, 1> x_hat_k_k;                   // filter state
  Eigen::Matrix<double, Eigen::Dynamic, 1> y_;              // variable to store the pose measures
  Eigen::Matrix<double, 12, 1> u_;                          // variable to store the force measures
  Eigen::Matrix<double, 20, 20> W_;                         // process noise covariance matrix
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V_; // measurement noise covariance matrix
  Eigen::Matrix<double, 7, 7> V_single_measure_;            // covaiance matrix for the single measure

  double sample_time_; // sample time for the filter
  int num_frames_;     // number of frames measuring the object

  bool messsage_read = false;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFServer>());
  rclcpp::shutdown();
  return 0;
}