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

#include "../include/robots_spring_object_system.hpp"
#include "../include/robots_spring_object_system_ext.hpp"  // see this file to understande the system

#include <uclv_systems_lib/observers/ekf.hpp>
#include <uclv_systems_lib/discretization/forward_euler.hpp>

#include <eigen3/Eigen/Geometry>
#include <chrono>
#include <iostream>
#include <memory>
#include <yaml-cpp/yaml.h>

#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

constexpr int dim_state = 34;

class EKFServer : public rclcpp::Node
{
public:
  EKFServer() : Node("ekf_server")
  {
    // declare parameters
    this->declare_parameter<double>("sample_time", 0.001);
    this->get_parameter("sample_time", this->sample_time_);

    this->declare_parameter<double>("publishing_sample_time", 0.01);
    this->get_parameter("publishing_sample_time", this->publishing_sample_time_);

    this->declare_parameter<std::string>("robot_1_prefix", "robot_1");
    this->get_parameter("robot_1_prefix", this->robot_1_prefix_);

    this->declare_parameter<std::string>("robot_2_prefix", "robot_2");
    this->get_parameter("robot_2_prefix", this->robot_2_prefix_);

    this->declare_parameter<std::string>("base_frame_name", "base_frame");
    this->get_parameter("base_frame_name", this->base_frame_name);

    this->declare_parameter<std::vector<double>>(
        "covariance_state_diagonal",
        std::vector<double>{ 1e-12, 1e-12, 1e-5, 1e-5, 1e-12, 1e-12, 1e-12, 1e-12, 1e-8, 1e-10 });
    std::vector<double> covariance_state_diagonal;
    this->get_parameter("covariance_state_diagonal", covariance_state_diagonal);

    // check if the covariance_state_diagonal has the correct size
    if (covariance_state_diagonal.size() != 10)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "The covariance_state_diagonal parameter must have 10 elements representing the covariance on the "
                   "state. The first is the element for the position terms x,y,z. The next, is the covariance for "
                   "the quaternion terms qw,qx,qy,qz. The third is the covariance for the linear velocity terms "
                   "vx,vy,vz. "
                   "The fourth is the covariance for the angular velocity terms omegax,omegay,omegaz. The fifth is the "
                   "covariance of the position part of the calibration matrix between the two robots. "
                   "The sixth is the covariance of the position of the robot 1 fkine as well as the eight"
                   "The seventh is the covariance of the position of the robot 2 fkine as well as the ninth"
                   "The last is the covariance of the orientation part of the calibration matrix between the two "
                   "robots. the last two are the covariance of the force and torque measures ");
      return;
    }

    this->declare_parameter<std::vector<double>>(
        "covariance_measure_diagonal",
        std::vector<double>{ 1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-2, 1e-2,1e-2, 1e-2,1e-2, 1e-2, 1e-9, 1e-9, 1e-9, 1e-9 });
    std::vector<double> covariance_measure_diagonal;
    this->get_parameter("covariance_measure_diagonal", covariance_measure_diagonal);

    // check if the covariance_measure_diagonal has the correct size
    if (covariance_measure_diagonal.size() != 17)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "The covariance_measure_diagonal parameter must have 15 elements representing the covariance on the "
                   "measure, the first 3 are the position terms x,y,z, the next 4 are the quaternion terms "
                   "qw,qx,qy,qz.The next 6 are the covariance of the force and torque measures . The last 4 are the "
                   "covariance measure of the robot 1 and robot 2 fkine respectively");
      return;
    }

    // initialize covariance matrices W and V
    W_default_ << Eigen::Matrix<double, dim_state, dim_state>::Identity() * 1;
    W_default_.block<3, 3>(0, 0) = W_default_.block<3, 3>(0, 0) * covariance_state_diagonal[0];
    W_default_.block<4, 4>(3, 3) = W_default_.block<4, 4>(3, 3) * covariance_state_diagonal[1];
    W_default_.block<3, 3>(7, 7) = W_default_.block<3, 3>(7, 7) * covariance_state_diagonal[2];
    W_default_.block<3, 3>(10, 10) = W_default_.block<3, 3>(10, 10) * covariance_state_diagonal[3];
    W_default_.block<3, 3>(13, 13) = W_default_.block<3, 3>(13, 13) * covariance_state_diagonal[4];
    W_default_.block<4, 4>(16, 16) = W_default_.block<4, 4>(16, 16) * covariance_state_diagonal[5];
    W_default_.block<3, 3>(20, 20) = W_default_.block<3, 3>(20, 20) * covariance_state_diagonal[6];
    W_default_.block<4, 4>(23, 23) = W_default_.block<4, 4>(23, 23) * covariance_state_diagonal[7];
    W_default_.block<3, 3>(27, 27) = W_default_.block<3, 3>(27, 27) * covariance_state_diagonal[8];
    W_default_.block<4, 4>(30, 30) = W_default_.block<4, 4>(30, 30) * covariance_state_diagonal[9];

    // multiply W_default by the sample time
    W_default_ = W_default_ * sample_time_ / (0.001);  // covariance tuned with sample time == 1e-3

    W_ = W_default_;

    std::cout << "\n Initial W covariance matrix\n " << W_ << std::endl;

    V_single_measure_ << Eigen::Matrix<double, 7, 7>::Identity() * 1;
    for (int i = 0; i < 7; i++)
    {
      V_single_measure_(i, i) = covariance_measure_diagonal[i];
    }

    std::cout << "\nInitial single measure V covariance matrix\n" << V_single_measure_ << std::endl;

    V_forces_ << Eigen::Matrix<double, 12, 12>::Identity() * 1;
    
    V_forces_.block<6, 6>(0, 0) = Eigen::Matrix<double, 6, 6>::Identity();
    for (int i = 0; i < 6; i++)
    {
      V_forces_(i, i) = covariance_measure_diagonal[7 + i];
    }

    V_forces_.block<6, 6>(6, 6) = V_forces_.block<6, 6>(0, 0);

    std::cout << "\nInitial forces V covariance matrix\n" << V_forces_ << std::endl;

    V_fkine_robots_ << Eigen::Matrix<double, 14, 14>::Identity() * 1;
    V_fkine_robots_.block<3, 3>(0, 0) = V_fkine_robots_.block<3, 3>(0, 0) * covariance_measure_diagonal[13];
    V_fkine_robots_.block<4, 4>(3, 3) = V_fkine_robots_.block<4, 4>(3, 3) * covariance_measure_diagonal[14];
    V_fkine_robots_.block<3, 3>(7, 7) = V_fkine_robots_.block<3, 3>(7, 7) * covariance_measure_diagonal[15];
    V_fkine_robots_.block<4, 4>(10, 10) = V_fkine_robots_.block<4, 4>(10, 10) * covariance_measure_diagonal[16];

    std::cout << "\nInitial fkine V covariance matrix\n" << V_fkine_robots_ << std::endl;

    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);

    // initialize publishers
    object_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ekf/object_pose", qos);
    object_twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/ekf/object_twist", qos);
    transform_b2Tb1_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ekf/b2Tb1_filtered", qos);
    transform_b1Tb2_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ekf/b1Tb2_filtered", qos);
    filtered_wrench_1_publisher_ =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>("/ekf/wrench_robot1_filtered", qos);
    filtered_wrench_2_publisher_ =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>("/ekf/wrench_robot2_filtered", qos);

    fkine_robot1_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ekf/fkine1_filtered", 1);
    fkine_robot2_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ekf/fkine2_filtered", 1);

    // initialize twist subscribers
    int index = 0;
    twist_robot1_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        this->robot_1_prefix_ + "/twist_fkine", qos,
        [this, index](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
          this->twist_fkine_callback(msg, index);
        });
    index++;
    twist_robot2_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        this->robot_2_prefix_ + "/twist_fkine", qos,
        [this, index](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
          this->twist_fkine_callback(msg, index);
        });

    // Create the service server
    server_ = this->create_service<dual_arm_control_interfaces::srv::EKFService>(
        "ekf_service",
        std::bind(&EKFServer::handle_service_request, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "EKFService service server ready!");

    // create service
    server_object_grasped_ = this->create_service<std_srvs::srv::SetBool>(
        "ekf_set_object_grasped",
        std::bind(&EKFServer::handle_grasp_service_request, this, std::placeholders::_1, std::placeholders::_2));

    // iniit variables
    x0_.setZero();
    y_.setZero();
    u_.setZero();
    // debug publisher
    debug_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ekf/debug_msg", 1);
  }

private:
  void reset_filter()
  {
    filter_initialized_ = false;
    timer_->cancel();
    timer_publish_->cancel();
    ekf_ptr.reset();
    discretized_system_ptr_.reset();
    robots_object_system_ptr_.reset();
    robots_object_system_ext_ptr_.reset();
    robots_object_system_ext_ptr_.reset();

    x0_.setZero();
    y_.setZero();
    u_.setZero();
    W_.setZero();
    V_.setZero();
    num_frames_ = 0;

    W_ = W_default_;

    // remove the subscribers
    pose_subscribers_.clear();

    // remove the publishers
    filtered_pose_publishers_.clear();

    b1Tb2_convergence_status_ = false;

    // reset grasp detection
    object_grasped_[0] = false;
    object_grasped_[1] = false;

    mass_estimated_flag_ = false;
    mass_estimated_ = 0.0;
    mass_estimated_counter_ = 0;
  }

  void handle_service_request(const std::shared_ptr<dual_arm_control_interfaces::srv::EKFService::Request> request,
                              std::shared_ptr<dual_arm_control_interfaces::srv::EKFService::Response> response)
  {
    if (filter_initialized_)
    {
      RCLCPP_WARN(this->get_logger(), "Filter already initialized : the filter will be reset");
      reset_filter();
    }

    RCLCPP_INFO(this->get_logger(), "Received request");
    // save initial state
    save_initial_state(request, x0_);

    // read yaml file
    read_yaml_file(request->yaml_file_path.data.c_str(), request->object_name.data.c_str());

    // discretize the system
    discretized_system_ptr_ = std::make_shared<uclv::systems::ForwardEuler<dim_state, 12, Eigen::Dynamic>>(
        robots_object_system_ext_ptr_, sample_time_, x0_);

    // initialize the EKF
    V_.resize(num_frames_ * 14 + 12 + 14, num_frames_ * 14 + 12 + 14);
    V_.setIdentity();
    V_ = V_ * 1e10;

    ekf_ptr = std::make_shared<uclv::systems::ExtendedKalmanFilter<dim_state, 12, Eigen::Dynamic>>(
        discretized_system_ptr_, W_.block<dim_state, dim_state>(0, 0), V_);
    ekf_ptr->set_state(this->x0_.block<dim_state, 1>(0, 0));

    // start the estimation timer
    timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(sample_time_ * 1000)),
                                     std::bind(&EKFServer::ekf_callback, this));

    // timer for publishing the msgs
    timer_publish_ = this->create_wall_timer(std::chrono::milliseconds((int)(publishing_sample_time_ * 1000)),
                                             std::bind(&EKFServer::publish_callback, this));

    // initialize wrench subscribers
    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);
    int index = 0;
    wrench_robot1_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        this->robot_1_prefix_ + "/wrench", qos,
        [this, index](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) { this->wrench_callback(msg, index); });
    index++;
    wrench_robot2_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        this->robot_2_prefix_ + "/wrench", qos,
        [this, index](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) { this->wrench_callback(msg, index); });

    // initialize fkine subscribers
    index = 0;
    fkine_robot1_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->robot_1_prefix_ + "/fkine", qos,
        [this, index](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->fkine_callback(msg, index); });
    index++;
    fkine_robot2_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->robot_2_prefix_ + "/fkine", qos,
        [this, index](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->fkine_callback(msg, index); });

    // return the response
    filter_initialized_ = true;
    response->success = filter_initialized_;
  }

  void ekf_callback()
  {
    auto start = std::chrono::high_resolution_clock::now();

    std::cout << "Masure pose received vector: " << this->pose_measure_received_.transpose() << "\n" << std::endl;
    std::cout << "Masure force received vector: " << this->force_measure_received_.transpose() << "\n";
    std::cout << "Masure fkine received vector: " << this->fkine_measure_received_.transpose() << "\n";

    // update covariance on the base of the received measurements
    for (int i = 0; i < 2 * num_frames_; i++)
    {
      if (pose_measure_received_(i))
      {
        V_.block<7, 7>(i * 7, i * 7) = V_single_measure_;
      }
      else
      {
        // V_.block<7, 7>(i * 7, i * 7) = Eigen::Matrix<double, 7, 7>::Identity() * 1e10;
      }
      // pose_measure_received_(i) = false;
    }
    for (int i = 0; i < 2; i++)
    {
      if (force_measure_received_(i))
      {
        // V_.block<6, 6>(num_frames_ * 14 + i * 6, num_frames_ * 14 + i * 6) = V_forces_.block<6, 6>(i * 6, i * 6);
        V_.block<6, 6>(num_frames_ * 14 + i * 6, num_frames_ * 14 + i * 6) =
        Eigen::Matrix<double, 6, 6>::Identity() * 1e10;
      }
      else
      {
        V_.block<6, 6>(num_frames_ * 14 + i * 6, num_frames_ * 14 + i * 6) =
            Eigen::Matrix<double, 6, 6>::Identity() * 1e10;
      }
      force_measure_received_(i) = false;

      if (fkine_measure_received_(i))
      {
        V_.block<7, 7>(num_frames_ * 14 + 12 + i * 7, num_frames_ * 14 + 12 + i * 7) =
            V_fkine_robots_.block<7, 7>(i * 7, i * 7);
      }
      else
      {
        // V_.block<7, 7>(num_frames_ * 14 + 12 + i * 7, num_frames_ * 14 + 12 + i * 7) =
        //     Eigen::Matrix<double, 7, 7>::Identity() * 1e10;
      }
      fkine_measure_received_(i) = false;
    }

    Eigen::Matrix<double, dim_state, 1> x_old = ekf_ptr->get_state();

    // std::cout << "\n\nEKF update\n";
    // std::cout << "u_: " << u_.transpose() << "\n";
    // for (int i = 0; i < y_.size(); ++i) {
    //   std::cout << "y_[" << i << "]: " << y_(i) << "\n";
    // }
    ekf_ptr->kf_apply(u_, y_, W_, V_);
    x_hat_k_k = ekf_ptr->get_state();

    // std::cout << "EKF ITERATION \n";
    // std::cout << "x_hat_k_k bTo: " << x_hat_k_k.block<7, 1>(0, 0).transpose() << "\n";
    // std::cout << "x_hat_k_k o_twist_o: " << x_hat_k_k.block<6, 1>(7, 0).transpose() << "\n";
    // std::cout << "x_hat_k_k b1Te1: " << x_hat_k_k.block<7, 1>(13, 0).transpose() << "\n";
    // std::cout << "x_hat_k_k b2Te2: " << x_hat_k_k.block<7, 1>(20, 0).transpose() << "\n";
    // std::cout << "x_hat_k_k b1Tb2: " << x_hat_k_k.block<7, 1>(27, 0).transpose() << "\n";
    // int position_measure_vector = num_frames_ * 2 * 7 + 12 + 7 * 0;
    // std::cout << "y robt1: " << y_.block(position_measure_vector, 0, 7, 1).transpose() << "\n";
    // position_measure_vector = num_frames_ * 2 * 7 + 12 + 7 * 1;
    // std::cout << "y robt2: " << y_.block(position_measure_vector, 0, 7, 1).transpose() << "\n";

    // ensure quaternion continuity bQo
    Eigen::Matrix<double, 4, 1> qtmp;
    uclv::geometry_helper::quaternion_continuity(x_hat_k_k.block<4, 1>(3, 0), x_old.block<4, 1>(3, 0), qtmp);
    Eigen::Quaterniond q_(qtmp(0), qtmp(1), qtmp(2), qtmp(3));
    q_.normalize();
    x_hat_k_k.block<4, 1>(3, 0) << q_.w(), q_.vec();

    // ensure quaternion continuity b1Qe1
    {
      qtmp << x_hat_k_k.block<4, 1>(16, 0);
      uclv::geometry_helper::quaternion_continuity(qtmp, x_old.block<4, 1>(16, 0), qtmp);
      Eigen::Quaterniond qhat(qtmp(0), qtmp(1), qtmp(2), qtmp(3));
      qhat.normalize();
      x_hat_k_k.block<4, 1>(16, 0) << qhat.w(), qhat.vec();
    }

    // ensure quaternion continuity b2Qe2
    {
      qtmp << x_hat_k_k.block<4, 1>(23, 0);
      uclv::geometry_helper::quaternion_continuity(qtmp, x_old.block<4, 1>(23, 0), qtmp);
      Eigen::Quaterniond qhat(qtmp(0), qtmp(1), qtmp(2), qtmp(3));
      qhat.normalize();
      x_hat_k_k.block<4, 1>(23, 0) << qhat.w(), qhat.vec();
    }

    {
      // ensure quaternion continuity b2Qb1
      qtmp << x_hat_k_k.block<4, 1>(30, 0);
      uclv::geometry_helper::quaternion_continuity(qtmp, x_old.block<4, 1>(30, 0), qtmp);
      Eigen::Quaterniond qhat(qtmp(0), qtmp(1), qtmp(2), qtmp(3));
      qhat.normalize();
      x_hat_k_k.block<4, 1>(30, 0) << qhat.w(), qhat.vec();
    }

    // update filter state
    ekf_ptr->set_state(x_hat_k_k);

    // check b1Tb2 convergence
    if (b1Tb2_convergence_status_ == false && last_pose_msg_robot1_ && last_pose_msg_robot2_)
    {
      rclcpp::Time time_measure_1 = last_pose_msg_robot1_->header.stamp;
      rclcpp::Time time_measure_2 = last_pose_msg_robot2_->header.stamp;

      if (time_measure_1 - time_measure_2 < rclcpp::Duration::from_seconds(0.05) &&
          time_measure_1 - this->now() < rclcpp::Duration::from_seconds(0.05) &&
          time_measure_2 - this->now() < rclcpp::Duration::from_seconds(0.05))
      {
        Eigen::Matrix<double, 7, 1> aruco_pose_1;
        aruco_pose_1 << last_pose_msg_robot1_->pose.position.x, last_pose_msg_robot1_->pose.position.y,
            last_pose_msg_robot1_->pose.position.z, last_pose_msg_robot1_->pose.orientation.w,
            last_pose_msg_robot1_->pose.orientation.x, last_pose_msg_robot1_->pose.orientation.y,
            last_pose_msg_robot1_->pose.orientation.z;

        Eigen::Matrix<double, 7, 1> aruco_pose_2;
        aruco_pose_2 << last_pose_msg_robot2_->pose.position.x, last_pose_msg_robot2_->pose.position.y,
            last_pose_msg_robot2_->pose.position.z, last_pose_msg_robot2_->pose.orientation.w,
            last_pose_msg_robot2_->pose.orientation.x, last_pose_msg_robot2_->pose.orientation.y,
            last_pose_msg_robot2_->pose.orientation.z;

        check_b1Tb2_convergence(aruco_pose_1, aruco_pose_2);
      }
    }
    std::cout << "b1Tb2_convergence_status_: " << b1Tb2_convergence_status_ << "\n";

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Elapsed time: " << elapsed.count() << " s\n";
  }

  void publish_callback()
  {
    // get the state estimate
    x_hat_k_k = ekf_ptr->get_state();
    // get the filtered measures
    y_filtered_ = ekf_ptr->get_output();

    // get time stamp for the messages
    auto time_stamp = this->now();

    // publish messages
    geometry_msgs::msg::PoseStamped filtered_pose_msg;
    for (int i = 0; i < num_frames_; i++)
    {
      filtered_pose_msg.header.stamp = time_stamp;
      filtered_pose_msg.header.frame_id = measure_1_base_frame;
      filtered_pose_msg.pose.position.x = y_filtered_(7 * i);
      filtered_pose_msg.pose.position.y = y_filtered_(7 * i + 1);
      filtered_pose_msg.pose.position.z = y_filtered_(7 * i + 2);
      filtered_pose_msg.pose.orientation.w = y_filtered_(7 * i + 3);
      filtered_pose_msg.pose.orientation.x = y_filtered_(7 * i + 4);
      filtered_pose_msg.pose.orientation.y = y_filtered_(7 * i + 5);
      filtered_pose_msg.pose.orientation.z = y_filtered_(7 * i + 6);
      filtered_pose_publishers_[i]->publish(filtered_pose_msg);
    }
    for (int i = 0; i < num_frames_; i++)
    {
      filtered_pose_msg.header.stamp = time_stamp;
      filtered_pose_msg.header.frame_id = measure_2_base_frame;
      filtered_pose_msg.pose.position.x = y_filtered_(7 * (i + num_frames_));
      filtered_pose_msg.pose.position.y = y_filtered_(7 * (i + num_frames_) + 1);
      filtered_pose_msg.pose.position.z = y_filtered_(7 * (i + num_frames_) + 2);
      filtered_pose_msg.pose.orientation.w = y_filtered_(7 * (i + num_frames_) + 3);
      filtered_pose_msg.pose.orientation.x = y_filtered_(7 * (i + num_frames_) + 4);
      filtered_pose_msg.pose.orientation.y = y_filtered_(7 * (i + num_frames_) + 5);
      filtered_pose_msg.pose.orientation.z = y_filtered_(7 * (i + num_frames_) + 6);
      filtered_pose_publishers_[i + num_frames_]->publish(filtered_pose_msg);
    }

    // publish the object pose
    geometry_msgs::msg::PoseStamped object_pose_msg;
    object_pose_msg.header.stamp = time_stamp;
    object_pose_msg.header.frame_id = this->base_frame_name;
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
    object_twist_msg.header.stamp = time_stamp;
    object_twist_msg.header.frame_id = this->base_frame_name;
    object_twist_msg.twist.linear.x = x_hat_k_k(7);
    object_twist_msg.twist.linear.y = x_hat_k_k(8);
    object_twist_msg.twist.linear.z = x_hat_k_k(9);
    object_twist_msg.twist.angular.x = x_hat_k_k(10);
    object_twist_msg.twist.angular.y = x_hat_k_k(11);
    object_twist_msg.twist.angular.z = x_hat_k_k(12);
    object_twist_publisher_->publish(object_twist_msg);

    // publish the robot fkine
    geometry_msgs::msg::PoseStamped robot1_fkine_msg;
    robot1_fkine_msg.header.stamp = time_stamp;
    robot1_fkine_msg.header.frame_id = this->robot_1_prefix_;
    robot1_fkine_msg.pose.position.x = x_hat_k_k(13);
    robot1_fkine_msg.pose.position.y = x_hat_k_k(14);
    robot1_fkine_msg.pose.position.z = x_hat_k_k(15);
    robot1_fkine_msg.pose.orientation.w = x_hat_k_k(16);
    robot1_fkine_msg.pose.orientation.x = x_hat_k_k(17);
    robot1_fkine_msg.pose.orientation.y = x_hat_k_k(18);
    robot1_fkine_msg.pose.orientation.z = x_hat_k_k(19);
    fkine_robot1_pub_->publish(robot1_fkine_msg);

    geometry_msgs::msg::PoseStamped robot2_fkine_msg;
    robot2_fkine_msg.header.stamp = time_stamp;
    robot2_fkine_msg.header.frame_id = this->robot_2_prefix_;
    robot2_fkine_msg.pose.position.x = x_hat_k_k(20);
    robot2_fkine_msg.pose.position.y = x_hat_k_k(21);
    robot2_fkine_msg.pose.position.z = x_hat_k_k(22);
    robot2_fkine_msg.pose.orientation.w = x_hat_k_k(23);
    robot2_fkine_msg.pose.orientation.x = x_hat_k_k(24);
    robot2_fkine_msg.pose.orientation.y = x_hat_k_k(25);
    robot2_fkine_msg.pose.orientation.z = x_hat_k_k(26);
    fkine_robot2_pub_->publish(robot2_fkine_msg);

    // publish the transform b2Tb1
    geometry_msgs::msg::PoseStamped b2Tb1_msg;
    b2Tb1_msg.header.stamp = time_stamp;
    b2Tb1_msg.pose.position.x = x_hat_k_k(27);
    b2Tb1_msg.pose.position.y = x_hat_k_k(28);
    b2Tb1_msg.pose.position.z = x_hat_k_k(29);
    b2Tb1_msg.pose.orientation.w = x_hat_k_k(30);
    b2Tb1_msg.pose.orientation.x = x_hat_k_k(31);
    b2Tb1_msg.pose.orientation.y = x_hat_k_k(32);
    b2Tb1_msg.pose.orientation.z = x_hat_k_k(33);
    transform_b2Tb1_publisher_->publish(b2Tb1_msg);

    {
      // publish the transform b1Tb2
      Eigen::Matrix<double, 4, 4> b1Tb2_filtered;
      Eigen::Matrix<double, 4, 4> b2Tb1_filtered;
      uclv::geometry_helper::pose_to_matrix(x_hat_k_k.block<7, 1>(27, 0), b2Tb1_filtered);

      geometry_msgs::msg::PoseStamped transform_b1Tb2_msg;
      transform_b1Tb2_msg.header.stamp = time_stamp;
      transform_b1Tb2_msg.header.frame_id = this->measure_1_base_frame;

      b1Tb2_filtered = b2Tb1_filtered.inverse();
      transform_b1Tb2_msg.pose.position.x = b1Tb2_filtered(0, 3);
      transform_b1Tb2_msg.pose.position.y = b1Tb2_filtered(1, 3);
      transform_b1Tb2_msg.pose.position.z = b1Tb2_filtered(2, 3);
      Eigen::Quaterniond q(b1Tb2_filtered.block<3, 3>(0, 0));
      q.normalize();
      transform_b1Tb2_msg.pose.orientation.w = q.w();
      transform_b1Tb2_msg.pose.orientation.x = q.x();
      transform_b1Tb2_msg.pose.orientation.y = q.y();
      transform_b1Tb2_msg.pose.orientation.z = q.z();
      transform_b1Tb2_publisher_->publish(transform_b1Tb2_msg);
    }

    {
      int position_wrench1_output_vector = num_frames_ * 2 * 7;
      int position_wrench2_output_vector = num_frames_ * 2 * 7 + 6;

      // publish filtered wrenches
      geometry_msgs::msg::WrenchStamped filtered_wrench_msg;
      filtered_wrench_msg.header.stamp = time_stamp;
      filtered_wrench_msg.wrench.force.x = y_filtered_(position_wrench1_output_vector);
      filtered_wrench_msg.wrench.force.y = y_filtered_(position_wrench1_output_vector + 1);
      filtered_wrench_msg.wrench.force.z = y_filtered_(position_wrench1_output_vector + 2);
      filtered_wrench_msg.wrench.torque.x = y_filtered_(position_wrench1_output_vector + 3);
      filtered_wrench_msg.wrench.torque.y = y_filtered_(position_wrench1_output_vector + 4);
      filtered_wrench_msg.wrench.torque.z = y_filtered_(position_wrench1_output_vector + 5);
      filtered_wrench_1_publisher_->publish(filtered_wrench_msg);

      filtered_wrench_msg.header.stamp = time_stamp;
      filtered_wrench_msg.wrench.force.x = y_filtered_(position_wrench2_output_vector);
      filtered_wrench_msg.wrench.force.y = y_filtered_(position_wrench2_output_vector + 1);
      filtered_wrench_msg.wrench.force.z = y_filtered_(position_wrench2_output_vector + 2);
      filtered_wrench_msg.wrench.torque.x = y_filtered_(position_wrench2_output_vector + 3);
      filtered_wrench_msg.wrench.torque.y = y_filtered_(position_wrench2_output_vector + 4);
      filtered_wrench_msg.wrench.torque.z = y_filtered_(position_wrench2_output_vector + 5);
      filtered_wrench_2_publisher_->publish(filtered_wrench_msg);
    }

    {
      // publish debug message
      std_msgs::msg::Float64MultiArray debug_msg;
      debug_msg.data.resize(dim_state + y_.size() + u_.size());
      for (int i = 0; i < dim_state; i++)
      {
        debug_msg.data[i] = x_hat_k_k(i);
      }
      for (int i = 0; i < y_.size(); i++)
      {
        debug_msg.data[dim_state + i] = y_(i);
      }
      for (int i = 0; i < u_.size(); i++)
      {
        debug_msg.data[dim_state + y_.size() + i] = u_(i);
      }
      debug_publisher_->publish(debug_msg);
    }
  }

  void save_initial_state(const std::shared_ptr<dual_arm_control_interfaces::srv::EKFService::Request> request,
                          Eigen::Matrix<double, dim_state, 1>& x0)
  {
    x0.block<7, 1>(0, 0) << request->object_pose.pose.position.x, request->object_pose.pose.position.y,
        request->object_pose.pose.position.z, request->object_pose.pose.orientation.w,
        request->object_pose.pose.orientation.x, request->object_pose.pose.orientation.y,
        request->object_pose.pose.orientation.z;

    x0.block<6, 1>(7, 0) << request->object_twist.twist.linear.x, request->object_twist.twist.linear.y,
        request->object_twist.twist.linear.z, request->object_twist.twist.angular.x,
        request->object_twist.twist.angular.y, request->object_twist.twist.angular.z;

    x0.block<7, 1>(13, 0) << request->fkine_robot1.pose.position.x, request->fkine_robot1.pose.position.y,
        request->fkine_robot1.pose.position.z, request->fkine_robot1.pose.orientation.w,
        request->fkine_robot1.pose.orientation.x, request->fkine_robot1.pose.orientation.y,
        request->fkine_robot1.pose.orientation.z;

    x0.block<7, 1>(20, 0) << request->fkine_robot2.pose.position.x, request->fkine_robot2.pose.position.y,
        request->fkine_robot2.pose.position.z, request->fkine_robot2.pose.orientation.w,
        request->fkine_robot2.pose.orientation.x, request->fkine_robot2.pose.orientation.y,
        request->fkine_robot2.pose.orientation.z;

    x0.block<7, 1>(27, 0) << request->robots_relative_transform.pose.position.x,
        request->robots_relative_transform.pose.position.y, request->robots_relative_transform.pose.position.z,
        request->robots_relative_transform.pose.orientation.w, request->robots_relative_transform.pose.orientation.x,
        request->robots_relative_transform.pose.orientation.y, request->robots_relative_transform.pose.orientation.z;

    std::cout << "Initial State: \n" << x0.transpose() << std::endl;
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
    bTb1_ = bTb1;

    YAML::Node object_node = config[object_name];

    // read viscous friction
    Eigen::Matrix<double, 6, 6> viscous_friction_matrix;
    read_matrix_6x6(config, viscous_friction_matrix, "viscous_friction");

    // read spring stiffness K_1
    read_matrix_6x6(config, K_1_matrix_, "K_1");

    // read spring stiffness K_2
    read_matrix_6x6(config, K_2_matrix_, "K_2");

    // read spring damping B_1
    read_matrix_6x6(config, B_1_matrix_, "B_1");

    // read spring damping B_2
    read_matrix_6x6(config, B_2_matrix_, "B_2");

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
    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);
    int num_frames = frame_names.size();
    int index = 0;
    for (const auto& frame_name : frame_names)
    {
      pose_subscribers_.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/" + this->robot_1_prefix_ + "/" + object_name + "/" + frame_name + "/pose", qos,
          [this, index](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->pose_callback(msg, index); }));

      pose_subscribers_.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/" + this->robot_2_prefix_ + "/" + object_name + "/" + frame_name + "/pose", qos,
          [this, num_frames, index](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            this->pose_callback(msg, num_frames + index);
          }));

      index++;
    }
    num_frames_ = num_frames;

    // create the system
    robots_object_system_ptr_ = std::make_shared<uclv::systems::RobotsSpringObjectSystem>(
        x0_.block<27, 1>(0, 0), Bm, bg, oTg1, oTg2, b2Tb1, bTb1, viscous_friction_matrix, 0 * K_1_matrix_,
        0 * B_1_matrix_, 0 * K_2_matrix_, 0 * B_2_matrix_, num_frames_);

    // create the extended system
    robots_object_system_ext_ptr_ =
        std::make_shared<uclv::systems::RobotsSpringObjectSystemExt>(x0_, robots_object_system_ptr_);

    // resize the output variable
    y_.resize(num_frames_ * 14 + 12 + 14, 1);
    y_.setZero();

    y_filtered_.resize(num_frames_ * 14 + 12 + 14, 1);
    y_filtered_.setZero();

    pose_measure_received_.resize(num_frames_ * 2);
    pose_measure_received_.setZero();
    force_measure_received_.setZero();
    fkine_measure_received_.setZero();

    // initialize y measured with the initial pose
    robots_object_system_ext_ptr_->output_fcn(x0_, Eigen::Matrix<double, 12, 1>::Zero(), y_);

    std::cout << "intial state x0: \n" << x0_.transpose() << std::endl;
    std::cout << "Initial y: \n" << y_ << std::endl;

    // initialize filtered pose publishers
    for (int i = 0; i < num_frames_; i++)
    {
      filtered_pose_publishers_.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/ekf/" + this->robot_1_prefix_ + "/" + object_name + "/" + frame_names[i] + "/pose_filtered", 1));
    }
    for (int i = num_frames; i < 2 * num_frames_; i++)
    {
      filtered_pose_publishers_.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/ekf/" + this->robot_2_prefix_ + "/" + object_name + "/" + frame_names[i - num_frames] + "/pose_filtered",
          1));
    }
  }

  void fkine_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const int& index)
  {
    // RCLCPP_INFO(this->get_logger(), "Received fkine from %d", index);
    // if (this->object_grasped_[index])
    // {
    int position_measure_vector = num_frames_ * 2 * 7 + 12 + 7 * index;
    y_.block(position_measure_vector, 0, 7, 1) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
        msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
    fkine_measure_received_(index) = true;
    // }
  }

  void twist_fkine_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg, const int& index)
  {
    // RCLCPP_INFO(this->get_logger(), "Received fkine from %d", index);
    this->u_.block<6, 1>(index * 6, 0) << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
        msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
  }

  void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg, const int& index)
  {
    // RCLCPP_INFO(this->get_logger(), "Received wrench from %d", index);
    // if (mass_estimated_flag_ == false)
    // {
    //   this->measured_wrench_robots_mass_estimation_.block<6, 1>(index * 6, 0) << msg->wrench.force.x,
    //       msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
    //   // this->measured_wrench_robots_mass_estimation_.block<6, 1>(index * 6, 0) =
    //   //     -this->measured_wrench_robots_mass_estimation_.block<6, 1>(index * 6, 0);
    //   return;
    // }

    if (this->object_grasped_[index])
    {
      int position_measure_vector = num_frames_ * 2 * 7 + 6 * index;
      y_.block(position_measure_vector, 0, 6, 1) << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
          msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;

      // ALERT! change sign to measured force due to sign convention sensors
      // y_.block(position_measure_vector, 0, 6, 1) = -y_.block(position_measure_vector, 0, 6, 1);
      force_measure_received_(index) = true;
    }
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const int& index)
  {
    // RCLCPP_INFO(this->get_logger(), "Received pose from %d", index);
    // this->y_.block<7, 1>(index * 7, 0) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
    //     msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;

    if (measure_1_base_frame == "" && index < num_frames_)
    {
      measure_1_base_frame = msg->header.frame_id;
    }
    if (measure_2_base_frame == "" && index >= num_frames_)
    {
      measure_2_base_frame = msg->header.frame_id;
    }

    // ensure quaternion continuity
    Eigen::Matrix<double, 4, 1> q;
    q << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;

    y_filtered_ = ekf_ptr->get_output();
    uclv::geometry_helper::quaternion_continuity(q, y_filtered_.block<4, 1>(index * 7 + 3, 0), q);

    y_.block<4, 1>(index * 7 + 3, 0) = q;
    y_.block<3, 1>(index * 7, 0) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

    pose_measure_received_(index) = true;

    // store last available msg
    if (index < num_frames_)
    {
      last_pose_msg_robot1_ = msg;
    }
    else
    {
      last_pose_msg_robot2_ = msg;
    }
  }

  void check_b1Tb2_convergence(Eigen::Matrix<double, 7, 1> measure_robot1, Eigen::Matrix<double, 7, 1> measure_robot2)
  {
    Eigen::Matrix<double, dim_state, 1> x_hat = ekf_ptr->get_state();

    Eigen::Matrix<double, 4, 4> b2Tb1;
    b2Tb1.setIdentity();
    uclv::geometry_helper::pose_to_matrix(x_hat.block<7, 1>(27, 0), b2Tb1);

    Eigen::Matrix<double, 4, 4> b1Tb2_filtered;
    b1Tb2_filtered.setIdentity();
    b1Tb2_filtered = b2Tb1.inverse();

    Eigen::Matrix<double, 4, 4> bTb1;
    bTb1.setIdentity();
    bTb1 << robots_object_system_ptr_->bTb1_;

    Eigen::Matrix<double, 4, 4> b2To_measure2;
    b2To_measure2.setIdentity();
    uclv::geometry_helper::pose_to_matrix(measure_robot2, b2To_measure2);

    Eigen::Matrix<double, 4, 4> bTo_frommeasure2 = bTb1 * b1Tb2_filtered * b2To_measure2;

    Eigen::Matrix<double, 4, 4> b1To_measure1;
    b1To_measure1.setIdentity();
    uclv::geometry_helper::pose_to_matrix(measure_robot1, b1To_measure1);
    Eigen::Matrix<double, 4, 4> bTo_frommeasure1 = bTb1 * b1To_measure1;

    // evaluate error
    Eigen::Matrix<double, 4, 4> error = bTo_frommeasure1.inverse() * bTo_frommeasure2;

    // std::cout << "Error matrix: \n" << error << std::endl;
    // std::cout << "bTo_frommeasure1 matrix: \n" << bTo_frommeasure1 << std::endl;
    // std::cout << "bTo_frommeasure2 matrix: \n" << bTo_frommeasure2 << std::endl;

    // Compute distance of error matrix from identity
    std::cout << "Error postion norm: " << error.block<3, 1>(0, 3).norm() << std::endl;

    // compute angle axis of the error matrix
    Eigen::AngleAxisd error_angle_axis(error.block<3, 3>(0, 0));
    std::cout << "Error angle axis: " << error_angle_axis.angle() << std::endl;
    // std::cout << "Error axis: " << error_angle_axis.axis().transpose() << std::endl;

    if (error.block<3, 1>(0, 3).norm() < 0.007 && error_angle_axis.angle() < 0.01)
    {
      // std::cout << "b1Tb2 converged" << std::endl;
      W_.block<7, 7>(27, 27) = W_default_.block<7, 7>(27, 27) * 0.0;
      ekf_ptr->setP(W_);
      // std::cout << "W_ updated: \n" << W_.block<7, 7>(13, 13) << std::endl;
      b1Tb2_convergence_status_ = true;
    }
    else
    {
      // W_.block<7, 7>(13, 13) = W_default_.block<7, 7>(13, 13);
    }
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

  void handle_grasp_service_request(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (filter_initialized_ == false)
    {
      response->success = false;
      response->message = "EKF not initialized";
      return;
    }
    if (request->data)  // object grasped
    {
      object_grasped_[0] = true;
      object_grasped_[1] = true;

      std::cout << "Object grasped" << std::endl;
      // Update grasping frames oTg1 and oTg2
      Eigen::Matrix<double, 34, 1> x_state = ekf_ptr->get_state();
      Eigen::Matrix<double, 4, 4> bTo;
      bTo.setIdentity();
      uclv::geometry_helper::pose_to_matrix(x_state.block<7, 1>(0, 0), bTo);
      std::cout << "bTo: \n" << bTo << std::endl;

      Eigen::Matrix<double, 4, 4> b2Tb1;
      b2Tb1.setIdentity();
      uclv::geometry_helper::pose_to_matrix(x_state.block<7, 1>(27, 0), b2Tb1);
      std::cout << "b2Tb1: \n" << b2Tb1 << std::endl;

      Eigen::Matrix<double, 4, 4> b1Te1;
      b1Te1.setIdentity();
      uclv::geometry_helper::pose_to_matrix(x_state.block<7, 1>(13, 0), b1Te1);
      std::cout << "b1Te1: \n" << b1Te1 << std::endl;

      Eigen::Matrix<double, 4, 4> b2Te2;
      b2Te2.setIdentity();
      uclv::geometry_helper::pose_to_matrix(x_state.block<7, 1>(20, 0), b2Te2);
      std::cout << "b2Te2: \n" << b2Te2 << std::endl;

      std::cout << "bTb1_: \n" << bTb1_ << std::endl;

      // set the new oTg1 and oTg2
      Eigen::Matrix<double, 4, 4> oTg1;
      oTg1.setIdentity();
      oTg1 = bTo.inverse() * bTb1_ * b1Te1;
      std::cout << "oTg1: \n" << oTg1 << std::endl;

      Eigen::Matrix<double, 4, 4> oTg2;
      oTg2.setIdentity();
      oTg2 = bTo.inverse() * bTb1_ * b2Tb1.inverse() * b2Te2;
      std::cout << "oTg2: \n" << oTg2 << std::endl;

      robots_object_system_ptr_->set_oTg1(oTg1);
      robots_object_system_ptr_->set_oTg2(oTg2);

      // enable contact in the model
      robots_object_system_ptr_->set_K_1(K_1_matrix_);
      robots_object_system_ptr_->set_B_1(B_1_matrix_);
      robots_object_system_ptr_->set_K_2(K_2_matrix_);
      robots_object_system_ptr_->set_B_2(B_2_matrix_);

      int position_measure_vector = num_frames_ * 2 * 7 + 12;

      y_.block(position_measure_vector, 0, 14, 1) = ekf_ptr->get_output().block(position_measure_vector, 0, 14, 1);

      robots_object_system_ptr_->set_gravity(Eigen::Matrix<double, 3, 1>(0.0, 0.0, -9.81));

      // if (this->mass_estimated_flag_ == false)
      // {
      //   timer_mass_estimation_ = this->create_wall_timer(std::chrono::milliseconds(10),
      //                                                    std::bind(&EKFServer::mass_estimation_callback, this));
      // }
    }
    else
    {
      object_grasped_[0] = false;
      object_grasped_[1] = false;
      robots_object_system_ptr_->set_K_1(Eigen::Matrix<double, 6, 6>::Zero());
      robots_object_system_ptr_->set_B_1(Eigen::Matrix<double, 6, 6>::Zero());
      robots_object_system_ptr_->set_K_2(Eigen::Matrix<double, 6, 6>::Zero());
      robots_object_system_ptr_->set_B_2(Eigen::Matrix<double, 6, 6>::Zero());
      std::cout << "Object released" << std::endl;
    }

    response->success = true;
    response->message = request->data ? "EKF object grasped True" : "EKF object grasped False";
  }

  void mass_estimation_callback()
  {
    if (mass_estimated_counter_ == 0)
      external_wrenches_.resize(6, mass_estimation_samples_);

    std::cout << "MASS ESIMATION CB " << mass_estimated_counter_ << std::endl;
    Eigen::Matrix<double, 6, 1> external_wrench;
    robots_object_system_ptr_->get_object_wrench(measured_wrench_robots_mass_estimation_, external_wrench);

    external_wrenches_.block<6, 1>(0, mass_estimated_counter_) << external_wrench;

    mass_estimated_counter_++;
    if (mass_estimated_counter_ == mass_estimation_samples_)
    {
      mass_estimated_flag_ = true;
      double norm_force = 0.0;
      Eigen::Vector3d medium_force;
      medium_force.setZero();
      for (int i = 0; i < mass_estimation_samples_; i++)
      {
        Eigen::Vector3d force = external_wrenches_.block<3, 1>(0, i);
        norm_force = norm_force + force.norm();
        medium_force = medium_force + force;
      }
      medium_force = medium_force / mass_estimation_samples_;
      mass_estimated_ = (norm_force / mass_estimation_samples_) / 9.81;

      RCLCPP_INFO(this->get_logger(), "Mass estimated: %f", mass_estimated_);
      timer_mass_estimation_->cancel();

      // set the mass in the system
      // robots_object_system_ptr_->set_mass(mass_estimated_);
      // robots_object_system_ptr_->set_gravity(Eigen::Matrix<double, 3, 1>(0.0, 0.0, 9.81));

      // estimate bias external wrench
      Eigen::Matrix<double, 6, 1> bias_external_wrench;
      bias_external_wrench.setZero();
      for (int i = 0; i < mass_estimation_samples_; i++)
      {
        bias_external_wrench = bias_external_wrench + external_wrenches_.block<6, 1>(0, i);  //-
        // Eigen::Matrix<double, 6, 1>(medium_force(0), medium_force(1), medium_force(2), 0.0, 0.0, 0.0);
      }
      bias_external_wrench = bias_external_wrench / mass_estimation_samples_;
      // robots_object_system_ptr_->set_ho_bias(bias_external_wrench);
      std::cout << "medium_force: \n" << medium_force << std::endl;
      std::cout << "Bias external wrench: \n" << bias_external_wrench << std::endl;

      // enable contact in the model
      robots_object_system_ptr_->set_K_1(K_1_matrix_);
      robots_object_system_ptr_->set_B_1(B_1_matrix_);
      robots_object_system_ptr_->set_K_2(K_2_matrix_);
      robots_object_system_ptr_->set_B_2(B_2_matrix_);
    }
  }

  rclcpp::Service<dual_arm_control_interfaces::srv::EKFService>::SharedPtr server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr server_object_grasped_;

  // subscribers to poseStamped
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subscribers_;

  // subscribers to WrenchStamped
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_robot1_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_robot2_sub_;

  // subscribers to fkine
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr fkine_robot1_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr fkine_robot2_sub_;

  // twist subscribers
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_robot1_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_robot2_sub_;

  // timer for ekf update
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_publish_;
  rclcpp::TimerBase::SharedPtr timer_mass_estimation_;

  // publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr object_twist_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr transform_b2Tb1_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr transform_b1Tb2_publisher_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> filtered_pose_publishers_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr filtered_wrench_1_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr filtered_wrench_2_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fkine_robot1_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fkine_robot2_pub_;

  // strings to attach at the topic name to subscribe
  std::string robot_1_prefix_;
  std::string robot_2_prefix_;

  // define systems for EKF
  uclv::systems::RobotsSpringObjectSystem::SharedPtr robots_object_system_ptr_;
  uclv::systems::RobotsSpringObjectSystemExt::SharedPtr robots_object_system_ext_ptr_;
  uclv::systems::ForwardEuler<dim_state, 12, Eigen::Dynamic>::SharedPtr discretized_system_ptr_;
  uclv::systems::ExtendedKalmanFilter<dim_state, 12, Eigen::Dynamic>::SharedPtr ekf_ptr;

  Eigen::Matrix<double, dim_state, 1> x0_;        // filter initial state
  Eigen::Matrix<double, dim_state, 1> x_hat_k_k;  // filter state
  Eigen::Matrix<double, Eigen::Dynamic, 1> y_;  // variable to store the measures for the filter [aruco poses, whrences,
                                                // fkines]
  Eigen::Matrix<double, Eigen::Dynamic, 1> y_filtered_;      // variable to store the measures filtered
  Eigen::Matrix<double, 12, 1> u_;                           // variable to store the robots twist input
  Eigen::Matrix<double, dim_state, dim_state> W_;            // process noise covariance matrix
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V_;  // measurement noise covariance matrix
  Eigen::Matrix<double, dim_state, dim_state> W_default_;    // default covariance matrix for the state
  Eigen::Matrix<double, 7, 7> V_single_measure_;             // covariance matrix for the single object pose measure
  Eigen::Matrix<double, 12, 12> V_forces_;                   // covariance matrix for the forces and torques
  Eigen::Matrix<double, 14, 14> V_fkine_robots_;             // covariance matrix for the fkine measures

  bool filter_initialized_ = false;  // flag to check if the filter has been initialized

  double sample_time_;             // sample time for the filter
  double publishing_sample_time_;  // sample time for the publishing
  int num_frames_;                 // number of frames measuring the object

  // variables for occlusion handling
  Eigen::Vector<bool, Eigen::Dynamic> pose_measure_received_;  // this vector indicates if a measure has been received
  Eigen::Vector<bool, 2> force_measure_received_;  // this vector indicates if a force measure has been received
  Eigen::Vector<bool, 2> fkine_measure_received_;  // this vector indicates if a fkine measure has been received

  std::string base_frame_name;  // base frame name
  std::string measure_1_base_frame;
  std::string measure_2_base_frame;

  bool b1Tb2_convergence_status_ = false;

  // mass estimation variables
  bool object_grasped_[2] = { false, false };  // use force measures only if the object is grasped
  bool mass_estimated_flag_ = false;
  double mass_estimated_ = 0.0;
  int mass_estimated_counter_ = 0.0;
  int mass_estimation_samples_ = 100;
  Eigen::Matrix<double, 12, 1> measured_wrench_robots_mass_estimation_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> external_wrenches_;

  // last available aruco msg
  geometry_msgs::msg::PoseStamped::SharedPtr last_pose_msg_robot1_;
  geometry_msgs::msg::PoseStamped::SharedPtr last_pose_msg_robot2_;

  // store spring parameters
  Eigen::Matrix<double, 6, 6> K_1_matrix_;
  Eigen::Matrix<double, 6, 6> B_1_matrix_;
  Eigen::Matrix<double, 6, 6> B_2_matrix_;
  Eigen::Matrix<double, 6, 6> K_2_matrix_;
  Eigen::Matrix<double, 4, 4> bTb1_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFServer>());
  rclcpp::shutdown();
  return 0;
}