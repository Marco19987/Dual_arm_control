#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <eigen3/Eigen/Geometry>
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_helper.hpp"

using namespace Eigen;

class SpringModelNode : public rclcpp::Node
{
public:
  SpringModelNode() : Node("spring_model_node"), sample_time_(0.01)
  {
    // Initialize ROS2 node
    this->declare_parameter<double>("sample_time", sample_time_);
    this->get_parameter("sample_time", sample_time_);

    timer_ = this->create_wall_timer(std::chrono::duration<double>(sample_time_),
                                     std::bind(&SpringModelNode::timer_callback, this));

    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);
    state_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("object_state", qos);
    wrench1_publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench1", qos);
    wrench2_publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench2", qos);
    twist1_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist1", qos);
    twist2_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist2", qos);

    // subscriber robot1 pose
    sub_robot1_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "robot1_pose", qos, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          Eigen::Vector<double, 7> bx1_old = bx1_;
          bx1_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->pose.orientation.w,
              msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
          fkine_robot1_read = true;
          compute_end_effector_twist(bx1_old, bx1_, bx1_dot_);
          // publish twist
          geometry_msgs::msg::TwistStamped twist_msg;
          twist_msg.header.stamp = this->now();
          twist_msg.twist.linear.x = bx1_dot_(0);
          twist_msg.twist.linear.y = bx1_dot_(1);
          twist_msg.twist.linear.z = bx1_dot_(2);
          twist_msg.twist.angular.x = bx1_dot_(3);
          twist_msg.twist.angular.y = bx1_dot_(4);
          twist_msg.twist.angular.z = bx1_dot_(5);
          twist1_publisher_->publish(twist_msg);
        });

    // subscriber robot2 pose
    sub_robot2_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "robot2_pose", qos, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          Eigen::Vector<double, 7> bx2_old = bx2_;
          bx2_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->pose.orientation.w,
              msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
          fkine_robot2_read = true;
          compute_end_effector_twist(bx2_old, bx2_, bx2_dot_);
          // publish twist
          geometry_msgs::msg::TwistStamped twist_msg;
          twist_msg.header.stamp = this->now();
          twist_msg.twist.linear.x = bx2_dot_(0);
          twist_msg.twist.linear.y = bx2_dot_(1);
          twist_msg.twist.linear.z = bx2_dot_(2);
          twist_msg.twist.angular.x = bx2_dot_(3);
          twist_msg.twist.angular.y = bx2_dot_(4);
          twist_msg.twist.angular.z = bx2_dot_(5);
          twist2_publisher_->publish(twist_msg);
        });

    // create service
    service_ = this->create_service<std_srvs::srv::SetBool>(
        "activate_model_simulator",
        std::bind(&SpringModelNode::activateSimulationCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize state vector x (object pose and twist, orientation represented as quaternion qw qx qy qz)
    x_ << 0.9, 0.0, 0.32, 0.0, 0.0, -0.7071068, 0.7071068, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    simulation_active_ = false;
  }

private:
  void compute_grasp_frames()
  {
    Eigen::Matrix<double, 4, 4> bTo;
    uclv::geometry_helper::pose_to_matrix(x_.block<7, 1>(0, 0), bTo);

    Eigen::Matrix<double, 4, 4> bTfkine1;
    Eigen::Matrix<double, 4, 4> bTfkine2;

    uclv::geometry_helper::pose_to_matrix(bx1_, bTfkine1);
    uclv::geometry_helper::pose_to_matrix(bx2_, bTfkine2);

    Eigen::Matrix<double, 4, 4> oTfkine1 = bTo.inverse() * bTfkine1;
    Eigen::Matrix<double, 4, 4> oTfkine2 = bTo.inverse() * bTfkine2;
    // Define grasp frames
    opg1 << oTfkine1.block<3, 1>(0, 3);
    opg2 << oTfkine2.block<3, 1>(0, 3);
    oRg1 << oTfkine1.block<3, 3>(0, 0);
    oRg2 << oTfkine2.block<3, 3>(0, 0);

    std::cout << "opg1: " << opg1.transpose() << std::endl;
    std::cout << "opg2: " << opg2.transpose() << std::endl;
    std::cout << "oRg1: " << std::endl << oRg1 << std::endl;
    std::cout << "oRg2: " << std::endl << oRg2 << std::endl;
  }

  void compute_end_effector_twist(Eigen::Vector<double, 7> x_old, Eigen::Vector<double, 7> x_new,
                                  Eigen::Vector<double, 6>& x_dot)
  {
    double measure_sample_time = (1 / 10.0);
    x_dot.segment<3>(0) = (x_new.segment<3>(0) - x_old.segment<3>(0)) / measure_sample_time;
    Eigen::Quaterniond q_old(x_old(3), x_old(4), x_old(5), x_old(6));
    Eigen::Quaterniond q_new(x_new(3), x_new(4), x_new(5), x_new(6));
    Eigen::Quaterniond relative_q = q_old.inverse() * q_new;
    Eigen::AngleAxisd relative_aa(relative_q);
    x_dot.segment<3>(3) = q_old.toRotationMatrix() * (relative_aa.angle() / measure_sample_time) * relative_aa.axis();
  }

  void activateSimulationCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data)
    {
      simulation_active_ = true;
    }
    else
    {
      simulation_active_ = false;
      grasp_frames_computed = false;
      fkine_robot1_read = false;
      fkine_robot2_read = false;
      x_ << 0.9, 0.0, 0.32, 0.0, 0.0, -0.7071068, 0.7071068, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    }
    response->success = true;
    response->message = request->data ? "Simulation activated" : "Simulation deactivated";
  }

  void timer_callback()
  {
    if (simulation_active_ && fkine_robot1_read && fkine_robot2_read)
    {
      if (!grasp_frames_computed)
      {
        compute_grasp_frames();
        grasp_frames_computed = true;
      }

      // Define inputs and parameters for spring_model
      Matrix3d K_1 = Matrix3d::Identity() * 5000;         // Linear stiffnes contact
      Matrix3d K_2 = Matrix3d::Identity() * 5000;         // Linear stiffnes contact
      Matrix3d B_1 = Matrix3d::Identity() * 5;            // Viscous friction contact
      Matrix3d B_2 = Matrix3d::Identity() * 5;            // Viscous friction contact
      MatrixXd Bm = MatrixXd::Identity(6, 6) * 10;        // Object Inertia Matrix
      Bm.block<3, 3>(3, 3) = Matrix3d::Identity() * 0.1;  // Object Inertia Matrix
      VectorXd bg = VectorXd::Zero(3);                    // gravity vector direction
      bg(2) = -0;
      std::string eul_choice = "ZYX";  // euler angle convention for angle error computetion in torsional spring

      VectorXd xdot(13);
      VectorXd h(12);


      double tk = 0.0;
      VectorXd x_obj_micro = x_;
      double micro_step = 0.0001;
      while (tk < sample_time_)
      {
        tk += micro_step;
        auto [xdot_micro, h_micro] = spring_model(x_obj_micro, bx1_, bx2_, bx1_dot_, bx2_dot_, K_1, K_2, B_1, B_2, Bm, bg,
                                      eul_choice, opg1, opg2, oRg1, oRg2);
        xdot = xdot_micro;
        h = h_micro;
        x_obj_micro += micro_step * xdot;
        Eigen::Quaterniond q(x_obj_micro(3), x_obj_micro(4), x_obj_micro(5), x_obj_micro(6));
        q.normalize();
        x_obj_micro.segment<4>(3) << q.w(), q.x(), q.y(), q.z();

      }
      x_ = x_obj_micro;

      // Integrate xdot to update state x
      // x_ += xdot * sample_time_;

      // Eigen::Quaterniond q(x_(3), x_(4), x_(5), x_(6));
      // q.normalize();
      // x_.segment<4>(3) << q.w(), q.x(), q.y(), q.z();

      // Publish wrenches
      // ALERT !!!! change sign wrenches due to measure sign convention
      h = -h;
      geometry_msgs::msg::WrenchStamped wrench1_msg;
      wrench1_msg.header.stamp = this->now();
      wrench1_msg.wrench.force.x = h(0);
      wrench1_msg.wrench.force.y = h(1);
      wrench1_msg.wrench.force.z = h(2);
      wrench1_msg.wrench.torque.x = h(3);
      wrench1_msg.wrench.torque.y = h(4);
      wrench1_msg.wrench.torque.z = h(5);
      wrench1_publisher_->publish(wrench1_msg);

      geometry_msgs::msg::WrenchStamped wrench2_msg;
      wrench2_msg.header.stamp = this->now();
      wrench2_msg.wrench.force.x = h(6);
      wrench2_msg.wrench.force.y = h(7);
      wrench2_msg.wrench.force.z = h(8);
      wrench2_msg.wrench.torque.x = h(9);
      wrench2_msg.wrench.torque.y = h(10);
      wrench2_msg.wrench.torque.z = h(11);
      wrench2_publisher_->publish(wrench2_msg);
    }

    // Publish state as PoseStamped
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "world";
    pose_msg.pose.position.x = x_(0);
    pose_msg.pose.position.y = x_(1);
    pose_msg.pose.position.z = x_(2);
    pose_msg.pose.orientation.w = x_(3);
    pose_msg.pose.orientation.x = x_(4);
    pose_msg.pose.orientation.y = x_(5);
    pose_msg.pose.orientation.z = x_(6);
    state_publisher_->publish(pose_msg);
  }

  std::pair<VectorXd, MatrixXd> spring_model(const VectorXd& x, const VectorXd& bx1, const VectorXd& bx2,
                                             const VectorXd& bx1_dot, const VectorXd& bx2_dot, const Matrix3d& K_1,
                                             const Matrix3d& K_2, const Matrix3d& B_1, const Matrix3d& B_2,
                                             const MatrixXd& Bm, const VectorXd& bg, const std::string& eul_choice,
                                             const Vector3d& opg1, const Vector3d& opg2, const Matrix3d& oRg1,
                                             const Matrix3d& oRg2)
  {
    Vector3d bpo = x.segment<3>(0);
    Matrix3d bRo = quat2rotm(x.segment<4>(3).transpose());
    Vector3d bp1 = bx1.segment<3>(0);
    Vector3d bp2 = bx2.segment<3>(0);
    Vector3d bpg1 = bpo + bRo * opg1;
    Vector3d bpg2 = bpo + bRo * opg2;

    Vector3d bpo_dot = x.segment<3>(7);
    Vector3d bomegao = x.segment<3>(10);
    Vector3d bp1_dot = bx1_dot.segment<3>(0);
    Vector3d bp2_dot = bx2_dot.segment<3>(0);
    Vector3d bomega1_dot = bx1_dot.segment<3>(3);
    Vector3d bomega2_dot = bx2_dot.segment<3>(3);

    Vector3d bpg1_dot = bpo_dot + skew(bomegao) * bRo * opg1;
    Vector3d bpg2_dot = bpo_dot + skew(bomegao) * bRo * opg2;

    Matrix3d bR1 = quat2rotm(bx1.segment<4>(3).transpose());
    Matrix3d bR2 = quat2rotm(bx2.segment<4>(3).transpose());

    // std::cout << "bpo: " << bpo.transpose() << std::endl;
    // std::cout << "bp1: " << bp1.transpose() << std::endl;
    // std::cout << "bp2: " << bp2.transpose() << std::endl;
    // std::cout << "bpg1: " << bpg1.transpose() << std::endl;
    // std::cout << "bpg2: " << bpg2.transpose() << std::endl;
    // std::cout << "bRo: " << std::endl << bRo << std::endl;
    // std::cout << "bR1: " << std::endl << bR1 << std::endl;
    // std::cout << "bR2: " << std::endl << bR2 << std::endl;
    // std::cout << "bp1dot: " << bp1_dot.transpose() << std::endl;
    // std::cout << "bp2dot: " << bp2_dot.transpose() << std::endl;
    // std::cout << "bpg1_dot: " << bpg1_dot.transpose() << std::endl;
    // std::cout << "bpg2_dot: " << bpg2_dot.transpose() << std::endl;
    // std::cout << "bomegao: " << bomegao.transpose() << std::endl;
    // std::cout << "bomega1_dot: " << bomega1_dot.transpose() << std::endl;
    // std::cout << "bomega2_dot: " << bomega2_dot.transpose() << std::endl;

    // wrench grasp frame 1 expressed in g1 : g1hg1
    Matrix3d g1Rb = oRg1.transpose() * bRo.transpose();
    Vector3d g1fe_1 = K_1 * g1Rb * (-bpg1 + bp1);
    Vector3d g1fbeta_1 = B_1 * g1Rb * (-bpg1_dot + bp1_dot);
    // Vector3d g1taue_1 = K_1 * rotm2eul(g1Rb * bR1, eul_choice);
    Vector3d g1taue_1 = 0.5 * K_1 * compute_torsional_error(g1Rb * bR1);

    Vector3d g1tau_beta_1 = 0.1 * B_1 * g1Rb * (-bomegao + bomega1_dot);
    VectorXd g1hg1(6);
    g1hg1 << 1 * g1fe_1 + 1 * g1fbeta_1, 1 * g1taue_1 + 1 * g1tau_beta_1;

    // wrench grasp frame 2 expressed in g2 : g2hg2
    Matrix3d g2Rb = oRg2.transpose() * bRo.transpose();
    Vector3d g2fe_2 = K_2 * g2Rb * (-bpg2 + bp2);
    Vector3d g2fbeta_2 = B_2 * g2Rb * (-bpg2_dot + bp2_dot);
    // Vector3d g2taue_2 = K_2 * rotm2eul(g2Rb * bR2, eul_choice);
    Vector3d g2taue_2 = 0.5 * K_2 * compute_torsional_error(g2Rb * bR2);
    Vector3d g2tau_beta_2 = 0.1 * B_2 * g2Rb * (-bomegao + bomega2_dot);
    VectorXd g2hg2(6);
    g2hg2 << 1 * g2fe_2 + 1 * g2fbeta_2, 1 * g2taue_2 + 1 * g2tau_beta_2;

    // compute grasp matrices and Rbar matrix
    MatrixXd Wg1(6, 6);
    Wg1 << Matrix3d::Identity(), Matrix3d::Zero(), -skew(opg1).transpose(), Matrix3d::Identity();
    MatrixXd Wg2(6, 6);
    Wg2 << Matrix3d::Identity(), Matrix3d::Zero(), -skew(opg2).transpose(), Matrix3d::Identity();
    MatrixXd W(6, 12);
    W << Wg1, Wg2;

    MatrixXd Rbar(12, 12);
    Rbar << oRg1, Matrix3d::Zero(), Matrix3d::Zero(), Matrix3d::Zero(), Matrix3d::Zero(), oRg1, Matrix3d::Zero(),
        Matrix3d::Zero(), Matrix3d::Zero(), Matrix3d::Zero(), oRg2, Matrix3d::Zero(), Matrix3d::Zero(),
        Matrix3d::Zero(), Matrix3d::Zero(), oRg2;

    // object wrench
    VectorXd oh = W * Rbar * (VectorXd(12) << g1hg1, g2hg2).finished();
    // std::cout << "oh: " << oh.transpose() << std::endl;
    // oh.segment<3>(3) << 0,0,0;

    // xdot evaluation
    VectorXd bxobj_dot(7);
    bxobj_dot << bpo_dot, 0, 0, 0, 0;
    Eigen::Quaterniond q_dot;
    uclv::geometry_helper::quaternion_propagation(Eigen::Quaterniond(x_(3), x_(4), x_(5), x_(6)), bomegao, q_dot);
    bxobj_dot.segment<4>(3) << q_dot.w(), q_dot.x(), q_dot.y(), q_dot.z();

    VectorXd bxobj_dot_dot(6);
    bxobj_dot_dot << (MatrixXd(6, 6) << bRo, Matrix3d::Zero(), Matrix3d::Zero(), bRo).finished() * (Bm.inverse() * oh) +
                         (VectorXd(6) << bg, 0, 0, 0).finished() +
                         MatrixXd::Identity(6, 6) * 0.001 * (Eigen::VectorXd(6) << bpo_dot, bomegao).finished();

    VectorXd xdot(13);
    xdot << bxobj_dot, bxobj_dot_dot;

    VectorXd h(12);
    h << g1hg1, g2hg2;

    return { xdot, h };
  }
  Vector3d compute_torsional_error(const Matrix3d& R)
  {
    Eigen::AngleAxisd relative_aa(R);
    return relative_aa.angle() * relative_aa.axis();
  }

  Matrix3d quat2rotm(const Vector4d& q)
  {
    Eigen::Quaterniond quat(q(0), q(1), q(2), q(3));
    return quat.toRotationMatrix();
  }

  Matrix3d skew(const Vector3d& v)
  {
    Matrix3d m;
    m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return m;
  }

  Vector3d rotm2eul(const Matrix3d& R, const std::string& eul_choice)
  {
    Vector3d eul;
    if (eul_choice == "ZYX")
    {
      eul(0) = atan2(R(1, 0), R(0, 0));
      eul(1) = atan2(-R(2, 0), sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
      eul(2) = atan2(R(2, 1), R(2, 2));
    }
    else if (eul_choice == "ZYZ")
    {
      eul(0) = atan2(R(2, 1), R(2, 2));
      eul(1) = atan2(sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)), R(2, 0));
      eul(2) = atan2(R(1, 0), -R(0, 0));
    }
    else
    {
      throw std::invalid_argument("Unsupported Euler angle convention");
    }
    return eul;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench1_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench2_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist1_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist2_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_robot1_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_robot2_pose_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

  double sample_time_;
  Eigen::Vector<double, 13> x_;
  Eigen::Vector<double, 7> bx1_;      // robot 1 pose
  Eigen::Vector<double, 7> bx2_;      // robot 2 pose
  Eigen::Vector<double, 6> bx1_dot_;  // robot 1 twist
  Eigen::Vector<double, 6> bx2_dot_;  // robot 2 twist

  Vector3d opg1 = Vector3d::Zero();      // grasp position robot 1 object frame
  Vector3d opg2 = Vector3d::Zero();      // grasp position robot 2 object frame
  Matrix3d oRg1 = Matrix3d::Identity();  // grasp orientation robot 1 object frame
  Matrix3d oRg2 = Matrix3d::Identity();  // grasp orientation robot 2 object frame
  bool grasp_frames_computed = false;

  bool fkine_robot1_read = false;
  bool fkine_robot2_read = false;

  bool simulation_active_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpringModelNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}