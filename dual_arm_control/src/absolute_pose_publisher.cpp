#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <eigen3/Eigen/Geometry>

class AbsolutePosePublisher : public rclcpp::Node
{
public:
  AbsolutePosePublisher() : Node("absolute_pose_publisher")
  {
    subscription1_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/robot1/fkine", rclcpp::SensorDataQoS(),
        std::bind(&AbsolutePosePublisher::poseCallback1, this, std::placeholders::_1));
    subscription2_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/robot2/fkine", rclcpp::SensorDataQoS(),
        std::bind(&AbsolutePosePublisher::poseCallback2, this, std::placeholders::_1));
    subscription3_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/robot1/fkine_camera", rclcpp::SensorDataQoS(),
        std::bind(&AbsolutePosePublisher::poseCallback3, this, std::placeholders::_1));
    subscription4_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/robot2/fkine_camera", rclcpp::SensorDataQoS(),
        std::bind(&AbsolutePosePublisher::poseCallback4, this, std::placeholders::_1));

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("absolute_pose", 1);
    publisher_fkine1_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("fkine_1", 1);
    publisher_fkine2_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("fkine_2", 1);
    publisher_fkine3_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("debug/fkine_camera_1", 1);
    publisher_fkine4_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("debug/fkine_camera_2", 1);


    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&AbsolutePosePublisher::timerCallback, this));
  }

private:
  void timerCallback()
  {
    if (pose_msg_1_ && pose_msg_2_)
    {
      RCLCPP_INFO(this->get_logger(), "Absolute pose from robot 1: [x: %f, y: %f, z: %f]", pose_msg_1_->pose.position.x,
                  pose_msg_1_->pose.position.y, pose_msg_1_->pose.position.z);
      RCLCPP_INFO(this->get_logger(), "Absolute pose from robot 2: [x: %f, y: %f, z: %f]", pose_msg_2_->pose.position.x,
                  pose_msg_2_->pose.position.y, pose_msg_2_->pose.position.z);

      // define b1Tb2
      Eigen::Matrix<double, 4, 4> b1Tb2;
      b1Tb2.setIdentity();
      b1Tb2.block<3, 1>(0, 3) << 1.6, 0, 0;
      b1Tb2.block<3, 3>(0, 0) << -1, 0, 0, 0, -1, 0, 0, 0, 1;

      // rotate pose_msg_2_ by b1Tb2
      Eigen::Quaterniond q(pose_msg_2_->pose.orientation.w, pose_msg_2_->pose.orientation.x,
                           pose_msg_2_->pose.orientation.y, pose_msg_2_->pose.orientation.z);
      Eigen::Matrix<double, 4, 4> T;
      T.setIdentity();
      T.block<3, 1>(0, 3) << pose_msg_2_->pose.position.x, pose_msg_2_->pose.position.y, pose_msg_2_->pose.position.z;
      T.block<3, 3>(0, 0) = q.toRotationMatrix();
      Eigen::Matrix<double, 4, 4> b1Te2 = b1Tb2 * T;  // end effector pose of robot 2 in the base frame of robot 1

      // compute the mean between the two poses
      Eigen::Matrix<double, 4, 4> b1Te1;
      b1Te1.setIdentity();
      b1Te1.block<3, 1>(0, 3) << pose_msg_1_->pose.position.x, pose_msg_1_->pose.position.y,
          pose_msg_1_->pose.position.z;
      Eigen::Quaterniond q1(pose_msg_1_->pose.orientation.w, pose_msg_1_->pose.orientation.x,
                            pose_msg_1_->pose.orientation.y, pose_msg_1_->pose.orientation.z);
      b1Te1.block<3, 3>(0, 0) = q1.toRotationMatrix();

      Eigen::Vector<double, 3> mean_position = (b1Te1.block<3, 1>(0, 3) + b1Te2.block<3, 1>(0, 3)) / 2;

      Eigen::Matrix3d bR1 = b1Te1.block<3, 3>(0, 0);
      Eigen::Matrix3d bR2 = b1Te2.block<3, 3>(0, 0);
      Eigen::Matrix3d R12 = bR1.transpose() * bR2;

      Eigen::AngleAxisd angleAxis(R12);
      double theta12 = angleAxis.angle();
      Eigen::Vector3d r12 = angleAxis.axis();

      Eigen::Matrix3d R12_half = Eigen::AngleAxisd(theta12 / 2, r12).toRotationMatrix();
      Eigen::Matrix3d bRa = bR1 * R12_half;

      // create a new pose message
      geometry_msgs::msg::PoseStamped new_pose_msg;
      new_pose_msg.header.stamp = this->now();
      new_pose_msg.header.frame_id = "world";

      new_pose_msg.pose.position.x = mean_position(0);
      new_pose_msg.pose.position.y = mean_position(1);
      new_pose_msg.pose.position.z = mean_position(2);

      Eigen::Quaterniond mean_orientation(bRa);
      new_pose_msg.pose.orientation.w = mean_orientation.w();
      new_pose_msg.pose.orientation.x = mean_orientation.x();
      new_pose_msg.pose.orientation.y = mean_orientation.y();
      new_pose_msg.pose.orientation.z = mean_orientation.z();

      // publish the new pose
      publisher_->publish(new_pose_msg);

      // publish the fkine of robot 1 and robot 2
      geometry_msgs::msg::PoseStamped fkine2_msg;
      fkine2_msg.header.stamp = this->now();
      fkine2_msg.header.frame_id = "world";
      fkine2_msg.pose.position.x = b1Te2(0, 3);
      fkine2_msg.pose.position.y = b1Te2(1, 3);
      fkine2_msg.pose.position.z = b1Te2(2, 3);
      Eigen::Quaterniond q2(b1Te2.block<3, 3>(0, 0));
      fkine2_msg.pose.orientation.w = q2.w();
      fkine2_msg.pose.orientation.x = q2.x();
      fkine2_msg.pose.orientation.y = q2.y();
      fkine2_msg.pose.orientation.z = q2.z();
      publisher_fkine2_->publish(fkine2_msg);


      geometry_msgs::msg::PoseStamped fkine1_msg;
      fkine1_msg.header.stamp = this->now();
      fkine1_msg.header.frame_id = "world";
      fkine1_msg.pose.position.x = pose_msg_1_->pose.position.x;
      fkine1_msg.pose.position.y = pose_msg_1_->pose.position.y;
      fkine1_msg.pose.position.z = pose_msg_1_->pose.position.z;
      fkine1_msg.pose.orientation.w = pose_msg_1_->pose.orientation.w;
      fkine1_msg.pose.orientation.x = pose_msg_1_->pose.orientation.x;
      fkine1_msg.pose.orientation.y = pose_msg_1_->pose.orientation.y;
      fkine1_msg.pose.orientation.z = pose_msg_1_->pose.orientation.z;
      publisher_fkine1_->publish(fkine1_msg);
      


    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for pose messages...");
    }
  }

  void poseCallback1(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    pose_msg_1_ = msg;
  }

  void poseCallback2(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    pose_msg_2_ = msg;
  }
  void poseCallback3(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "world";
    pose_msg.pose.position.x = msg->pose.position.x;
    pose_msg.pose.position.y = msg->pose.position.y;
    pose_msg.pose.position.z = msg->pose.position.z;
    pose_msg.pose.orientation.w = msg->pose.orientation.w;
    pose_msg.pose.orientation.x = msg->pose.orientation.x;
    pose_msg.pose.orientation.y = msg->pose.orientation.y;
    pose_msg.pose.orientation.z = msg->pose.orientation.z;
    publisher_fkine3_->publish(pose_msg); 
  }
  void poseCallback4(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "world";
    pose_msg.pose.position.x = msg->pose.position.x;
    pose_msg.pose.position.y = msg->pose.position.y;
    pose_msg.pose.position.z = msg->pose.position.z;
    pose_msg.pose.orientation.w = msg->pose.orientation.w;
    pose_msg.pose.orientation.x = msg->pose.orientation.x;
    pose_msg.pose.orientation.y = msg->pose.orientation.y;
    pose_msg.pose.orientation.z = msg->pose.orientation.z;
    publisher_fkine4_->publish(pose_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription1_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription2_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription3_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription4_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_fkine1_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_fkine2_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_fkine3_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_fkine4_;

  geometry_msgs::msg::PoseStamped::SharedPtr pose_msg_1_;
  geometry_msgs::msg::PoseStamped::SharedPtr pose_msg_2_;


  // timer
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AbsolutePosePublisher>());
  rclcpp::shutdown();
  return 0;
}