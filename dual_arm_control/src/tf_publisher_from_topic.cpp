#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class TfPublisherFromTopic : public rclcpp::Node
{
public:
    TfPublisherFromTopic()
    : Node("tf_publisher_from_topic")
    {
        this->declare_parameter<std::string>("source_frame", "source_frame");
        this->declare_parameter<std::string>("target_frame", "target_frame");
        this->get_parameter("source_frame", source_frame_);
        this->get_parameter("target_frame", target_frame_);

        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "pose_topic", rclcpp::SensorDataQoS(), std::bind(&TfPublisherFromTopic::poseCallback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;


        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = source_frame_;
        transform_stamped.child_frame_id = target_frame_;

        transform_stamped.transform.translation.x = msg->pose.position.x;
        transform_stamped.transform.translation.y = msg->pose.position.y;
        transform_stamped.transform.translation.z = msg->pose.position.z;

        transform_stamped.transform.rotation = msg->pose.orientation;

        tf_broadcaster_->sendTransform(transform_stamped);
    }

    std::string source_frame_;
    std::string target_frame_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfPublisherFromTopic>());
    rclcpp::shutdown();
    return 0;
}