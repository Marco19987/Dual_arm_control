#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class TopicPublisherFromTf : public rclcpp::Node
{
public:
    TopicPublisherFromTf()
    : Node("topic_publisher_from_tf")
    {
        this->declare_parameter<std::string>("source_frame", "source_frame");
        this->declare_parameter<std::string>("target_frame", "target_frame");
        this->declare_parameter<int>("timer_sample_time", 100);

        this->get_parameter("source_frame", source_frame_);
        this->get_parameter("target_frame", target_frame_);
        int timer_sample_time;
        this->get_parameter("timer_sample_time", timer_sample_time);

        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(1);
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_topic", qos);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_sample_time),
            std::bind(&TopicPublisherFromTf::timerCallback, this));
    }

private:
    void timerCallback()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_->lookupTransform(source_frame_, target_frame_, tf2::TimePointZero);

            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = transform_stamped.header.stamp;
            pose_msg.header.frame_id = source_frame_;

            pose_msg.pose.position.x = transform_stamped.transform.translation.x;
            pose_msg.pose.position.y = transform_stamped.transform.translation.y;
            pose_msg.pose.position.z = transform_stamped.transform.translation.z;

            pose_msg.pose.orientation = transform_stamped.transform.rotation;

            pose_publisher_->publish(pose_msg);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", source_frame_.c_str(), target_frame_.c_str(), ex.what());
        }
    }

    std::string source_frame_;
    std::string target_frame_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TopicPublisherFromTf>());
    rclcpp::shutdown();
    return 0;
}