#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointMux : public rclcpp::Node
{
public:
    JointMux() : Node("joint_mux"), joint1_received_(false), joint2_received_(false)
    {
        joint1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states_1", 1, std::bind(&JointMux::joint1_callback, this, std::placeholders::_1));
        joint2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states_2", 1, std::bind(&JointMux::joint2_callback, this, std::placeholders::_1));
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    }

private:
    void joint1_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        joint1_state_ = *msg;
        joint1_received_ = true;
        publish_joint_states();
    }

    void joint2_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        joint2_state_ = *msg;
        joint2_received_ = true;
        publish_joint_states();
    }

    void publish_joint_states()
    {
        if (joint1_received_ && joint2_received_)
        {
            auto joint_msg = sensor_msgs::msg::JointState();
            joint_msg.header.stamp = this->now();

            joint_msg.name.insert(joint_msg.name.end(), joint1_state_.name.begin(), joint1_state_.name.end());
            joint_msg.name.insert(joint_msg.name.end(), joint2_state_.name.begin(), joint2_state_.name.end());

            joint_msg.position.insert(joint_msg.position.end(), joint1_state_.position.begin(), joint1_state_.position.end());
            joint_msg.position.insert(joint_msg.position.end(), joint2_state_.position.begin(), joint2_state_.position.end());

            joint_msg.velocity.insert(joint_msg.velocity.end(), joint1_state_.velocity.begin(), joint1_state_.velocity.end());
            joint_msg.velocity.insert(joint_msg.velocity.end(), joint2_state_.velocity.begin(), joint2_state_.velocity.end());

            joint_msg.effort.insert(joint_msg.effort.end(), joint1_state_.effort.begin(), joint1_state_.effort.end());
            joint_msg.effort.insert(joint_msg.effort.end(), joint2_state_.effort.begin(), joint2_state_.effort.end());

            joint_pub_->publish(joint_msg);

            // Reset the flags
            joint1_received_ = false;
            joint2_received_ = false;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint2_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    sensor_msgs::msg::JointState joint1_state_;
    sensor_msgs::msg::JointState joint2_state_;
    bool joint1_received_;
    bool joint2_received_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointMux>());
    rclcpp::shutdown();
    return 0;
}
