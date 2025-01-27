#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointMux : public rclcpp::Node
{
public:
    JointMux() : Node("joint_mux")
    {
        this->declare_parameter<int>("joint_num_1", 7);
        this->declare_parameter<int>("joint_num_2", 1);
        this->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>());

        this->get_parameter("joint_num_1", joint_num_1_);
        this->get_parameter("joint_num_2", joint_num_2_);
        this->get_parameter("joint_names", joint_names_);

        // print joint names    
        for (const auto& name : joint_names_) {
            std::cout << name << std::endl;
        }

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
        publish_joint_states();
    }

    void joint2_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        joint2_state_ = *msg;
        publish_joint_states();
    }

    void publish_joint_states()
    {
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = this->now();

        joint_msg.position.reserve(joint_num_1_ + joint_num_2_);
        joint_msg.velocity.reserve(joint_num_1_ + joint_num_2_);
        joint_msg.effort.reserve(joint_num_1_ + joint_num_2_);

        joint_msg.position.insert(joint_msg.position.end(), joint1_state_.position.begin(), joint1_state_.position.end());
        joint_msg.position.insert(joint_msg.position.end(), joint2_state_.position.begin(), joint2_state_.position.end());

        joint_msg.velocity.insert(joint_msg.velocity.end(), joint1_state_.velocity.begin(), joint1_state_.velocity.end());
        joint_msg.velocity.insert(joint_msg.velocity.end(), joint2_state_.velocity.begin(), joint2_state_.velocity.end());

        joint_msg.effort.insert(joint_msg.effort.end(), joint1_state_.effort.begin(), joint1_state_.effort.end());
        joint_msg.effort.insert(joint_msg.effort.end(), joint2_state_.effort.begin(), joint2_state_.effort.end());

        if (joint_names_.empty()) {
            joint_msg.name.reserve(joint_num_1_ + joint_num_2_);
            joint_msg.name.insert(joint_msg.name.end(), joint1_state_.name.begin(), joint1_state_.name.end());
            joint_msg.name.insert(joint_msg.name.end(), joint2_state_.name.begin(), joint2_state_.name.end());
        } else {
            joint_msg.name.reserve(joint_names_.size());
            joint_msg.name = joint_names_;
        }

        joint_pub_->publish(joint_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint2_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    sensor_msgs::msg::JointState joint1_state_;
    sensor_msgs::msg::JointState joint2_state_;

    int joint_num_1_;
    int joint_num_2_;
    std::vector<std::string> joint_names_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointMux>());
    rclcpp::shutdown();
    return 0;
}
