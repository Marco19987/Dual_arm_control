#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointDemuxNode : public rclcpp::Node
{
public:
    JointDemuxNode() : Node("joint_demux_node")
    {
        this->declare_parameter<int>("split_index", 0);

        split_index_ = this->get_parameter("split_index").as_int();

        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(1);

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", qos, std::bind(&JointDemuxNode::jointStateCallback, this, std::placeholders::_1));

        joint_state_pub_1_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states_1", qos);
        joint_state_pub_2_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states_2", qos);
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        auto joint_state_1 = sensor_msgs::msg::JointState();
        auto joint_state_2 = sensor_msgs::msg::JointState();

        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (i < split_index_)
            {
                joint_state_1.name.push_back(msg->name[i]);
                if (i < msg->position.size()) {
                    joint_state_1.position.push_back(msg->position[i]);
                }
                if (i < msg->velocity.size()) {
                    joint_state_1.velocity.push_back(msg->velocity[i]);
                }
                if (i < msg->effort.size()) {
                    joint_state_1.effort.push_back(msg->effort[i]);
                }
            }
            else
            {
                joint_state_2.name.push_back(msg->name[i]);
                if (i < msg->position.size()) {
                    joint_state_2.position.push_back(msg->position[i]);
                }
                if (i < msg->velocity.size()) {
                    joint_state_2.velocity.push_back(msg->velocity[i]);
                }
                if (i < msg->effort.size()) {
                    joint_state_2.effort.push_back(msg->effort[i]);
                }
            }
        }

        joint_state_pub_1_->publish(joint_state_1);
        joint_state_pub_2_->publish(joint_state_2);
    }

    int split_index_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_1_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_2_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointDemuxNode>());
    rclcpp::shutdown();
    return 0;
}
