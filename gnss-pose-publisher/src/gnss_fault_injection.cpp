#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include <random>

class GnssFaultInjectionNode : public rclcpp::Node
{
public:
    GnssFaultInjectionNode()
    : Node("gnss_fault_injection")
    {
        // subscribe gnss pose(awsim gnss pose)
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/sensing/gnss/pose_with_covariance", 10,
            std::bind(&GnssFaultInjectionNode::pose_callback, this, std::placeholders::_1));

        // user can inject noise via commandline
        noise_command_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/gnss_fault_injection/command", 10,
            std::bind(&GnssFaultInjectionNode::command_callback, this, std::placeholders::_1));

        // publish gnss pose with noise
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/sensing/gnss/pose_with_fault_injection", 10);
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (apply_noise_)
        {
            // random noise
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(-0.5, 0.5);

            // add noise to the pose, orientation
            geometry_msgs::msg::PoseWithCovarianceStamped noisy_pose = *msg;
            noisy_pose.pose.pose.position.x += dis(gen);
            noisy_pose.pose.pose.position.y += dis(gen);
            noisy_pose.pose.pose.position.z += dis(gen);
            noisy_pose.pose.pose.orientation.x += dis(gen);
            noisy_pose.pose.pose.orientation.y += dis(gen);
            noisy_pose.pose.pose.orientation.z += dis(gen);

            // if user wrote commandline with 'start' publish gnss pose wiwth noise
            pose_publisher_->publish(noisy_pose);
        }
        else
        {
            // if not publish gnss pose without noise
            pose_publisher_->publish(*msg);
        }
    }

    void command_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "start")
        {
            // start noise injection
            apply_noise_ = true;
            RCLCPP_INFO(this->get_logger(), "Noise injection started.");
        }
        else if (msg->data == "stop")
        {
            // stop noise injection
            apply_noise_ = false;
            RCLCPP_INFO(this->get_logger(), "Noise injection stopped.");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr noise_command_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
    bool apply_noise_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GnssFaultInjectionNode>());
    rclcpp::shutdown();
    return 0;
}
