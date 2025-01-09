#include <cstdio>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"
#include "joint_simulator/PIDController.h"
#include "joint_simulator_msg/srv/set_reference.hpp"

class PIDControllerNode : public rclcpp::Node
{
public:
    PIDControllerNode()
    : Node("pid_controller_node"), pid_controller_(1.0, 0.0, 0.0) // Initialize PID with some gains
    {
        // Subscriber to "/measured_angle"
        angle_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/measured_angle", 10, std::bind(&PIDControllerNode::angle_callback, this, std::placeholders::_1));
        
        // Publisher to "/effort"
        effort_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/effort", 10);
        
        

        service_ = this->create_service<joint_simulator_msg::srv::SetReference>(
            "SetReference",
            std::bind(&PIDControllerNode::set_reference_callback, this, std::placeholders::_1, std::placeholders::_2));


    }

private:
    void set_reference_callback(
        const std::shared_ptr<joint_simulator_msg::srv::SetReference::Request> request,
        std::shared_ptr<joint_simulator_msg::srv::SetReference::Response> response)
    {
        double new_reference = request->reference;
        pid_controller_.setReference(new_reference); // Assuming PIDController has setReference method
        response->success = true;
    }
    rclcpp::Service<joint_simulator_msg::srv::SetReference>::SharedPtr service_;

    void angle_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        double measured_angle = msg->data;
        pid_controller_.update(measured_angle); // Update effort using PID controller
        

        auto effort_msg = std_msgs::msg::Float64();
        effort_msg.data = pid_controller_.getEffort(); // Get the control effort
        effort_publisher_->publish(effort_msg); // Publish effort
    }

    PIDController pid_controller_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr effort_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDControllerNode>());
    rclcpp::shutdown();
    return 0;
}