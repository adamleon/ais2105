//
// Created by ein on 25.11.24.
//

//
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <random>

class JointSimulator : public rclcpp::Node {
public:
    JointSimulator() : Node("joint_simulator"), angle(0.0), angular_velocity(0.0), torque(1.0), inertia(1.0), noise(1.0) {
        // Publisher for the measured angle
        angle_publisher = this->create_publisher<std_msgs::msg::Float64>("/measured_angle", 10);

        // Subscriber for the effort (torque)
        effort_subscriber = this->create_subscription<std_msgs::msg::Float64>(
            "/effort", 10, std::bind(&JointSimulator::torque_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Initialized subscribers");

        // Timer to update angle and publish it
        timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&JointSimulator::update_angle_and_publish, this)
        );

        RCLCPP_INFO(this->get_logger(), "Initialized timer");
    }

private:
    // Class variables
    double angle;
    double angular_velocity;
    double torque;
    double inertia;
    double noise;

    // ROS2 Publisher and Subscriber
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_publisher;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr effort_subscriber;

    // Timer
    rclcpp::TimerBase::SharedPtr timer;

    // Random number generator for noise
    std::default_random_engine generator;
    std::normal_distribution<double> distribution{0.0, 1.0};

    /**
     * Callback for the torque subscriber.
     * @param msg The message from the "/effort" topic, containing the new torque value.
     */
    void torque_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        torque = msg->data;
    }

    /**
     * Function to update the angle based on the current torque, inertia, and noise.
     * This function is called every 1/10 second.
     */
    void update_angle_and_publish() {
        // Calculate noise
        noise = distribution(generator);

        // Update angular velocity based on the current torque, inertia, and noise
        angular_velocity += (torque / inertia) * 0.1 + noise;

        // Update the angle based on the angular velocity
        angle += angular_velocity * 0.1;

        RCLCPP_INFO(this->get_logger(), "New angle: %f", angle);
        // Publish the updated angle
        std_msgs::msg::Float64 msg;
        msg.data = angle;
        angle_publisher->publish(msg);
    }
};

// Main function to initialize the ROS2 node
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointSimulator>());
    rclcpp::shutdown();
    return 0;
}
