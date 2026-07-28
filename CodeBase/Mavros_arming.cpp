#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"

// Necessary MAVROS Message/Service headers
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"

using namespace std::chrono_literals;

class PixhawkArmingNode : public rclcpp::Node
{
public:
    PixhawkArmingNode() : Node("pixhawk_arming_node"), request_pending_(false)
    {
        // MAVROS uses best_effort Quality of Service (QoS) profiles for state tracking
        rclcpp::QoS qos_profile(10);
        qos_profile.best_effort();

        // 1. Subscribe to the MAVROS state topic to monitor connection status
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", qos_profile,
            std::bind(&PixhawkArmingNode::state_callback, this, std::placeholders::_1));

        // 2. Create a service client to send the Arm/Disarm command
        arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

        // 3. Create a control loop timer running at 10Hz (every 100ms)
        timer_ = this->create_wall_timer(
            100ms, std::bind(&PixhawkArmingNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Arming Node Started. Awaiting Pixhawk Connection...");
    }

private:
    // Variables to track drone state
    mavros_msgs::msg::State current_state_;
    bool request_pending_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Callback that saves the latest telemetry state updates
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
    }

    // Core state loop executed every 100ms
    void control_loop()
    {
        // Ensure MAVROS has confirmed it is talking to the Pixhawk
        if (!current_state_.connected) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for FCU connection...");
            return;
        }

        // If the vehicle is connected but not yet armed, attempt to arm it
        if (!current_state_.armed) {
            if (!request_pending_) {
                send_arm_request(true);
            }
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Success! Drone is safely ARMED.");
        }
    }

    // Sends the asynchronous arm command to the Pixhawk
    void send_arm_request(bool arm_status)
    {
        if (!arm_client_->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Arming service not available!");
            return;
        }

        request_pending_ = true;
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = arm_status; // true = Arm, false = Disarm

        RCLCPP_INFO(this->get_logger(), "Sending Arm request...");

        arm_client_->async_send_request(request,
            [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "Arm command accepted by flight controller.");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Arm command rejected! (Check safety switch or pre-arm failsafes)");
                    // Reset flag so the node can retry next cycle
                    this->request_pending_ = false; 
                }
            });
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PixhawkArmingNode>());
    rclcpp::shutdown();
    return 0;
}