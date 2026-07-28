// ==========================================
// INCLUDES (Bringing in the necessary tools)
// ==========================================
#include <chrono>   
#include <memory>   
#include <string>   

#include "rclcpp/rclcpp.hpp" 

// These are the MAVROS message types required for the gripper tracking.
#include "mavros_msgs/msg/state.hpp"               
#include "geometry_msgs/msg/pose_stamped.hpp"      
#include "mavros_msgs/srv/command_long.hpp"        // Service to send the Servo command

using namespace std::chrono_literals;

// ==========================================
// THE MAIN NODE CLASS
// ==========================================
class GripperControlNode : public rclcpp::Node
{
public:
    GripperControlNode() : Node("gripper_control_node"), gripper_actuated_(false)
    {
        rclcpp::QoS qos_profile(10);
        qos_profile.best_effort();

        // --- SUBSCRIBERS (Listening to the drone) ---
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", qos_profile, 
            std::bind(&GripperControlNode::state_cb, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos_profile, 
            std::bind(&GripperControlNode::pose_cb, this, std::placeholders::_1));

        // --- CLIENTS (Talking to the gripper servo via MAVROS) ---
        cmd_client_ = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command"); 

        // --- MONITORING TIMER ---
        timer_ = this->create_wall_timer(
            100ms, std::bind(&GripperControlNode::gripper_loop, this));

        RCLCPP_INFO(this->get_logger(), "C++ Gripper Control Node Initialized. Monitoring altitude...");
    }

private:
    // ==========================================
    // VARIABLES
    // ==========================================
    mavros_msgs::msg::State current_mavros_state_; 
    double current_altitude_ = 0.0;              
    double target_altitude_ = 10.0;              
    bool gripper_actuated_;                      // Ensures we only actuate the gripper once

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr cmd_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ==========================================
    // CALLBACK FUNCTIONS
    // ==========================================
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_mavros_state_ = *msg; 
    }

    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_altitude_ = msg->pose.position.z; 
    }

    // ==========================================
    // GRIPPER LOGIC ENGINE
    // ==========================================
    void actuate_gripper(bool open_gripper)
    {
        if (!cmd_client_->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Command service not available!");
            return;
        }

        auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        req->broadcast = false;
        req->command = 183; // MAV_CMD_DO_SET_SERVO
        req->confirmation = 0;
        req->param1 = 1.0;  // AUX 1 (Servo 1)
        req->param2 = open_gripper ? 2000.0 : 1000.0; 

        RCLCPP_INFO(this->get_logger(), "Actuating gripper... PWM: %f", req->param2);

        cmd_client_->async_send_request(
            req, std::bind(&GripperControlNode::gripper_response_cb, this, std::placeholders::_1));
    }

    void gripper_response_cb(rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future)
    {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Gripper actuated successfully.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Gripper command failed. Code: %d", response->result);
        }
    }

    // ==========================================
    // GRIPPER LOOP (Checks conditions 10x every second)
    // ==========================================
    void gripper_loop()
    {
        if (!current_mavros_state_.connected) {
            return; 
        }

        // Semantics Match: Trigger if we haven't already and the threshold height is met
        if (!gripper_actuated_ && current_altitude_ >= (target_altitude_ - 0.5)) {
            RCLCPP_INFO(this->get_logger(), "Target altitude detected by monitor. Dropping payload!");
            actuate_gripper(true); 
            gripper_actuated_ = true; // Set flag to avoid repeating commands
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GripperControlNode>());
    rclcpp::shutdown();
    return 0;
}