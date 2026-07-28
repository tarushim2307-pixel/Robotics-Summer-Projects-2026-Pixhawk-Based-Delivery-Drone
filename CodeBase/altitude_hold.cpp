// ==========================================
// INCLUDES (Bringing in the necessary tools)
// ==========================================
#include <chrono>   // For handling time (like waiting 10 seconds or running at 10Hz)
#include <memory>   // For smart pointers (memory management, a core part of modern C++)
#include <string>   // For handling text strings (like "TAKEOFF" or "LOITER")

#include "rclcpp/rclcpp.hpp" // The core ROS 2 C++ library. This makes this script a "Node".

// These are the MAVROS message types we are bringing in. 
#include "mavros_msgs/msg/state.hpp"               // Drone's overall state (Connected? Armed? Mode?)
#include "geometry_msgs/msg/pose_stamped.hpp"      // 3D position data (X, Y, Z altitude)
#include "mavros_msgs/srv/command_bool.hpp"        // Service to send a True/False command (used for Arming)
#include "mavros_msgs/srv/set_mode.hpp"            // Service to change the flight mode (e.g., to TAKEOFF)

// This lets us write "100ms" or "10s" directly in the code for time measurements.
using namespace std::chrono_literals;

// ==========================================
// STATE MACHINE SETUP
// ==========================================
enum class MissionState {
    IDLE,       // Waiting to connect
    SETUP,      // (Optional) Setting parameters
    ARMING,     // Turning the motors on
    TAKEOFF,    // Climbing automatically
    CLIMBING,   // Monitoring the altitude until we hit our target
    HOLDING,    // Hovering for a set duration
    LANDING,    // Coming back down
    DONE        // Mission finished, do nothing
};

// ==========================================
// THE MAIN NODE CLASS
// ==========================================
class AltitudeHoldNode : public rclcpp::Node
{
public:
    AltitudeHoldNode() : Node("altitude_hold_node"), current_state_(MissionState::IDLE)
    {
        rclcpp::QoS qos_profile(10);
        qos_profile.best_effort();

        // --- SUBSCRIBERS (Listening to the drone) ---
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", qos_profile, 
            std::bind(&AltitudeHoldNode::state_cb, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos_profile, 
            std::bind(&AltitudeHoldNode::pose_cb, this, std::placeholders::_1));

        // --- CLIENTS (Talking to the drone) ---
        arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        // --- THE HEARTBEAT TIMER ---
        timer_ = this->create_wall_timer(
            100ms, std::bind(&AltitudeHoldNode::mission_loop, this));

        RCLCPP_INFO(this->get_logger(), "C++ Altitude Hold Node Initialized. Waiting for connection...");
    }

private:
    // ==========================================
    // VARIABLES
    // ==========================================
    MissionState current_state_;                 
    mavros_msgs::msg::State current_mavros_state_; 
    double current_altitude_ = 0.0;              
    double target_altitude_ = 10.0;              
    rclcpp::Time hold_start_time_;               
    bool request_pending_ = false;               

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
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
    // MISSION LOOP (Runs 10x every second)
    // ==========================================
    void mission_loop()
    {
        if (!current_mavros_state_.connected) {
            return; 
        }

        switch (current_state_) {
            
            case MissionState::IDLE:
                RCLCPP_INFO(this->get_logger(), "Connected to PX4. Starting flight mission.");
                current_state_ = MissionState::ARMING;
                break;

            case MissionState::SETUP:
                current_state_ = MissionState::ARMING;
                break;

            case MissionState::ARMING:
                if (!current_mavros_state_.armed && !request_pending_) {
                    arm_drone(); 
                } 
                else if (current_mavros_state_.armed) {
                    RCLCPP_INFO(this->get_logger(), "Drone is ARMED. Taking off.");
                    request_pending_ = false; 
                    current_state_ = MissionState::TAKEOFF; 
                }
                break;

            case MissionState::TAKEOFF:
                if (current_mavros_state_.mode != "TAKEOFF" && !request_pending_) {
                    set_mode("TAKEOFF"); 
                } 
                else if (current_mavros_state_.mode == "TAKEOFF") {
                    request_pending_ = false;
                    current_state_ = MissionState::CLIMBING; 
                }
                break;

            case MissionState::CLIMBING:
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "Climbing... Alt: %.2f / %.2f", current_altitude_, target_altitude_);
                
                // Target reached (10m - 0.5m margin)
                if (current_altitude_ >= (target_altitude_ - 0.5)) {
                    RCLCPP_INFO(this->get_logger(), "Target altitude reached. Switching to LOITER.");
                    
                    set_mode("LOITER");    // Tell the drone to hold its GPS position
                    
                    hold_start_time_ = this->get_clock()->now(); 
                    current_state_ = MissionState::HOLDING; 
                }
                break;

            case MissionState::HOLDING:
                {
                    auto elapsed = (this->get_clock()->now() - hold_start_time_).seconds();
                    
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                        "Holding position... %.1fs / 10.0s", elapsed);
                    
                    if (elapsed >= 10.0) {
                        RCLCPP_INFO(this->get_logger(), "Hold complete. Landing.");
                        request_pending_ = false; // Ensure flag is cleared for LANDING
                        current_state_ = MissionState::LANDING; 
                    }
                }
                break;

            case MissionState::LANDING:
                if (current_mavros_state_.mode != "LAND" && !request_pending_) {
                    set_mode("LAND"); 
                } 
                else if (!current_mavros_state_.armed) {
                    RCLCPP_INFO(this->get_logger(), "Drone has landed and disarmed. Flight Mission Complete.");
                    current_state_ = MissionState::DONE; 
                }
                break;

            case MissionState::DONE:
                break;
        }
    }

    // ==========================================
    // HELPER FUNCTIONS
    // ==========================================
    void arm_drone()
    {
        if (!arm_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Arming service not ready yet.");
            return;
        }

        request_pending_ = true; 
        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        req->value = true; 
        
        arm_client_->async_send_request(req, 
            [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
                if (future.get()->success) {
                    RCLCPP_INFO(this->get_logger(), "Arm command sent successfully.");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to send Arm command.");
                    this->request_pending_ = false; 
                }
            });
    }

    void set_mode(const std::string& mode)
    {
        if (!mode_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "SetMode service not ready yet.");
            return;
        }

        request_pending_ = true;
        auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        req->custom_mode = mode; 
        
        mode_client_->async_send_request(req, 
            [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                this->request_pending_ = false; // Reset flag when response is received
                if (future.get()->mode_sent) {
                    RCLCPP_INFO(this->get_logger(), "Set mode %s sent successfully.", mode.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set mode %s.", mode.c_str());
                }
            });
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AltitudeHoldNode>());
    rclcpp::shutdown();
    return 0;
}