#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool

class WaypointNavigationNode(Node):
    def __init__(self):
        super().__init__('waypoint_navigation_node')

        # Publishers & Subscribers
        self.target_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile_sensor_data
        )
        # NEW: needed to know current mode/armed status before retrying
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile_sensor_data
        )

        # Service Clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # Waypoints: (X_meters, Y_meters, Z_meters) in ENU local frame
        # Note: in ENU, X = East, Y = North (comments below just label distance/order)
        self.waypoints = [
            (0.0, 0.0, 5.0),    # Takeoff to 5m
            (5.0, 0.0, 5.0),    # WP 1: 5m along +X (East)
            (5.0, 5.0, 5.0),    # WP 2: 5m along +Y (North)
            (0.0, 5.0, 5.0),    # WP 3: back along -X
            (0.0, 0.0, 5.0)     # WP 4: Return Home
        ]

        self.current_wp_idx = 0
        self.current_pose = None
        self.current_state = State()
        self.step_counter = 0
        self.last_request_time = self.get_clock().now()

        # Arrival Acceptance threshold (0.2 meters = 20 cm accuracy)
        self.acceptance_radius = 0.2

        # Timer loop running at 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Waypoint Navigation with Error Logging Started!")

    def pose_callback(self, msg):
        self.current_pose = msg.pose.position

    def state_callback(self, msg):
        self.current_state = msg

    def control_loop(self):
        if self.current_pose is None:
            return  # Wait until live position feedback arrives

        # Current target waypoint
        target_x, target_y, target_z = self.waypoints[self.current_wp_idx]

        # Send Target Position to Drone (must be streamed continuously, before
        # AND after arming/offboard, or PX4 will drop back out of OFFBOARD)
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "map"
        target.pose.position.x = target_x
        target.pose.position.y = target_y
        target.pose.position.z = target_z
        self.target_pub.publish(target)

        self.step_counter += 1

        # Stream setpoints for 3s (60 loops @ 20Hz) before attempting anything -
        # PX4 needs a healthy setpoint stream already running before it will
        # accept an OFFBOARD switch
        if self.step_counter < 60:
            return

        # Retry arm/offboard every 1s until BOTH are actually confirmed via
        # /mavros/state, instead of one unchecked attempt that can silently fail
        # if PX4's prearm/health checks weren't ready yet
        now = self.get_clock().now()
        if (now - self.last_request_time).nanoseconds > 1e9:
            if self.current_state.mode != "OFFBOARD":
                self.set_offboard_mode()
            elif not self.current_state.armed:
                self.arm_drone()
            self.last_request_time = now

        # Calculate actual Euclidean 3D Position Error
        actual_x = self.current_pose.x
        actual_y = self.current_pose.y
        actual_z = self.current_pose.z

        error_x = target_x - actual_x
        error_y = target_y - actual_y
        error_z = target_z - actual_z

        pos_error = math.sqrt(error_x**2 + error_y**2 + error_z**2)

        # Percentage error calculation
        target_distance = math.sqrt(target_x**2 + target_y**2 + target_z**2)
        if target_distance > 0:
            percentage_error = (pos_error / target_distance) * 100
        else:
            percentage_error = 0.0

        # Print Live Status Data to Terminal every 1 second
        if self.step_counter % 20 == 0:
            self.get_logger().info(
                f"\n--- WAYPOINT {self.current_wp_idx} STATUS ---"
                f"\nMode: {self.current_state.mode} | Armed: {self.current_state.armed} | Connected: {self.current_state.connected}"
                f"\nTarget Pos : X={target_x:.2f}m, Y={target_y:.2f}m, Z={target_z:.2f}m"
                f"\nActual Pos : X={actual_x:.2f}m, Y={actual_y:.2f}m, Z={actual_z:.2f}m"
                f"\nPosition Error : {pos_error:.3f} meters"
                f"\nAccuracy Error : {percentage_error:.2f} %"
            )

        # Check if Waypoint is Reached (only once actually armed & in OFFBOARD)
        if self.current_state.armed and pos_error < self.acceptance_radius:
            self.get_logger().info(f"✅ WAYPOINT {self.current_wp_idx} REACHED! (Final Error: {pos_error:.3f}m)")
            if self.current_wp_idx < len(self.waypoints) - 1:
                self.current_wp_idx += 1
            else:
                self.get_logger().info("🎉 MISSION COMPLETE! All waypoints reached.")

    def set_offboard_mode(self):
        if self.set_mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            self.set_mode_client.call_async(req)
        else:
            self.get_logger().warn("set_mode service not ready yet")

    def arm_drone(self):
        if self.arming_client.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            self.arming_client.call_async(req)
        else:
            self.get_logger().warn("arming service not ready yet")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
