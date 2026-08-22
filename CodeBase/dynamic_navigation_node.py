#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from std_srvs.srv import Trigger

class DynamicWaypointNode(Node):
    def __init__(self):
        super().__init__('dynamic_waypoint_node')

        # Publishers & Subscribers
        self.target_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile_sensor_data
        )
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile_sensor_data
        )

        # Service Clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # ROS 2 Topic & Service for Dynamic Waypoints
        self.wp_sub = self.create_subscription(
            Point,
            '/add_waypoint',
            self.add_waypoint_callback,
            10
        )
        self.clear_wp_service = self.create_service(Trigger, 'clear_waypoints', self.clear_waypoints_callback)

        # Waypoint Queue: List of (x, y, z) tuples
        self.waypoints = []
        self.current_wp_idx = 0

        self.current_pose = None
        self.current_state = State()
        self.step_counter = 0
        self.last_request_time = self.get_clock().now()

        # Arrival Acceptance threshold (20 cm)
        self.acceptance_radius = 0.2

        # Timer loop running at 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Dynamic Waypoint Node Active! Publish waypoints to '/add_waypoint'.")

    def pose_callback(self, msg):
        self.current_pose = msg.pose.position

    def state_callback(self, msg):
        self.current_state = msg

    def add_waypoint_callback(self, msg):
        wp = (msg.x, msg.y, msg.z)
        self.waypoints.append(wp)
        self.get_logger().info(f"➕ Added Waypoint [{len(self.waypoints)-1}]: X={wp[0]}, Y={wp[1]}, Z={wp[2]}")

    def clear_waypoints_callback(self, request, response):
        self.waypoints.clear()
        self.current_wp_idx = 0
        response.success = True
        response.message = "Waypoints cleared successfully!"
        self.get_logger().info("🧹 All Waypoints Cleared!")
        return response

    def control_loop(self):
        if self.current_pose is None:
            return  # Wait until live position feedback arrives

        # Determine Target Coordinates
        if self.waypoints and self.current_wp_idx < len(self.waypoints):
            target_x, target_y, target_z = self.waypoints[self.current_wp_idx]
        elif self.waypoints:
            # Hold last commanded waypoint position (prevents resetting target to current pose)
            target_x, target_y, target_z = self.waypoints[-1]
        else:
            # Default initial hover target before any waypoints are added
            target_x, target_y = 0.0, 0.0
            target_z = 2.0

        # Publish target setpoint stream
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "map"
        target.pose.position.x = target_x
        target.pose.position.y = target_y
        target.pose.position.z = target_z
        self.target_pub.publish(target)

        self.step_counter += 1

        if self.step_counter < 60:
            return

        now = self.get_clock().now()
        if (now - self.last_request_time).nanoseconds > 1e9:
            if self.current_state.mode != "OFFBOARD":
                self.set_offboard_mode()
            elif not self.current_state.armed and target_z > 0.1:
                self.arm_drone()
            self.last_request_time = now

        # Calculate actual Euclidean 3D Position Error
        actual_x, actual_y, actual_z = self.current_pose.x, self.current_pose.y, self.current_pose.z
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

        # Print Live Status Data
        if self.step_counter % 20 == 0:
            self.get_logger().info(
                f"\n--- WAYPOINT {self.current_wp_idx}/{max(0, len(self.waypoints)-1)} STATUS ---"
                f"\nMode: {self.current_state.mode} | Armed: {self.current_state.armed}"
                f"\nTarget Pos : X={target_x:.2f}m, Y={target_y:.2f}m, Z={target_z:.2f}m"
                f"\nActual Pos : X={actual_x:.2f}m, Y={actual_y:.2f}m, Z={actual_z:.2f}m"
                f"\nPosition Error : {pos_error:.3f} meters"
                f"\nAccuracy Error : {percentage_error:.2f} %"
            )

        # Check Arrival
        if self.current_state.armed and self.waypoints and self.current_wp_idx < len(self.waypoints):
            if pos_error < self.acceptance_radius:
                self.get_logger().info(f"✅ WAYPOINT {self.current_wp_idx} REACHED! (Final Error: {pos_error:.3f}m, {percentage_error:.2f}%)")
                self.current_wp_idx += 1
                if self.current_wp_idx >= len(self.waypoints):
                    self.get_logger().info("🎉 MISSION COMPLETE! Holding position or waiting for new waypoints...")

    def set_offboard_mode(self):
        if self.set_mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            self.set_mode_client.call_async(req)

    def arm_drone(self):
        if self.arming_client.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            self.arming_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicWaypointNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
