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

        # Waypoint Queue & Smooth Trajectory Tracking Variables
        self.waypoints = []
        self.current_wp_idx = 0
        self.setpoint_x = 0.0
        self.setpoint_y = 0.0
        self.setpoint_z = 0.0
        self.is_setpoint_initialized = False

        # Flight Parameters for Smooth Movement
        self.max_velocity = 1.0       # Max speed in meters/second (prevents sudden jerks)
        self.dt = 0.05                # Timer interval (20Hz = 0.05s)
        self.acceptance_radius = 0.2  # 20 cm arrival radius

        self.current_pose = None
        self.current_state = State()
        self.step_counter = 0
        self.last_request_time = self.get_clock().now()

        # Timer loop running at 20Hz
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Smooth Dynamic Waypoint Node Active! Velocity capped at 1.0 m/s.")

    def pose_callback(self, msg):
        self.current_pose = msg.pose.position
        if not self.is_setpoint_initialized:
            self.setpoint_x = self.current_pose.x
            self.setpoint_y = self.current_pose.y
            self.setpoint_z = self.current_pose.z
            self.is_setpoint_initialized = True

    def state_callback(self, msg):
        self.current_state = msg

    def add_waypoint_callback(self, msg):
        wp = (msg.x, msg.y, msg.z)
        self.waypoints.append(wp)
        self.get_logger().info(f"➕ Added Waypoint [{len(self.waypoints)-1}]: X={wp[0]}, Y={wp[1]}, Z={wp[2]}")

    def clear_waypoints_callback(self, request, response):
        self.waypoints.clear()
        self.current_wp_idx = 0
        if self.current_pose:
            self.setpoint_x = self.current_pose.x
            self.setpoint_y = self.current_pose.y
            self.setpoint_z = self.current_pose.z
        response.success = True
        response.message = "Waypoints cleared successfully!"
        self.get_logger().info("🧹 All Waypoints Cleared!")
        return response

    def control_loop(self):
        if self.current_pose is None or not self.is_setpoint_initialized:
            return  # Wait for position initialization

        # Determine Target Waypoint
        if self.waypoints and self.current_wp_idx < len(self.waypoints):
            target_x, target_y, target_z = self.waypoints[self.current_wp_idx]
        elif self.waypoints:
            target_x, target_y, target_z = self.waypoints[-1]
        else:
            target_x, target_y, target_z = self.setpoint_x, self.setpoint_y, max(self.setpoint_z, 2.0)

        # SMOOTH TRAJECTORY INTERPOLATION (Position Ramping)
        dx = target_x - self.setpoint_x
        dy = target_y - self.setpoint_y
        dz = target_z - self.setpoint_z
        dist_to_target = math.sqrt(dx**2 + dy**2 + dz**2)

        # Max distance the setpoint can step in a single 20Hz frame
        max_step = self.max_velocity * self.dt

        if dist_to_target > max_step:
            # Interpolate incrementally towards target
            self.setpoint_x += (dx / dist_to_target) * max_step
            self.setpoint_y += (dy / dist_to_target) * max_step
            self.setpoint_z += (dz / dist_to_target) * max_step
        else:
            # Arrived at waypoint setpoint
            self.setpoint_x = target_x
            self.setpoint_y = target_y
            self.setpoint_z = target_z

        # Publish smooth interpolated target
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "map"
        target.pose.position.x = self.setpoint_x
        target.pose.position.y = self.setpoint_y
        target.pose.position.z = self.setpoint_z
        self.target_pub.publish(target)

        self.step_counter += 1

        if self.step_counter < 60:
            return

        # Auto Arm and Offboard Handling
        now = self.get_clock().now()
        if (now - self.last_request_time).nanoseconds > 1e9:
            if self.current_state.mode != "OFFBOARD":
                self.set_offboard_mode()
            elif not self.current_state.armed and target_z > 0.1:
                self.arm_drone()
            self.last_request_time = now

        # Calculate position error against real drone position
        actual_x, actual_y, actual_z = self.current_pose.x, self.current_pose.y, self.current_pose.z
        pos_error = math.sqrt((target_x - actual_x)**2 + (target_y - actual_y)**2 + (target_z - actual_z)**2)

        target_distance = math.sqrt(target_x**2 + target_y**2 + target_z**2)
        percentage_error = (pos_error / target_distance * 100) if target_distance > 0 else 0.0

        # Print Status every 1s
        if self.step_counter % 20 == 0:
            self.get_logger().info(
                f"\n--- WAYPOINT {self.current_wp_idx}/{max(0, len(self.waypoints)-1)} STATUS ---"
                f"\nMode: {self.current_state.mode} | Armed: {self.current_state.armed}"
                f"\nTarget Pos : X={target_x:.2f}m, Y={target_y:.2f}m, Z={target_z:.2f}m"
                f"\nActual Pos : X={actual_x:.2f}m, Y={actual_y:.2f}m, Z={actual_z:.2f}m"
                f"\nPosition Error : {pos_error:.3f} meters"
                f"\nAccuracy Error : {percentage_error:.2f} %"
            )

        # Advance to next waypoint
        if self.current_state.armed and self.waypoints and self.current_wp_idx < len(self.waypoints):
            if pos_error < self.acceptance_radius:
                self.get_logger().info(f"✅ WAYPOINT {self.current_wp_idx} REACHED! (Final Error: {pos_error:.3f}m, {percentage_error:.2f}%)")
                self.current_wp_idx += 1

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
