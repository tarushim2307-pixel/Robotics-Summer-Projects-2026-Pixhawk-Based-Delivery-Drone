#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, GlobalPositionTarget
from mavros_msgs.srv import SetMode, CommandBool
from std_srvs.srv import Trigger

class GlobalWaypointNode(Node):
    def __init__(self):
        super().__init__('global_waypoint_node')

        # Global Position Publisher & Subscriber
        self.global_target_pub = self.create_publisher(
            GlobalPositionTarget, 
            '/mavros/setpoint_raw/global', 
            10
        )
        self.global_pose_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.global_pose_callback,
            qos_profile_sensor_data
        )
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile_sensor_data
        )

        # Services
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # Runtime Input Topic: Accepts (latitude, longitude, altitude)
        self.wp_sub = self.create_subscription(
            NavSatFix,
            '/add_global_waypoint',
            self.add_waypoint_callback,
            10
        )
        self.clear_wp_service = self.create_service(Trigger, 'clear_waypoints', self.clear_waypoints_callback)

        # Global Waypoint Queue: List of (lat, lon, alt)
        self.waypoints = []
        self.current_wp_idx = 0
        self.home_alt = 0.0

        self.current_lat = None
        self.current_lon = None
        self.current_alt = None
        self.current_state = State()
        self.step_counter = 0
        self.last_request_time = self.get_clock().now()

        # Arrival Radius (~2.5 meters in lat/lon distance)
        self.acceptance_radius_m = 2.5

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Global Waypoint Node Active! Send (Lat, Lon, Alt) to '/add_global_waypoint'.")

    def global_pose_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude
        if self.home_alt == 0.0 and msg.altitude != 0.0:
            self.home_alt = msg.altitude

    def state_callback(self, msg):
        self.current_state = msg

    def add_waypoint_callback(self, msg):
        lat, lon, alt = msg.latitude, msg.longitude, msg.altitude

        # REALISTIC CORRIDOR FOR GLOBAL COORDINATES
        # If altitude is 0 or near ground, first fly overhead at current altitude, then land
        if alt <= 0.5:
            cruise_alt = self.current_alt if self.current_alt else self.home_alt + 5.0
            self.waypoints.append((lat, lon, cruise_alt))  # Overflight
            self.waypoints.append((lat, lon, self.home_alt)) # Vertical Land
            self.get_logger().info(f"✈️ Auto Corridor: Fly to ({lat:.6f}, {lon:.6f}) at alt {cruise_alt:.1f}m, then land.")
        else:
            self.waypoints.append((lat, lon, alt))
            self.get_logger().info(f"➕ Added Global Waypoint [{len(self.waypoints)-1}]: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.1f}m")

    def clear_waypoints_callback(self, request, response):
        self.waypoints.clear()
        self.current_wp_idx = 0
        response.success = True
        response.message = "Global waypoints cleared!"
        self.get_logger().info("🧹 All Waypoints Cleared!")
        return response

    def calculate_distance_meters(self, lat1, lon1, lat2, lon2):
        # Haversine formula to find distance between two GPS coordinates
        R = 6371000.0
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat / 2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def control_loop(self):
        if self.current_lat is None:
            return

        # Determine target Lat/Lon/Alt
        if self.waypoints and self.current_wp_idx < len(self.waypoints):
            target_lat, target_lon, target_alt = self.waypoints[self.current_wp_idx]
        elif self.waypoints:
            target_lat, target_lon, target_alt = self.waypoints[-1]
        else:
            target_lat, target_lon = self.current_lat, self.current_lon
            target_alt = self.current_alt

        # Publish Global Position Target
        target = GlobalPositionTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "earth"
        target.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_INT
        target.type_mask = 4088  # Position control only (ignore vel/accel/yaw)
        target.latitude = target_lat
        target.longitude = target_lon
        target.altitude = target_alt
        self.global_target_pub.publish(target)

        self.step_counter += 1
        if self.step_counter < 60:
            return

        # State management
        now = self.get_clock().now()
        if (now - self.last_request_time).nanoseconds > 1e9:
            if self.current_state.mode != "OFFBOARD":
                self.set_offboard_mode()
            elif not self.current_state.armed:
                self.arm_drone()
            self.last_request_time = now

        # Compute distance error in meters
        dist_h = self.calculate_distance_meters(self.current_lat, self.current_lon, target_lat, target_lon)
        dist_v = abs(target_alt - self.current_alt)
        total_error = math.sqrt(dist_h**2 + dist_v**2)

        # Print Status every 1s
        if self.step_counter % 20 == 0:
            self.get_logger().info(
                f"\n--- GLOBAL WAYPOINT {self.current_wp_idx}/{max(0, len(self.waypoints)-1)} ---"
                f"\nMode: {self.current_state.mode} | Armed: {self.current_state.armed}"
                f"\nTarget : Lat={target_lat:.6f}, Lon={target_lon:.6f}, Alt={target_alt:.1f}m"
                f"\nActual : Lat={self.current_lat:.6f}, Lon={self.current_lon:.6f}, Alt={self.current_alt:.1f}m"
                f"\nDistance Error : {total_error:.2f} meters"
            )

        # Check Arrival
        if self.current_state.armed and self.waypoints and self.current_wp_idx < len(self.waypoints):
            if total_error < self.acceptance_radius_m:
                self.get_logger().info(f"✅ GLOBAL WAYPOINT {self.current_wp_idx} REACHED!")
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
    node = GlobalWaypointNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
