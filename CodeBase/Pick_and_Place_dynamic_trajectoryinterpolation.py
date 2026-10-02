#!/usr/bin/env python3

import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import time

class PickAndPlaceNode(Node):
    def __init__(self, box_x, box_y, drop_x, drop_y):
        super().__init__('pick_and_place_node')

        self.current_state = State()
        self.target_pose = PoseStamped()

        self.state = "INIT"
        self.state_timer = 0.0

        # Dynamic runtime coordinates
        self.HOME_POS = [0.0, 0.0, 0.25]       # Initial takeoff spot
        self.BOX_POS = [box_x, box_y, 0.25]    # Pickup position
        self.DROP_POS = [drop_x, drop_y, 0.25] # Drop position
        self.CRUISE_ALT = 3.0                  # Flight altitude

        # Trajectory Interpolation parameters
        self.MAX_VELOCITY = 0.5   # 0.5 m/s speed (Smooth flight)
        self.TIMER_PERIOD = 0.05  # 20 Hz loop rate

        # Calculate exact required transit times based on 0.5 m/s velocity + safety margins
        dist_drop_to_home = math.sqrt(drop_x**2 + drop_y**2)
        self.RETURN_HOME_TIME = max(12.0, (dist_drop_to_home / self.MAX_VELOCITY) + 4.0)

        self.current_setpoint = [0.0, 0.0, 0.0]
        self.target_goal = [0.0, 0.0, 0.0]
        self.land_requested = False

        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.timer = self.create_timer(self.TIMER_PERIOD, self.control_loop)
        self.get_logger().info(f"Node Initialized! Target Pickup: [{box_x}, {box_y}] | Target Drop: [{drop_x}, {drop_y}]")
        self.get_logger().info(f"Return home allocated time: {self.RETURN_HOME_TIME:.1f} seconds")

    def state_cb(self, msg):
        self.current_state = msg

    def set_target(self, x, y, z):
        self.target_goal = [float(x), float(y), float(z)]

    def update_interpolated_setpoint(self):
        dx = self.target_goal[0] - self.current_setpoint[0]
        dy = self.target_goal[1] - self.current_setpoint[1]
        dz = self.target_goal[2] - self.current_setpoint[2]

        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        max_step = self.MAX_VELOCITY * self.TIMER_PERIOD

        if distance <= max_step or distance == 0.0:
            self.current_setpoint = list(self.target_goal)
        else:
            self.current_setpoint[0] += (dx / distance) * max_step
            self.current_setpoint[1] += (dy / distance) * max_step
            self.current_setpoint[2] += (dz / distance) * max_step

    def control_loop(self):
        self.update_interpolated_setpoint()

        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose.header.frame_id = "base_link"
        self.target_pose.pose.position.x = self.current_setpoint[0]
        self.target_pose.pose.position.y = self.current_setpoint[1]
        self.target_pose.pose.position.z = self.current_setpoint[2]

        self.local_pos_pub.publish(self.target_pose)

        if not self.current_state.connected:
            return

        if self.state == "INIT":
            self.set_target(self.HOME_POS[0], self.HOME_POS[1], self.CRUISE_ALT)
            if self.current_state.mode != 'OFFBOARD':
                self.set_mode_client.call_async(SetMode.Request(custom_mode='OFFBOARD'))
            elif not self.current_state.armed:
                self.arming_client.call_async(CommandBool.Request(value=True))
            else:
                self.get_logger().info("Armed & Offboard active! Initiating Smooth Takeoff...")
                self.state = "TAKEOFF"
                self.state_timer = time.time()

        elif self.state == "TAKEOFF":
            if time.time() - self.state_timer > 8.0:
                self.get_logger().info("Cruising to box coordinates...")
                self.state = "NAV_TO_BOX"
                self.state_timer = time.time()

        elif self.state == "NAV_TO_BOX":
            self.set_target(self.BOX_POS[0], self.BOX_POS[1], self.CRUISE_ALT)
            if time.time() - self.state_timer > 10.0:
                self.get_logger().info("Descending to pick box...")
                self.state = "DESCEND_BOX"
                self.state_timer = time.time()

        elif self.state == "DESCEND_BOX":
            self.set_target(self.BOX_POS[0], self.BOX_POS[1], self.BOX_POS[2])
            if time.time() - self.state_timer > 8.0:
                self.get_logger().info("Simulated Pickup complete! Lifting off...")
                self.state = "ASCEND_WITH_CARGO"
                self.state_timer = time.time()

        elif self.state == "ASCEND_WITH_CARGO":
            self.set_target(self.BOX_POS[0], self.BOX_POS[1], self.CRUISE_ALT)
            if time.time() - self.state_timer > 8.0:
                self.get_logger().info("Navigating to Drop Zone...")
                self.state = "NAV_TO_DROP"
                self.state_timer = time.time()

        elif self.state == "NAV_TO_DROP":
            self.set_target(self.DROP_POS[0], self.DROP_POS[1], self.CRUISE_ALT)
            if time.time() - self.state_timer > 10.0:
                self.get_logger().info("Descending at Drop Zone...")
                self.state = "DESCEND_DROP"
                self.state_timer = time.time()

        elif self.state == "DESCEND_DROP":
            self.set_target(self.DROP_POS[0], self.DROP_POS[1], self.DROP_POS[2])
            if time.time() - self.state_timer > 8.0:
                self.get_logger().info("Simulated Release complete! Returning to Red Square (Start)...")
                self.state = "RETURN_HOME"
                self.state_timer = time.time()

        elif self.state == "RETURN_HOME":
            self.set_target(self.HOME_POS[0], self.HOME_POS[1], self.CRUISE_ALT)
            # Uses dynamically calculated distance transit time
            if time.time() - self.state_timer > self.RETURN_HOME_TIME:
                self.state = "LAND"

        elif self.state == "LAND":
            if not self.land_requested:
                self.get_logger().info("Arrived at Red Square! Initiating auto landing...")
                self.set_mode_client.call_async(SetMode.Request(custom_mode='AUTO.LAND'))
                self.land_requested = True

def main(args=None):
    rclpy.init(args=args)

    filtered_args = rclpy.utilities.remove_ros_args(sys.argv)

    box_x, box_y = 2.0, 0.0
    drop_x, drop_y = 5.0, 3.0

    if len(filtered_args) >= 5:
        box_x = float(filtered_args[1])
        box_y = float(filtered_args[2])
        drop_x = float(filtered_args[3])
        drop_y = float(filtered_args[4])

    node = PickAndPlaceNode(box_x, box_y, drop_x, drop_y)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
