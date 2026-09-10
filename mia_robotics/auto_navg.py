#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
import time

class HolonomicAutoNavigator(Node):
    def __init__(self):
        super().__init__('auto_nav_holonomic_node')

        # --- ROS 2 Publishers & Subscribers ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.create_subscription(Float32, '/ultrasonic_distance', self.ultrasonic_cb, 10)
        self.create_subscription(Float32, '/target_x_error', self.vision_error_cb, 10)
        self.create_subscription(String, '/target_type', self.target_type_cb, 10)

        # --- State Tracking & Variables ---
        self.state = "SEARCH_STRAFE"
        self.scrolls_found = 0
        self.current_distance = 999.0  # Initial fallback distance in cm
        self.x_error = 0.0             # Horizontal pixel offset (-1.0 to 1.0)
        self.target_type = "none"

        # --- PD Controller Gains ---
        self.Kp = 0.8
        self.Kd = 0.12
        self.prev_error = 0.0

        # --- Speed Parameters ---
        self.STOP_DISTANCE = 25.0      # Safety stopping distance (cm)
        self.MAX_FORWARD_SPEED = 0.45  # Linear speed (m/s)
        self.MAX_STRAFE_SPEED = 0.55   # Sideways strafe speed (m/s)
        self.SEARCH_STRAFE_DIR = 1.0   # Direction toggle (1.0 right, -1.0 left)

        # --- ROS 2 Control Loop Timer (20 Hz -> 0.05s) ---
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("ROS 2 Holonomic Auto Navigator Node Initialized.")

    def ultrasonic_cb(self, msg):
        self.current_distance = msg.data

    def vision_error_cb(self, msg):
        self.x_error = msg.data

    def target_type_cb(self, msg):
        self.target_type = msg.data.lower()

    def handle_first_scroll_cleared(self):
        self.get_logger().info("First REAL Scroll reached! Executing bypass maneuver...")
        cmd = Twist()

        # Step 1: Pause briefly
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        self.cmd_pub.publish(cmd)
        time.sleep(1.0)

        # Step 2: Back off slightly
        cmd.linear.x = -0.15
        self.cmd_pub.publish(cmd)
        time.sleep(1.0)

        # Step 3: Strafe sideways to bypass
        cmd.linear.x = 0.0
        cmd.linear.y = 0.35 * self.SEARCH_STRAFE_DIR
        self.cmd_pub.publish(cmd)
        time.sleep(1.2)

        # Reset state for 2nd scroll search
        self.scrolls_found = 1
        self.state = "SEARCH_STRAFE"

    def control_loop(self):
        cmd = Twist()

        # --- STATE 1: SEARCHING ---
        if self.state == "SEARCH_STRAFE":
            if self.target_type == "real":
                self.state = "TRACK_HOLONOMIC"
                self.get_logger().info(f"REAL Target detected! Tracking scroll #{self.scrolls_found + 1}...")
            else:
                cmd.linear.x = 0.12
                cmd.linear.y = 0.3 * self.SEARCH_STRAFE_DIR
                cmd.angular.z = 0.0

        # --- STATE 2: TRACKING ---
        elif self.state == "TRACK_HOLONOMIC":
            if self.target_type != "real":
                self.state = "SEARCH_STRAFE"
                return

            # Safety check via Ultrasonic
            if self.current_distance <= self.STOP_DISTANCE:
                if self.scrolls_found == 0:
                    self.handle_first_scroll_cleared()
                    return
                else:
                    self.state = "STOPPING"
                    return

            # PD Control for side alignment
            error_derivative = self.x_error - self.prev_error
            strafe_correction = (self.Kp * self.x_error) + (self.Kd * error_derivative)
            self.prev_error = self.x_error

            # Clamp strafe limits
            cmd.linear.y = -max(min(strafe_correction, self.MAX_STRAFE_SPEED), -self.MAX_STRAFE_SPEED)

            # Linear deceleration
            if self.current_distance < 40.0:
                speed = self.MAX_FORWARD_SPEED * (self.current_distance / 40.0)
            else:
                speed = self.MAX_FORWARD_SPEED

            cmd.linear.x = max(speed, 0.1)
            cmd.angular.z = 0.0

        # --- STATE 3: STOPPING ---
        elif self.state == "STOPPING":
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("Autonomous Mission Complete! Awaiting Green Card.")

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = HolonomicAutoNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()