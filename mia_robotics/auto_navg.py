#!/usr/bin/env python3
import collections
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class AutoReturnWithTimeoutTriggerNavigator(Node):
    def __init__(self):
        super().__init__('auto_return_timeout_navigator')

        # --- 1. Dynamic ROS 2 Parameters ---
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('kd', 0.12)
        self.declare_parameter('max_forward_speed', 45.0)
        self.declare_parameter('max_strafe_speed', 55.0)
        self.declare_parameter('min_speed_floor', 20.0)
        self.declare_parameter('stop_distance', 25.0)

        self.Kp = self.get_parameter('kp').value
        self.Kd = self.get_parameter('kd').value
        self.MAX_FORWARD_SPEED = self.get_parameter('max_forward_speed').value
        self.MAX_STRAFE_SPEED = self.get_parameter('max_strafe_speed').value
        self.MIN_SPEED_FLOOR = self.get_parameter('min_speed_floor').value
        self.STOP_DISTANCE = self.get_parameter('stop_distance').value

        # --- 2. Publishers & Subscribers ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.create_subscription(Int32, '/ultrasonic_distance', self.ultrasonic_cb, 10)
        self.create_subscription(Float32, '/target_x_error', self.vision_error_cb, 10)
        self.create_subscription(String, '/target_type', self.target_type_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # --- 3. Safety Buffers & Vision Signals ---
        self.raw_distances = collections.deque(maxlen=5)
        self.filtered_distance = 999.0
        self.x_error = 0.0
        self.target_type = "none"

        # Staleness Watchdog
        self.last_ultra_time = self.get_clock().now()
        self.last_vision_time = self.get_clock().now()
        self.STALE_THRESHOLD_SEC = 2.0

        # --- 4. Navigation & State Machine Variables ---
        self.state = "SEARCH_STRAFE"
        self.real_scrolls_found = 0
        self.SEARCH_STRAFE_DIR = 1.0
        self.search_start_y = 0.0
        self.prev_error = 0.0

        # Odometry Tracking
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Non-blocking Bypass Sub-states
        self.bypass_substate = "INIT"
        self.bypass_start_x = 0.0
        self.bypass_start_y = 0.0

        # --- 5. Main Timer Loop (20Hz) ---
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Navigator Initialized: Processes two boxes/obstacles and stops!")

    # --- Callbacks ---
    def ultrasonic_cb(self, msg):
        self.raw_distances.append(msg.data)
        self.filtered_distance = sum(self.raw_distances) / len(self.raw_distances)
        self.last_ultra_time = self.get_clock().now()

    def vision_error_cb(self, msg):
        self.x_error = max(min(msg.data, 1.0), -1.0)
        self.last_vision_time = self.get_clock().now()

    def target_type_cb(self, msg):
        self.target_type = msg.data.lower().strip()
        self.last_vision_time = self.get_clock().now()

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    # --- Non-blocking Odometry Bypass / Action for ANY Cube or Obstacle ---
    def execute_odom_bypass(self):
        cmd = Twist()

        if self.bypass_substate == "INIT":
            self.bypass_start_x = self.robot_x
            self.bypass_start_y = self.robot_y
            self.bypass_substate = "BACK_OFF"
            self.get_logger().info(f" Backing off via ultrasonic...")

        elif self.bypass_substate == "BACK_OFF":
            dist = abs(self.robot_y - self.bypass_start_y)
            if dist < 0.2:
                cmd.linear.x = -self.MIN_SPEED_FLOOR
            else:
                self.bypass_start_y = self.robot_y
                self.bypass_substate = "STRAFE_SIDEWAYS"
                self.get_logger().info("Safe distance reached. Strafing sideways...")

        elif self.bypass_substate == "STRAFE_SIDEWAYS":
            dist = abs(self.robot_y - self.bypass_start_y)
            if dist < 0.40:
                cmd.linear.y = self.MAX_STRAFE_SPEED * 0.5 *self.SEARCH_STRAFE_DIR
            else:
                if self.target_type in ['real','fake']:
                    self.real_scrolls_found += 1
                    self.get_logger().info(f" Target/Obstacle detected (Count: {self.real_scrolls_found + 1})")
                self.bypass_substate = "INIT"
                
                if self.real_scrolls_found >= 2:
                    self.state = "STOPPED"
                    self.get_logger().info("Two items processed! Mission completed, stopping robot.")
                else:
                    self.state = "SEARCH_STRAFE"
                    self.get_logger().info("Bypassed. Resuming search...")

        self.cmd_pub.publish(cmd)

    # --- Main Control Loop ---
    def control_loop(self):
        cmd = Twist()

        # STATE 1: SEARCHING
        if self.state == "SEARCH_STRAFE":
            if self.target_type in ["real", "fake"]:
                self.get_logger().info(f"Target detected ({self.target_type})! Switching to track/bypass...")
                self.state = "TRACK_HOLONOMIC"
            
            elif self.filtered_distance <= self.STOP_DISTANCE:
                self.get_logger().warn("Wall/Obstacle detected via ultrasonic! Triggering bypass maneuver...")
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                self.cmd_pub.publish(cmd)
                
                # Allow the same bypass maneuver for obstacles or walls
                self.bypass_substate = "INIT"
                self.state = "BYPASS_FIRST"
            
            else:
                if abs(self.robot_y - self.search_start_y) > 1.5:
                    self.SEARCH_STRAFE_DIR *= -1.0
                    self.search_start_y = self.robot_y

                cmd.linear.x = self.MIN_SPEED_FLOOR
                cmd.linear.y = self.MAX_STRAFE_SPEED*0.5 * self.SEARCH_STRAFE_DIR
                self.cmd_pub.publish(cmd)

        # STATE 2: TRACKING / APPROACHING ANY CUBE
        elif self.state == "TRACK_HOLONOMIC":
            if self.target_type not in ["real", "fake"]:
                self.get_logger().warn("Lost target. Resuming search...")
                self.state = "SEARCH_STRAFE"
                return

            if self.filtered_distance <= self.STOP_DISTANCE:
                self.state = "BYPASS_FIRST"
                return

            # PD Control
            err_diff = self.x_error - self.prev_error
            strafe_corr = (self.Kp * self.x_error) + (self.Kd * err_diff)
            self.prev_error = self.x_error

            cmd.linear.y = -max(min(strafe_corr, self.MAX_STRAFE_SPEED), -self.MAX_STRAFE_SPEED)

            if self.filtered_distance < 40.0:
                speed = self.MAX_FORWARD_SPEED * (self.filtered_distance / 40.0)
            else:
                speed = self.MAX_FORWARD_SPEED

            cmd.linear.x = max(speed, self.MIN_SPEED_FLOOR)
            self.cmd_pub.publish(cmd)

        # STATE 3: BYPASS
        elif self.state == "BYPASS_FIRST":
            self.execute_odom_bypass()

        # STATE 4: STOPPED
        elif self.state == "STOPPED":
            self.cmd_pub.publish(Twist())

    def stop_robot(self):
        try:
            self.get_logger().info("Shutting down node. Zero velocity sent.")
            self.cmd_pub.publish(Twist())
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = AutoReturnWithTimeoutTriggerNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()