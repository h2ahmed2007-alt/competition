#!/usr/bin/env python3
import collections
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String,Int32
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
        self.declare_parameter('space_wait_timeout_sec', 15.0) 

        self.Kp = self.get_parameter('kp').value
        self.Kd = self.get_parameter('kd').value
        self.MAX_FORWARD_SPEED = self.get_parameter('max_forward_speed').value
        self.MAX_STRAFE_SPEED = self.get_parameter('max_strafe_speed').value
        self.MIN_SPEED_FLOOR = self.get_parameter('min_speed_floor').value
        self.STOP_DISTANCE = self.get_parameter('stop_distance').value
        self.WAIT_TIMEOUT = self.get_parameter('space_wait_timeout_sec').value

        # --- 2. Publishers & Subscribers ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.create_subscription(Int32, '/ultrasonic_distance', self.ultrasonic_cb, 10)
        self.create_subscription(Float32, '/target_x_error', self.vision_error_cb, 10)
        self.create_subscription(String, '/target_type', self.target_type_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        self.create_subscription(String, '/start_trigger', self.trigger_cb, 10)

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

        # Odometry Tracking (Start point / Home coordinates)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.has_home_pose = False

        # Non-blocking Bypass Sub-states
        self.bypass_substate = "INIT"
        self.bypass_start_x = 0.0
        self.bypass_start_y = 0.0

        # Wait Timeout Timing Variable
        self.wait_start_time = None

        # --- 5. Main Timer Loop (20Hz) ---
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Navigator Initialized: Auto-returns home, waits for external space trigger or timeouts into auto-loop!")

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
        
        if not self.has_home_pose:
            self.start_x = self.robot_x
            self.start_y = self.robot_y
            self.has_home_pose = True
            self.get_logger().info(f"Home Position Registered: X={self.start_x:.2f}, Y={self.start_y:.2f}")

    
    def trigger_cb(self, msg):
        cmd_str = msg.data.lower().strip()
        if self.state == "WAITING_FOR_TRIGGER":
            self.get_logger().info(f"Received space trigger from external node ('{cmd_str}'). Resetting mission...")
            self.reset_mission()

    def reset_mission(self):
        self.get_logger().info(">>> RE-STARTING MISSION RUN <<<")
        self.real_scrolls_found = 0
        self.bypass_substate = "INIT"
        self.search_start_y = self.robot_y
        self.wait_start_time = None
        self.state = "SEARCH_STRAFE"

    # --- Non-blocking Odometry Bypass ---
    def execute_odom_bypass(self):
        cmd = Twist()

        if self.bypass_substate == "INIT":
            self.bypass_start_x = self.robot_x
            self.bypass_start_y = self.robot_y
            self.bypass_substate = "BACK_OFF"
            self.get_logger().info("Bypassing first REAL cube: Backing off...")

        elif self.bypass_substate == "BACK_OFF":
            dist = abs(self.robot_x - self.bypass_start_x)
            if dist < 0.20:
                cmd.linear.x = -self.MIN_SPEED_FLOOR
            else:
                self.bypass_start_y = self.robot_y
                self.bypass_substate = "STRAFE_SIDEWAYS"

        elif self.bypass_substate == "STRAFE_SIDEWAYS":
            dist = abs(self.robot_y - self.bypass_start_y)
            if dist < 0.15 or self.target_type=="real":
                cmd.linear.y = self.MAX_STRAFE_SPEED * self.SEARCH_STRAFE_DIR
            else:
                self.real_scrolls_found += 1
                self.bypass_substate = "INIT"
                self.state = "SEARCH_STRAFE"
                self.get_logger().info("First REAL cube bypassed. Resuming search...")

        self.cmd_pub.publish(cmd)

    # --- Return to Home Point ---
    def execute_return_home(self):
        cmd = Twist()

        dx = self.start_x - self.robot_x
        dy = self.start_y - self.robot_y
        dist_to_home = math.hypot(dx, dy)

        if dist_to_home <= 0.10:
            self.state = "WAITING_FOR_TRIGGER"
            self.wait_start_time = self.get_clock().now()
            self.get_logger().info(f"=== Reached Start Point! Waiting up to {self.WAIT_TIMEOUT}s for space signal on /start_trigger ===")
            self.cmd_pub.publish(Twist())
            return

        cmd.linear.x = max(min(dx * 0.8, self.MAX_FORWARD_SPEED), -self.MAX_FORWARD_SPEED)
        cmd.linear.y = max(min(dy * 0.8, self.MAX_STRAFE_SPEED), -self.MAX_STRAFE_SPEED)

        if abs(cmd.linear.x) < 0.05 and abs(dx) > 0.02:
            cmd.linear.x = math.copysign(self.MIN_SPEED_FLOOR, dx)
        if abs(cmd.linear.y) < 0.05 and abs(dy) > 0.02:
            cmd.linear.y = math.copysign(self.MIN_SPEED_FLOOR, dy)

        self.cmd_pub.publish(cmd)

    # --- Main Control Loop ---
    def control_loop(self):
        cmd = Twist()
        now = self.get_clock().now()
        
        
        
        if self.filtered_distance<=self.STOP_DISTANCE:
            
            
            
            if self.state=="TRACK_HOLONOMIC":
                if self.real_scrolls_found==0:
                    self.state="BYPASS_FRIST"
                else:
                    self.state="RETURN_HOME"
                return 
            
            
            self.cmd_pub.publish(Twist())   

        # Watchdog Safety Check
        if self.state in ["SEARCH_STRAFE", "TRACK_HOLONOMIC"]:
            if ((now - self.last_ultra_time).nanoseconds / 1e9 > self.STALE_THRESHOLD_SEC or
                (now - self.last_vision_time).nanoseconds / 1e9 > self.STALE_THRESHOLD_SEC):
                self.get_logger().warn("DATA STALE! Emergency Stop engaged.", throttle_duration_sec=2.0)
                self.cmd_pub.publish(Twist())
                return

        # STATE 1: SEARCHING
        if self.state == "SEARCH_STRAFE":
            if self.target_type == "real":
                self.state = "TRACK_HOLONOMIC"
                self.get_logger().info("REAL target detected! Tracking...")
            else:
                if self.target_type == "fake":
                    self.get_logger().info("Ignoring FAKE target...", throttle_duration_sec=3.0)

                if abs(self.robot_y - self.search_start_y) > 1.5:
                    self.SEARCH_STRAFE_DIR *= -1.0
                    self.search_start_y = self.robot_y

                cmd.linear.x = self.MIN_SPEED_FLOOR
                cmd.linear.y = 0.35 * self.SEARCH_STRAFE_DIR
                self.cmd_pub.publish(cmd)

        # STATE 2: TRACKING
        elif self.state == "TRACK_HOLONOMIC":
            if self.target_type != "real":
                self.get_logger().warn("Lost REAL target. Resuming search...")
                self.state = "SEARCH_STRAFE"
                return

            if self.filtered_distance <= self.STOP_DISTANCE:
                if self.real_scrolls_found == 0:
                    self.state = "BYPASS_FIRST"
                else:
                    self.state = "RETURN_HOME"
                    self.get_logger().info("All REAL targets processed. Returning to Start Point...")
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

        # STATE 4: RETURN TO HOME
        elif self.state == "RETURN_HOME":
            self.execute_return_home()

        # STATE 5: WAITING FOR EXTERNAL TRIGGER OR TIMEOUT AUTO-RESTART
        elif self.state == "WAITING_FOR_TRIGGER":
            self.cmd_pub.publish(Twist())  

            if self.wait_start_time is not None:
                elapsed_wait = (now - self.wait_start_time).nanoseconds / 1e9

                # (Timeout)
                if elapsed_wait >= self.WAIT_TIMEOUT:
                    self.get_logger().warn(f"Wait timeout ({self.WAIT_TIMEOUT}s) reached without external space signal. Auto-looping now!")
                    self.reset_mission()

    def stop_robot(self):
        self.get_logger().info("Shutting down node. Zero velocity sent.")
        self.cmd_pub.publish(Twist())

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