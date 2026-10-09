"""
Teleoperation bridge node:
Subscribes to standard geometry_msgs/Twist on /cmd_vel (from teleop_twist_keyboard or joy)
and translates it to /throttle (Float32 in [-1.0, 1.0]) and /steer (Float32 in radians).

Supports two progression phases:
- Phase 1 (Milestone 3): Open-loop feedforward mapping with a safety watchdog timer.
- Phase 2 (Milestone 4): Closed-loop speed regulation using PIDLongitudinalController.
"""

import numpy as np  # noqa: F401
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# ==============================================================================
# Phase 2 (Milestone 4): Uncomment these imports when upgrading to cruise control
# ==============================================================================
from nav_msgs.msg import Odometry
from bicycle_control.longitudinal_pid import PIDLongitudinalController


class TeleopBridge(Node):
    def __init__(self):
        super().__init__('teleop_bridge')
        self.get_logger().info('Teleoperation Bridge Node Initialized')

        # Parameters
        self.declare_parameter('max_linear_vel', 5.0)     # m/s corresponding to full 1.0 throttle
        self.declare_parameter('max_angular_vel', 1.0)    # rad/s corresponding to full steering
        self.declare_parameter('max_steer_rad', 0.610865)  # radians (~35 degrees)
        self.declare_parameter('auto_zero_timeout', 0.5)  # seconds before zeroing commands
        self.declare_parameter('use_cruise_control', False)  # Enable in Milestone 4.2

        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        self.max_steer_rad = float(self.get_parameter('max_steer_rad').value)
        self.auto_zero_timeout = float(self.get_parameter('auto_zero_timeout').value)
        self.use_cruise_control = bool(self.get_parameter('use_cruise_control').value)

        # Publishers (10 Hz rate per assignment specification)
        self.throttle_pub = self.create_publisher(Float32, '/throttle', 10)
        self.steer_pub = self.create_publisher(Float32, '/steer', 10)

        # Subscribers
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        self.current_throttle = 0.0
        self.current_steer = 0.0
        self.target_vel = 0.0
        self.current_vel = 0.0
        self.last_cmd_time = self.get_clock().now()

        # TODO: Phase 2 (Milestone 4.2) — Closed-Loop Cruise Control Setup
        # This allows the car to automatically hold a steady speed instead of requiring manual throttle.
        # Initialize the PID speed controller and subscribe to odometry data.

        self.pid = PIDLongitudinalController(dt = 0.1)
        self.current_vel = 0.0
        self.odem_sub  = self.create_subscription(Odometry , "/state" , self.odom_callback,10)

        # Publish loop at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_commands)

    def odom_callback(self, msg: Odometry):
        self.current_vel = float(msg.twist.twist.linear.x)
    #     """Milestone 4.2: Extracts vehicle forward speed from /state odometry."""

    def cmd_callback(self, msg: Twist):
        """Translates Twist linear.x to throttle [-1, 1] and angular.z into steering (rad)."""
        ## now we get the linear/angluar commands from twist
        self.last_cmd_time = self.get_clock().now()
        # linearvel --> cmd from keyboard while l_velocity is the throttle percentage
        linearvel = float(msg.linear.x)
        self.target_vel = linearvel
        if self.max_linear_vel > 0.0:
            
           l_velocity = linearvel / self.max_linear_vel
        else:
            l_velocity = 0.0
        ## now we put the throttle into the input and make sure we cap it between -1 and 1 
        self.current_throttle = np.clip(l_velocity , -1.0 , 1.0)
        steer_angle = float(msg.angular.z)/self.max_angular_vel*self.max_steer_rad
        ## we put put the steering into the input and make sure it's inbetween min and max steering angles
        self.current_steer= np.clip ( steer_angle , -self.max_steer_rad , self.max_steer_rad)

        # TODO: Milestone 3.1 — Teleoperation Command Mapping
        # This connects user inputs (keyboard/joystick) to the car's physical actuators.
        # Map the incoming Twist linear/angular commands to throttle and steering.
    
    def check_watchdog (self):
        ## we need to calculate the time passed since the last command given so we know if watchdog should activate
        duration_since_last_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if duration_since_last_cmd > self.auto_zero_timeout:
            self.current_throttle = 0.0
            self.current_steer = 0.0
            self.target_vel = 0.0

    def publish_commands(self):
        self.check_watchdog()

        if self.use_cruise_control:
            self.current_throttle = self.pid.compute(self.target_vel,self.current_vel)

        throttle_msg = Float32()
        throttle_msg.data = float(self.current_throttle)
        self.throttle_pub.publish(throttle_msg)

        steer_msg = Float32()
        steer_msg.data = float(self.current_steer)
        self.steer_pub.publish(steer_msg)

       

        """Periodically publishes throttle and steering commands at 10 Hz."""
        # TODO: Milestone 3.2 — Safety Watchdog & Command Publishing
        # This prevents the car from running away if the user's connection drops.
        # Publish the commands, or zero them out if the last command is too old.
        


def main(args=None):
    rclpy.init(args=args)
    bridge = TeleopBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
