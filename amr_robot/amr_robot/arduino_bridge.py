#!/usr/bin/env python3
"""
arduino_bridge.py
─────────────────
Optimized ROS2 ↔ Arduino serial bridge.
Handles buffer clearing to prevent odometry lag and ensures 
time-sync compatibility with SLAM Toolbox.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import math

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # ── Parameters ──────────────────────────────────────
        self.declare_parameter('serial_port',   '/dev/arduino_mega')
        self.declare_parameter('baud_rate',     115200)
        self.declare_parameter('wheel_base',    0.165)   # Physical distance between wheels
        self.declare_parameter('publish_rate',  20.0)    # Rate in Hz

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.wheel_base = self.get_parameter('wheel_base').value

        # ── Serial Setup ────────────────────────────────────
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            
            # --- THE HANDSHAKE FIX ---
            import time
            self.ser.setDTR(False) # Drop DTR to reset Arduino
            time.sleep(1)
            self.ser.reset_input_buffer()
            self.ser.setDTR(True)  # Bring DTR back up
            time.sleep(2)          # Wait 2 seconds for Arduino to finish booting
            # -------------------------
            
            self.get_logger().info(f'Connected to Arduino on {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Could not open serial port: {e}')
            raise SystemExit(1)

        # ── ROS2 Publishers & Subscribers ───────────────────
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # Timer matches the requested publish rate
        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self.update)

        self.get_logger().info('Bridge Ready: /odom active, /cmd_vel listening.')

    def cmd_vel_cb(self, msg: Twist):
        """ Translates ROS Twist to Arduino CMD format. """
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive inverse kinematics
        vL = v - (w * self.wheel_base / 2.0)
        vR = v + (w * self.wheel_base / 2.0)

        # Send command to Arduino
        cmd = f'CMD,{vL:.4f},{vR:.4f}\n'
        try:
            self.ser.write(cmd.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    def update(self):
        """ Reads the most recent ODO packet from serial buffer. """
        try:
            if self.ser.in_waiting == 0:
                return

            # Read all bytes currently in the buffer to prevent lag
            raw_data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='replace')
            lines = raw_data.split('\r\n')

            # Find the latest complete line starting with "ODO,"
            target_line = None
            for line in reversed(lines):
                if line.startswith('ODO,'):
                    target_line = line
                    break

            if not target_line:
                return

            # Format: ODO,x,y,theta_deg,vL,vR,encL,encR
            parts = target_line.split(',')
            if len(parts) < 6:
                return

            x         = float(parts[1])
            y         = float(parts[2])
            theta_deg = float(parts[3])
            vL        = float(parts[4])
            vR        = float(parts[5])

            self.publish_data(x, y, theta_deg, vL, vR)

        except Exception as e:
            self.get_logger().debug(f'Read error: {e}')

    def publish_data(self, x, y, theta_deg, vL, vR):
        """ Computes kinematics and publishes Odom topic + TF. """
        theta = math.radians(theta_deg)
        now = self.get_clock().now().to_msg()
        
        # Calculate velocities for the odom message
        v_body = (vL + vR) / 2.0
        w_body = (vR - vL) / self.wheel_base

        # Quaternion from yaw
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        # 1. Publish Transform: odom -> base_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # 2. Publish Odometry Message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v_body
        odom.twist.twist.angular.z = w_body
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
