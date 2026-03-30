#!/usr/bin/env python3
"""
Phase 1: ROS2 Topic I/O verification script for Isaac Sim GOAT simulation.

Run Isaac Sim (00_standalone_scene.py) on Windows first, then run this on WSL2.

Usage:
    python 02_test_topic_io.py              # Listen-only mode (default)
    python 02_test_topic_io.py --test-single # Send effort to a single joint
    python 02_test_topic_io.py --test-all    # Send effort to all 8 joints
"""
import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState, Imu


class TopicIOTester(Node):
    def __init__(self, mode: str = "listen"):
        super().__init__("topic_io_tester")
        self.mode = mode

        # --- State tracking ---
        self.joint_states_received = False
        self.imu_received = False
        self.joint_names = []

        # QoS: match Isaac Sim defaults
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Subscribers ---
        self.joint_states_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_states_callback, qos
        )
        self.imu_sub = self.create_subscription(
            Imu, "/imu", self.imu_callback, qos
        )

        # --- Publisher (for test modes) ---
        self.joint_cmd_pub = self.create_publisher(JointState, "/joint_command", 10)

        # --- Test timer (starts after receiving first joint_states) ---
        self.test_started = False
        self.test_step = 0
        if mode != "listen":
            self.test_timer = self.create_timer(2.0, self.run_test_step)

        self.get_logger().info(f"TopicIOTester started in '{mode}' mode")
        self.get_logger().info("Waiting for /joint_states and /imu topics...")

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------
    def joint_states_callback(self, msg: JointState):
        if not self.joint_states_received:
            self.joint_states_received = True
            self.joint_names = list(msg.name)
            self.get_logger().info("=" * 60)
            self.get_logger().info("[/joint_states] FIRST MESSAGE RECEIVED")
            self.get_logger().info(f"  Joint count : {len(msg.name)}")
            self.get_logger().info(f"  Joint names : {list(msg.name)}")
            self.get_logger().info(f"  Position len: {len(msg.position)}")
            self.get_logger().info(f"  Velocity len: {len(msg.velocity)}")
            self.get_logger().info(f"  Effort len  : {len(msg.effort)}")
            self.get_logger().info("=" * 60)

        # Print periodic updates (every ~2 seconds at 200Hz = every 400 msgs)
        pos = np.array(msg.position)
        vel = np.array(msg.velocity)
        eff = np.array(msg.effort)

        # Use a simple counter for periodic logging
        if not hasattr(self, "_js_count"):
            self._js_count = 0
        self._js_count += 1
        if self._js_count % 100 == 0:
            np.set_printoptions(precision=4, suppress=True)
            self.get_logger().info(
                f"[/joint_states] pos(rad)={pos}  vel(rad/s)={vel}"
            )

    def imu_callback(self, msg: Imu):
        if not self.imu_received:
            self.imu_received = True
            o = msg.orientation
            av = msg.angular_velocity
            la = msg.linear_acceleration
            self.get_logger().info("=" * 60)
            self.get_logger().info("[/imu] FIRST MESSAGE RECEIVED")
            self.get_logger().info(
                f"  Orientation (xyzw): [{o.x:.4f}, {o.y:.4f}, {o.z:.4f}, {o.w:.4f}]"
            )
            self.get_logger().info(
                f"  Angular vel (rad/s): [{av.x:.4f}, {av.y:.4f}, {av.z:.4f}]"
            )
            self.get_logger().info(
                f"  Linear acc (m/s²) : [{la.x:.4f}, {la.y:.4f}, {la.z:.4f}]"
            )
            self.get_logger().info("=" * 60)

        if not hasattr(self, "_imu_count"):
            self._imu_count = 0
        self._imu_count += 1
        if self._imu_count % 100 == 0:
            o = msg.orientation
            av = msg.angular_velocity
            self.get_logger().info(
                f"[/imu] quat(xyzw)=[{o.x:.4f},{o.y:.4f},{o.z:.4f},{o.w:.4f}]  "
                f"gyro=[{av.x:.4f},{av.y:.4f},{av.z:.4f}]"
            )

    # ------------------------------------------------------------------
    # Test command publisher
    # ------------------------------------------------------------------
    def publish_joint_command(self, names: list, efforts: list):
        """Publish a /joint_command message with given joint names and effort values."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = []
        msg.velocity = []
        msg.effort = efforts
        self.joint_cmd_pub.publish(msg)
        self.get_logger().info(
            f"[/joint_command] Published: {dict(zip(names, efforts))}"
        )

    def run_test_step(self):
        """Timer callback for test modes."""
        if not self.joint_states_received:
            self.get_logger().warn("Waiting for /joint_states before sending commands...")
            return

        if self.mode == "test-single":
            self._run_single_joint_test()
        elif self.mode == "test-all":
            self._run_all_joints_test()

    def _run_single_joint_test(self):
        """Send effort to one joint at a time, cycling through all joints."""
        if self.test_step >= len(self.joint_names):
            self.get_logger().info("Single-joint test complete. All joints tested.")
            self.test_timer.cancel()
            return

        joint_name = self.joint_names[self.test_step]
        effort = 1.0  # Small test torque (Nm)
        self.get_logger().info(f"--- Test step {self.test_step}: {joint_name} = {effort} Nm ---")
        self.publish_joint_command([joint_name], [effort])
        self.test_step += 1

    def _run_all_joints_test(self):
        """Send effort to all 8 joints simultaneously."""
        if self.test_step >= 3:
            self.get_logger().info("All-joints test complete.")
            self.test_timer.cancel()
            return

        # Alternate between positive and zero effort
        if self.test_step % 2 == 0:
            efforts = [0.5] * len(self.joint_names)
            self.get_logger().info(f"--- Test step {self.test_step}: ALL joints = 0.5 Nm ---")
        else:
            efforts = [0.0] * len(self.joint_names)
            self.get_logger().info(f"--- Test step {self.test_step}: ALL joints = 0.0 Nm ---")

        self.publish_joint_command(list(self.joint_names), efforts)
        self.test_step += 1


def main():
    parser = argparse.ArgumentParser(description="GOAT Isaac Sim Topic I/O Tester")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--test-single", action="store_true", help="Test single joint effort one by one")
    group.add_argument("--test-all", action="store_true", help="Test all joints effort simultaneously")
    args = parser.parse_args()

    if args.test_single:
        mode = "test-single"
    elif args.test_all:
        mode = "test-all"
    else:
        mode = "listen"

    rclpy.init()
    node = TopicIOTester(mode=mode)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
