#!/usr/bin/env python3
"""
Phase 3: Sim Control Node — Classical control pipeline over ROS2 topics.

Subscribes to /joint_states and /imu from Isaac Sim OmniGraph,
runs the PD (joints) + PI (wheels) control pipeline from goat_sim,
and publishes torque commands to /joint_command.

Usage:
    # Default config path (from installed share directory)
    ros2 run goat control_node

    # Custom config
    ros2 run goat control_node --ros-args -p config:=/path/to/sim_goat_config.yaml

    # NSC balancing mode (uses pitch-based wheel control)
    ros2 run goat control_node --ros-args -p mode:=nsc

Run Isaac Sim (00_standalone_scene.py) on Windows first.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState, Imu

from goat.goat_sim.control.control_pipeline import ControlPipeline, ControlTargets
from goat.goat_sim.estimation.state_types import RobotState, ImuState
from goat.goat_sim.model.model_builder import build_control_pipeline_from_yaml


# ---------------------------------------------------------------------------
# Default paths
# ---------------------------------------------------------------------------
def _default_config_path() -> str:
    return str(
        Path(get_package_share_directory('goat')) / 'config' / 'sim_goat_config.yaml'
    )


# Control loop rate (Hz)
_CONTROL_RATE_HZ = 200


class SimControlNode(Node):
    """ROS2 node that runs the GOAT classical control pipeline.

    Mirrors references/control_node but uses standard sensor_msgs
    instead of custom motor_interfaces messages.
    """

    def __init__(self, config_path: str, mode: str = "action"):
        super().__init__("sim_control_node")
        self.mode = mode

        # --- Build model + pipeline from YAML ---
        self.get_logger().info(f"Loading config: {config_path}")
        self.goat_model, self.pipeline = build_control_pipeline_from_yaml(config_path)
        self.pipeline.reset()

        self.num_joints = self.goat_model.num_joints
        self.joint_names = list(self.goat_model.joint_names)
        self.natural_pos = list(self.goat_model.natural_joint_position)

        self.get_logger().info(f"Joints ({self.num_joints}): {self.joint_names}")
        self.get_logger().info(f"Natural position: {self.natural_pos}")
        self.get_logger().info(f"Control mode: {self.mode}")

        # --- Mutable state (updated by callbacks) ---
        self._robot_state: RobotState | None = None
        self._imu_state: ImuState | None = None
        self._joint_name_to_idx: dict[str, int] = {}  # populated on first /joint_states

        # Timing
        self._last_control_time: float | None = None

        # Diagnostics
        self._js_recv_count = 0
        self._imu_recv_count = 0
        self._cmd_pub_count = 0

        # --- QoS: match Isaac Sim defaults ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Subscribers ---
        self.create_subscription(JointState, "/joint_states", self._joint_states_cb, qos)
        self.create_subscription(Imu, "/imu", self._imu_cb, qos)

        # --- Publisher ---
        self._cmd_pub = self.create_publisher(JointState, "/joint_command", 10)

        # --- Control loop timer ---
        dt = 1.0 / _CONTROL_RATE_HZ
        self._control_timer = self.create_timer(dt, self._control_loop)

        self.get_logger().info(f"Control loop: {_CONTROL_RATE_HZ} Hz (dt={dt:.4f}s)")
        self.get_logger().info("Waiting for /joint_states and /imu ...")

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------
    def _joint_states_cb(self, msg: JointState):
        """Update robot state from /joint_states topic."""
        self._js_recv_count += 1
        names = list(msg.name)

        # Build name->index mapping on first message
        if not self._joint_name_to_idx:
            self._joint_name_to_idx = {n: i for i, n in enumerate(names)}
            self.get_logger().info(f"Joint name mapping: {self._joint_name_to_idx}")

        # Reorder sensor data to match goat_model joint order
        n = self.num_joints
        position = [0.0] * n
        velocity = [0.0] * n
        effort = [0.0] * n

        for cfg_idx, cfg_name in enumerate(self.joint_names):
            sim_idx = self._joint_name_to_idx.get(cfg_name)
            if sim_idx is not None and sim_idx < len(msg.position):
                position[cfg_idx] = msg.position[sim_idx]
                if sim_idx < len(msg.velocity):
                    velocity[cfg_idx] = msg.velocity[sim_idx]
                if sim_idx < len(msg.effort):
                    effort[cfg_idx] = msg.effort[sim_idx]

        self._robot_state = RobotState(
            joint_names=self.joint_names,
            joint_position_rad=position,
            joint_velocity_rad_per_sec=velocity,
            natural_joint_position=list(self.natural_pos),
            joint_effort_like=effort,
            motor_temperature_c=[0.0] * n,
            motor_error_flags=[0] * n,
            motor_operating_state=[0] * n,
            imu_state=self._imu_state,
            timestamp_sec=time.time(),
        )

        # Periodic log
        if self._js_recv_count == 1:
            self.get_logger().info("[/joint_states] First message received!")
        if self._js_recv_count % (2 * _CONTROL_RATE_HZ) == 0:
            np.set_printoptions(precision=4, suppress=True)
            self.get_logger().info(
                f"[state] pos={np.array(position)}  vel={np.array(velocity)}"
            )

    def _imu_cb(self, msg: Imu):
        """Update IMU state from /imu topic.

        ROS2 sensor_msgs/Imu quaternion order: (x, y, z, w)
        Internal ImuState expects: (w, x, y, z) — reorder here.
        """
        self._imu_recv_count += 1
        o = msg.orientation
        av = msg.angular_velocity
        la = msg.linear_acceleration

        self._imu_state = ImuState(
            orientation_quat_w=o.w,
            orientation_quat_x=o.x,
            orientation_quat_y=o.y,
            orientation_quat_z=o.z,
            gyroscope_x=av.x,
            gyroscope_y=av.y,
            gyroscope_z=av.z,
            acceleration_x=la.x,
            acceleration_y=la.y,
            acceleration_z=la.z,
            magnetic_field_x=0.0,
            magnetic_field_y=0.0,
            magnetic_field_z=0.0,
            sensor_time_ms=time.time() * 1000.0,
        )

        # Attach to current robot_state if it exists
        if self._robot_state is not None:
            self._robot_state.imu_state = self._imu_state

        if self._imu_recv_count == 1:
            self.get_logger().info("[/imu] First message received!")

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def _control_loop(self):
        """200 Hz control timer callback."""
        if self._robot_state is None:
            return  # No sensor data yet

        # Compute dt
        now = time.time()
        if self._last_control_time is None:
            self._last_control_time = now
            return  # Skip first tick (no valid dt)
        dt = now - self._last_control_time
        self._last_control_time = now

        # Clamp dt to avoid spikes
        dt = max(1e-4, min(dt, 0.05))

        # Ensure IMU state is attached
        if self._imu_state is not None:
            self._robot_state.imu_state = self._imu_state

        if self.mode == "nsc" and self._robot_state.imu_state is not None:
            # NSC mode: natural standing with pitch-based wheel balancing
            safe_torque, _ = self.pipeline.compute_natural_torque(
                self._robot_state, dt
            )
        else:
            # Action mode: delta=0, wheel_speed=0 (hold natural pose, wheels stopped)
            targets = ControlTargets(
                desired_joint_delta_position_rad=np.zeros(self.num_joints),
                desired_wheel_speed_rad_per_sec=np.zeros(self.num_joints),
            )
            safe_torque, _, _ = self.pipeline.compute_control(
                self._robot_state, targets, dt
            )

        # Publish /joint_command
        self._publish_command(safe_torque)

    def _publish_command(self, torques: np.ndarray):
        """Publish torque command as JointState with name + effort."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = []
        msg.velocity = []
        msg.effort = [float(t) for t in torques]
        self._cmd_pub.publish(msg)

        self._cmd_pub_count += 1
        if self._cmd_pub_count == 1:
            self.get_logger().info(
                f"[/joint_command] First command published! "
                f"effort={[f'{t:.4f}' for t in torques]}"
            )
        if self._cmd_pub_count % (5 * _CONTROL_RATE_HZ) == 0:
            np.set_printoptions(precision=4, suppress=True)
            self.get_logger().info(
                f"[cmd] effort={np.array([float(t) for t in torques])}"
            )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="GOAT Sim Control Node")
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="Path to sim_goat_config.yaml (default: installed share directory)",
    )
    parser.add_argument(
        "--mode",
        type=str,
        choices=["action", "nsc"],
        default="action",
        help="Control mode: 'action' (delta=0 hold) or 'nsc' (natural standing w/ balancing)",
    )
    args = parser.parse_args()

    config_path = args.config if args.config is not None else _default_config_path()

    if not os.path.isfile(config_path):
        print(f"ERROR: Config file not found: {config_path}")
        sys.exit(1)

    rclpy.init()
    node = SimControlNode(config_path=config_path, mode=args.mode)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
