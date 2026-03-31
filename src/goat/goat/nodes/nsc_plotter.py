#!/usr/bin/env python3
"""
NSC control node data plotter.

Subscribes to /joint_states, /imu, /nsc_debug.
Accumulates data until --duration expires OR Ctrl-C is pressed,
then renders all plots at once.

Usage:
    ros2 run goat nsc_plotter
    ros2 run goat nsc_plotter --duration 15
    ros2 run goat nsc_plotter --duration 30 --save true
    ros2 launch goat nsc.launch.xml duration:=30.0 save:=true
"""
import argparse
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray

# ── joint metadata ─────────────────────────────────────────────────────────────
# ROS order: [hip_L, hip_R, thigh_L, thigh_R, knee_L, knee_R, wheel_L, wheel_R]
JOINT_NAMES = ['hip_L', 'hip_R', 'thigh_L', 'thigh_R',
               'knee_L', 'knee_R', 'wheel_L', 'wheel_R']

# Reference target positions in ROS order
# Source: TARGET_JOINT_ANGLE (PIN: hip_L, thigh_L, knee_L, wheel_L, hip_R, thigh_R, knee_R, wheel_R)
#                             = [0.0, 0.738, 1.462, 0.0, 0.0, -0.738, -1.462, 0.0]
TARGET_Q_ROS = [0.0, 0.0, 0.738, -0.738, 1.462, -1.462, 0.0, 0.0]

# PIN actuator-order → ROS order  (used to reorder /nsc_debug torque arrays)
# PIN: [hip_L(0), thigh_L(1), knee_L(2), wheel_L(3), hip_R(4), thigh_R(5), knee_R(6), wheel_R(7)]
# ROS: [hip_L(0), hip_R(1),   thigh_L(2), thigh_R(3), knee_L(4), knee_R(5), wheel_L(6), wheel_R(7)]
PIN_TO_ROS = [0, 4, 1, 5, 2, 6, 3, 7]

# ── /nsc_debug message layout (37 floats, torques in PIN order) ────────────────
DBG_THETA     = 0
DBG_THETA_DOT = 1
DBG_THETA_CMD = 2
DBG_L         = 3
DBG_WHEEL_TAU = 4
DBG_TAU_CMD   = slice(5,  13)
DBG_TAU_RNEA  = slice(13, 21)
DBG_TAU_PD    = slice(21, 29)
DBG_TAU_EXT   = slice(29, 37)
DBG_LEN       = 37


# ── data collection node ───────────────────────────────────────────────────────
class NSCPlotNode(Node):

    def __init__(self, duration: float):
        super().__init__('nsc_plotter')
        self.done = False
        self._t0 = self.get_clock().now().nanoseconds * 1e-9

        self.buf: dict[str, list] = {
            # /joint_states  (ROS order)
            't_js': [], 'q': [], 'v': [], 'tau_meas': [],
            # /nsc_debug  (scalar signals + (N,8) torques in ROS order)
            't_dbg': [],
            'theta': [], 'theta_dot': [], 'theta_cmd': [], 'L': [], 'wheel_tau': [],
            'tau_cmd': [], 'tau_rnea': [], 'tau_pd': [], 'tau_ext': [],
            # /imu  → euler angles
            't_imu': [], 'roll': [], 'pitch': [], 'yaw': [],
        }

        self.create_subscription(JointState,         '/joint_states', self._js_cb,  10)
        self.create_subscription(Float64MultiArray,  '/nsc_debug',    self._dbg_cb, 10)
        self.create_subscription(Imu,                '/imu',          self._imu_cb, 10)

        self.create_timer(duration, lambda: setattr(self, 'done', True))
        self.get_logger().info(
            f'[nsc_plotter] collecting for {duration:.1f} s  —  Ctrl-C to stop early and plot')

    # ── time helpers ──────────────────────────────────────────────────────────

    def _rel(self, stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9 - self._t0

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9 - self._t0

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState):
        t = self._rel(msg.header.stamp)
        self.buf['t_js'].append(t)
        self.buf['q'].append(list(msg.position))
        self.buf['v'].append(list(msg.velocity))
        self.buf['tau_meas'].append(list(msg.effort))

    def _dbg_cb(self, msg: Float64MultiArray):
        if len(msg.data) < DBG_LEN:
            return
        d = msg.data
        self.buf['t_dbg'].append(self._now())
        self.buf['theta'].append(float(d[DBG_THETA]))
        self.buf['theta_dot'].append(float(d[DBG_THETA_DOT]))
        self.buf['theta_cmd'].append(float(d[DBG_THETA_CMD]))
        self.buf['L'].append(float(d[DBG_L]))
        self.buf['wheel_tau'].append(float(d[DBG_WHEEL_TAU]))
        for key, slc in [('tau_cmd',  DBG_TAU_CMD),
                         ('tau_rnea', DBG_TAU_RNEA),
                         ('tau_pd',   DBG_TAU_PD),
                         ('tau_ext',  DBG_TAU_EXT)]:
            pin_vec = np.array(d[slc])
            self.buf[key].append(pin_vec[PIN_TO_ROS].tolist())

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x**2 + q.y**2)
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y**2 + q.z**2)
        self.buf['t_imu'].append(self._now())
        self.buf['roll'].append(math.atan2(sinr, cosr))
        self.buf['pitch'].append(math.asin(sinp))
        self.buf['yaw'].append(math.atan2(siny, cosy))

    # ── export ────────────────────────────────────────────────────────────────

    def arrays(self) -> dict:
        """Convert all buffers to numpy arrays (empty arrays for empty lists)."""
        return {k: np.array(v) if v else np.array([]) for k, v in self.buf.items()}


# ── shared subplot style ───────────────────────────────────────────────────────
def _style(ax, title: str, ylabel: str, xlabel: str = ''):
    ax.set_title(title, fontsize=9)
    ax.set_ylabel(ylabel, fontsize=8)
    if xlabel:
        ax.set_xlabel(xlabel, fontsize=8)
    ax.grid(True, linewidth=0.4, alpha=0.6)
    ax.tick_params(labelsize=7)


# ── plot functions ─────────────────────────────────────────────────────────────

def plot_joint_positions(data: dict):
    """Figure 1 — joint positions vs reference (q vs q_ref)."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return None

    fig, axes = plt.subplots(4, 2, figsize=(13, 10), sharex=True)
    fig.suptitle('Joint Positions  (blue = measured,  red dashed = reference)', fontsize=12)

    t = data['t_js']
    q = data['q']
    for i, name in enumerate(JOINT_NAMES):
        ax = axes[i // 2, i % 2]
        if len(t):
            ax.plot(t, q[:, i], linewidth=1.2, label='q')
            ax.axhline(TARGET_Q_ROS[i], color='tab:red', linestyle='--',
                       linewidth=0.9, label='q_ref')
        _style(ax, name, 'rad', xlabel='t [s]' if i >= 6 else '')
        ax.legend(fontsize=7, loc='upper right')

    fig.tight_layout()
    return fig


def plot_joint_velocities(data: dict):
    """Figure 2 — joint velocities."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return None

    fig, axes = plt.subplots(4, 2, figsize=(13, 10), sharex=True)
    fig.suptitle('Joint Velocities', fontsize=12)

    t = data['t_js']
    v = data['v']
    for i, name in enumerate(JOINT_NAMES):
        ax = axes[i // 2, i % 2]
        if len(t):
            ax.plot(t, v[:, i], linewidth=1.2, color='tab:green')
        _style(ax, name, 'rad/s', xlabel='t [s]' if i >= 6 else '')

    fig.tight_layout()
    return fig


def plot_torques(data: dict):
    """Figure 3 — torque breakdown per joint (cmd / rnea / pd / ext)."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return None

    fig, axes = plt.subplots(4, 2, figsize=(13, 12), sharex=True)
    fig.suptitle('Torque Breakdown  (cmd / rnea / pd / ext)', fontsize=12)

    t = data['t_dbg']
    components = [
        ('tau_cmd',  'tab:blue',   'solid',   'cmd'),
        ('tau_rnea', 'tab:orange', 'dashed',  'rnea'),
        ('tau_pd',   'tab:green',  'dotted',  'pd'),
        ('tau_ext',  'tab:red',    'dashdot', 'ext'),
    ]
    for i, name in enumerate(JOINT_NAMES):
        ax = axes[i // 2, i % 2]
        if len(t):
            for key, color, ls, label in components:
                arr = data[key]
                if len(arr):
                    ax.plot(t, arr[:, i], linewidth=1.0,
                            color=color, linestyle=ls, label=label)
        _style(ax, name, 'Nm', xlabel='t [s]' if i >= 6 else '')
        if i == 0:
            ax.legend(fontsize=7, loc='upper right')

    fig.tight_layout()
    return fig


def plot_wheel_balance(data: dict):
    """Figure 4 — wheel balance controller state."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return None

    fig, axes = plt.subplots(2, 2, figsize=(12, 7))
    fig.suptitle('Wheel Balance Control', fontsize=12)
    t = data['t_dbg']

    ax = axes[0, 0]
    if len(t):
        ax.plot(t, np.rad2deg(data['theta']),
                linewidth=1.3, label='θ actual')
        ax.plot(t, np.rad2deg(data['theta_cmd']),
                linewidth=1.0, linestyle='--', color='tab:red', label='θ cmd')
    _style(ax, 'Pitch angle  θ', 'deg', 't [s]')
    ax.legend(fontsize=8)

    ax = axes[0, 1]
    if len(t):
        ax.plot(t, np.rad2deg(data['theta_dot']),
                linewidth=1.3, color='tab:green')
    _style(ax, 'Pitch rate  θ̇', 'deg/s', 't [s]')

    ax = axes[1, 0]
    if len(t):
        ax.plot(t, data['L'], linewidth=1.3, color='tab:purple')
    _style(ax, 'Pendulum length  L', 'm', 't [s]')

    ax = axes[1, 1]
    if len(t):
        ax.plot(t, data['wheel_tau'], linewidth=1.3, color='tab:brown')
    _style(ax, 'Wheel torque  τ_wheel', 'Nm', 't [s]')

    fig.tight_layout()
    return fig


def plot_imu(data: dict):
    """Figure 5 — IMU orientation (Euler angles)."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return None

    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    fig.suptitle('IMU Orientation (Euler)', fontsize=12)

    t = data['t_imu']
    for ax, key, title in zip(axes,
                               ['roll', 'pitch', 'yaw'],
                               ['Roll', 'Pitch', 'Yaw']):
        if len(t):
            ax.plot(t, np.rad2deg(data[key]), linewidth=1.2)
        _style(ax, title, 'deg')
    axes[-1].set_xlabel('t [s]', fontsize=8)

    fig.tight_layout()
    return fig


def plot_all(data: dict, save: bool = False) -> list:
    """Generate all figures. Returns list of Figure objects."""
    entries = [
        ('joint_positions',  plot_joint_positions(data)),
        ('joint_velocities', plot_joint_velocities(data)),
        ('torques',          plot_torques(data)),
        ('wheel_balance',    plot_wheel_balance(data)),
        ('imu',              plot_imu(data)),
    ]
    figs = [(name, fig) for name, fig in entries if fig is not None]

    if save:
        for name, fig in figs:
            path = f'nsc_{name}.png'
            fig.savefig(path, dpi=150, bbox_inches='tight')
            print(f'  [nsc_plotter] saved → {path}')

    return [fig for _, fig in figs]


# ── entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    # Parse non-ROS arguments (ROS args start with --ros-args)
    parser = argparse.ArgumentParser(description='NSC control data plotter')
    parser.add_argument('--duration', type=float, default=30.0,
                        help='Data collection duration in seconds (default: 30)')
    parser.add_argument('--save', type=str, default='false',
                        help='Save figures as PNG files (true/false, default: false)')
    known, _ = parser.parse_known_args()
    save_flag = known.save.lower() == 'true'

    # Select matplotlib backend (fallback to Agg when no display, e.g. WSL2)
    import matplotlib
    try:
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
        plt.figure()
        plt.close()
    except Exception:
        matplotlib.use('Agg')
        if not save_flag:
            print('[nsc_plotter] WARNING: no display found, switching to Agg backend. '
                  'Re-run with --save true to write PNG files.')
            save_flag = True

    rclpy.init(args=args)
    node = NSCPlotNode(duration=known.duration)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    n_js  = len(node.buf['t_js'])
    n_dbg = len(node.buf['t_dbg'])
    node.get_logger().info(
        f'[nsc_plotter] collection done — '
        f'{n_js} joint_state samples,  {n_dbg} debug samples')

    data = node.arrays()
    node.destroy_node()
    rclpy.shutdown()

    import matplotlib.pyplot as plt
    figs = plot_all(data, save=save_flag)
    if figs:
        plt.show()
    else:
        print('[nsc_plotter] No data collected — nothing to plot.')


if __name__ == '__main__':
    main()
