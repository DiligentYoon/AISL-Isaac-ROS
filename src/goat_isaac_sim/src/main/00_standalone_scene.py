import os
import time
import numpy as np

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
import omni.timeline
import isaacsim.core.utils.stage as stage_utils
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.api import SimulationContext
from isaacsim.core.prims import Articulation, XFormPrim
from isaacsim.core.experimental.prims import Articulation as ExperimentalArticulation


class StandaloneEnv:
    def __init__(
        self,
        usd_mode: str,
        usd_path: str,
        robot_prim_path: str,
        physics_dt: float = 1 / 200,
        decimation: int = 4,
    ):
        self.usd_mode = usd_mode
        self.usd_path = usd_path
        self.robot_prim_path = robot_prim_path
        self.physics_dt = physics_dt
        self.decimation = decimation

        self.simulation_context = None
        self.robot = None
        self.robot_xform = None
        self._step_count = 0

        # rclpy references — populated in load_scene() after ros2.bridge is enabled
        self._rclpy = None
        self._ros_node = None
        self._next_step_time = None  # absolute wall-clock target for drift compensation
        self._command_received = False  # set True on first /commands message

        self.default_root_pos = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        self.default_root_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.default_joint_pos = None
        self.default_joint_vel = None

        # joint-type friction table
        self.joint_friction_cfg = {
            "hip": {
                "static": 0.12,
                "dynamic": 5.646268e-02,
                "viscous": 3.190248e-01,
            },
            "thigh": {
                "static": 0.12,
                "dynamic": 5.646268e-02,
                "viscous": 3.190248e-01,
            },
            "knee": {
                "static": 0.12,
                "dynamic": 5.373143e-02,
                "viscous": 8.441387e-02,
            },
            "wheel": {
                "static": 0.07,
                "dynamic": 3.218126e-02,
                "viscous": 1.715931e-02,
            },
        }


    def load_scene(self):
        enable_extension("isaacsim.ros2.bridge")
        simulation_app.update()

        # rclpy must be imported after ros2.bridge extension is enabled.
        # This is the Omniverse-compiled rclpy, not the system-level one.
        import rclpy
        from sensor_msgs.msg import JointState
        
        # rclpy initialization and ROS node creation
        self._rclpy = rclpy
        rclpy.init()
        self._ros_node = rclpy.create_node("isaac_goat_sim")

        # Subscribe to /commands to detect the first control input.
        self._ros_node.create_subscription(
            JointState, "/commands", self._on_first_command, 10
        )

        ok = stage_utils.open_stage(self.usd_path)
        if not ok:
            raise RuntimeError(f"Failed to open USD stage: {self.usd_path}")

        while stage_utils.is_stage_loading():
            simulation_app.update()
            time.sleep(0.01)

        stage = stage_utils.get_current_stage()
        if stage is None:
            raise RuntimeError("Current stage is None after opening USD.")

        # SimulationContext manages both physics and rendering dt.
        # rendering_dt = physics_dt * decimation → render every N physics steps.
        self.simulation_context = SimulationContext(
            physics_dt=self.physics_dt,
            rendering_dt=self.physics_dt * self.decimation,
            stage_units_in_meters=1.0,
        )

        # wrappers
        self.robot = Articulation(self.robot_prim_path)
        self.robot_xform = XFormPrim(self.robot_prim_path)
        self.robot_view = ExperimentalArticulation(self.robot_prim_path)


    def initialize_handles(self):
        # initialize physics and start simulation
        self.simulation_context.initialize_physics()
        self.simulation_context.play()
        simulation_app.update()

        self.robot.initialize()
        self.apply_joint_friction_properties()


    def _get_joint_names(self):
        if hasattr(self.robot, "dof_names"):
            return list(self.robot.dof_names)
        if hasattr(self.robot, "joint_names"):
            return list(self.robot.joint_names)
        raise AttributeError("Could not find joint name list. Check self.robot.dof_names or self.robot.joint_names.")


    def _classify_joint_type(self, joint_name: str) -> str:
        name = joint_name.lower()

        if "wheel" in name:
            return "wheel"
        if "knee" in name:
            return "knee"
        if "thigh" in name:
            return "thigh"
        if "hip" in name:
            return "hip"

        raise ValueError(f"Unknown joint type for joint '{joint_name}'. Add a rule in _classify_joint_type().")


    def apply_joint_friction_properties(self):

        joint_names = self._get_joint_names()
        num_dof = len(joint_names)

        static_f = np.zeros((1, num_dof), dtype=np.float32)
        dynamic_f = np.zeros((1, num_dof), dtype=np.float32)
        viscous_f = np.zeros((1, num_dof), dtype=np.float32)

        for i, joint_name in enumerate(joint_names):
            joint_type = self._classify_joint_type(joint_name)
            cfg = self.joint_friction_cfg[joint_type]

            static_f[0, i] = cfg["static"]
            dynamic_f[0, i] = cfg["dynamic"]
            viscous_f[0, i] = cfg["viscous"]

        for i, name in enumerate(joint_names):
            print(
                f"  {i:02d} | {name:20s} | "
                f"static={static_f[0, i]:.6f}, "
                f"dynamic={dynamic_f[0, i]:.6f}, "
                f"viscous={viscous_f[0, i]:.6f}"
            )
    
        self.robot_view.set_dof_friction_properties(
                static_frictions=static_f,
                dynamic_frictions=dynamic_f,
                viscous_frictions=viscous_f,
            )
        
        print("Applied friction via experimental articulation wrapper")


    def set_default_root_state(self, position, orientation):
        self.default_root_pos = np.array(position, dtype=np.float32).copy()
        self.default_root_quat = np.array(orientation, dtype=np.float32).copy()


    def set_default_joint_state(self, joint_pos, joint_vel=None):
        self.default_joint_pos = np.array(joint_pos, dtype=np.float32).copy()
        if joint_vel is None:
            joint_vel = np.zeros_like(self.default_joint_pos)
        self.default_joint_vel = np.array(joint_vel, dtype=np.float32).copy()


    def reset(self):
        # Stop physics before writing state to avoid instability.
        self.simulation_context.stop()

        # root pose reset : Fixed base don't use
        if self.usd_mode == "Floating":
            self.robot.set_default_state(positions=self.default_root_pos[None, :], orientations=self.default_root_quat[None, :])

        # joint reset
        self.robot.set_joints_default_state(positions=self.default_joint_pos, velocities=self.default_joint_vel)

        # update reset command
        self.simulation_context.play()
        simulation_app.update()
        
        self.robot.initialize()
        self.apply_joint_friction_properties()
        self.robot.post_reset()

        self.simulation_context.step(render=True)
        self._step_count = 0
        self._next_step_time = time.perf_counter()


    def _on_first_command(self, msg):
        """Callback for /commands — only used to flip the ready flag."""
        if not self._command_received:
            self._command_received = True


    def wait_for_first_command(self):
        """Block until the first /commands message is received.

        Physics is paused during the wait — the robot stays frozen
        in its reset pose.  While waiting, a manual rclpy publisher
        sends /joint_states so the control node can bootstrap and
        send /commands back (avoiding deadlock).
        After the first command arrives, the manual publisher is
        destroyed and Action Graph takes over /joint_states publishing.
        """
        self.simulation_context.pause()

        # Manual /joint_states publisher to break the deadlock
        from sensor_msgs.msg import JointState, Imu
        from nav_msgs.msg import Odometry


        js_pub = self._ros_node.create_publisher(JointState, "/sim_joint_states", 10)
        im_pub = self._ros_node.create_publisher(Imu, "/sim_imu", 10)
        od_pub = self._ros_node.create_publisher(Odometry, "/odom", 10)
        joint_names = self._get_joint_names()

        print("[StandaloneEnv] Waiting for /commands ...")
        while not self._command_received:
            # Publish frozen joint state so the control node can start
            js_msg = JointState()
            js_msg.header.stamp = self._ros_node.get_clock().now().to_msg()
            js_msg.name = joint_names
            js_msg.position = self.default_joint_pos.tolist()
            js_msg.velocity = (self.default_joint_vel.tolist()
                               if self.default_joint_vel is not None
                               else [0.0] * len(joint_names))
            js_msg.effort = [0.0] * len(joint_names)
            js_pub.publish(js_msg)

            im_msg = Imu()
            im_msg.header.stamp = self._ros_node.get_clock().now().to_msg()
            im_msg.header.frame_id = "imu_link"
            im_msg.orientation.x = 0.0
            im_msg.orientation.y = 0.0
            im_msg.orientation.z = 0.0
            im_msg.orientation.w = 1.0
            im_msg.angular_velocity.x = 0.0
            im_msg.angular_velocity.y = 0.0
            im_msg.angular_velocity.z = 0.0
            im_msg.linear_acceleration.x = 0.0
            im_msg.linear_acceleration.y = 0.0
            im_msg.linear_acceleration.z = -9.81
            im_pub.publish(im_msg)

            od_msg = Odometry()
            od_msg.header.stamp = self._ros_node.get_clock().now().to_msg()
            od_msg.header.frame_id = "odom"
            od_msg.child_frame_id = "base_link"
            od_msg.twist.twist.linear.x = 0.0
            od_msg.twist.twist.linear.y = 0.0
            od_msg.twist.twist.linear.z = 0.0
            od_pub.publish(od_msg)


            simulation_app.update()
            self._rclpy.spin_once(self._ros_node, timeout_sec=0.1)

        print("[StandaloneEnv] /commands received — starting simulation loop.")
        self._ros_node.destroy_publisher(js_pub)
        self.simulation_context.play()
        self._next_step_time = time.perf_counter()


    def step(self, num_steps: int = 1):
        for _ in range(num_steps):
            self._step_count += 1
            render = (self._step_count % self.decimation == 0)
            self.simulation_context.step(render=render)

            # Advance absolute target time (drift-compensating)
            self._next_step_time += self.physics_dt
            remaining = self._next_step_time - time.perf_counter()

            # spin_once doubles as rate limiter: blocks up to `remaining` seconds
            # while also processing any pending rclpy callbacks
            self._rclpy.spin_once(self._ros_node, timeout_sec=max(0.0, remaining))


    def close(self):
        if self.simulation_context is not None:
            self.simulation_context.stop()
        if self._ros_node is not None:
            self._ros_node.destroy_node()
            self._rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="GOAT Sim Control Node")
    parser.add_argument(
        "--usd_mode",
        type=str,
        default="Fixed",
        choices=["Floating", "Fixed", "Fixed_ng"],
        help="Path to USD Scene for simulation",
    )
    args = parser.parse_args()

    if args.usd_mode == "Floating":
        USD_PATH = os.path.abspath("src/assets/GOAT/WF_GOAT/usd/GOAT_ROS_HIL.usd")
    elif args.usd_mode == "Fixed":
        USD_PATH = os.path.abspath("src/assets/GOAT/WF_GOAT/usd/GOAT_ROS_Fixed.usd")
    else: 
        USD_PATH = os.path.abspath("src/assets/GOAT/WF_GOAT/usd/GOAT_ROS_Fixed_No_Gravity.usd")

    env = StandaloneEnv(
        usd_mode = args.usd_mode,
        usd_path=USD_PATH,
        robot_prim_path="/World/World/Robot",
        physics_dt=1/200,
        decimation=1,
    )

    env.load_scene()
    env.initialize_handles()

    # env.set_default_root_state(position=[0.0, 0.0, 0.51],
    #                            orientation=[1.0, 0.0, 0.0, 0.0])
    env.set_default_root_state(position=[0.0, 0.0, 0.4795],
                               orientation=[1.0, 0.0, 0.0, 0.0])

    env.set_default_joint_state(np.array([0.0, 0.0, 1.0, -1.0, 1.7, -1.7, 0.0, 0.0]))
    # env.set_default_joint_state(np.array([0.0, 0.0, 0.738, -0.738, 1.462, -1.462, 0.0, 0.0]))

    env.reset()
    env.wait_for_first_command()

    try:
        while True:
            env.step()
    except KeyboardInterrupt:
        print("\nShutting down simulation...")
    finally:
        env.close()
