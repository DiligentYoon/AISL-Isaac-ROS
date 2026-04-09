import rclpy
from rclpy.node import Node
from pinocchio.utils import *
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

import time
import os
import numpy as np
import pinocchio as pin
import math
import csv

from message_filters import Subscriber, ApproximateTimeSynchronizer

NUM_TRAJ_POINT = 100
DEFAULT_JOINT_ANGLE = [0.0, 1.0, 1.7, 0.0, 0.0, -1.0, -1.7, 0.0]
TARGET_JOINT_ANGLE = [0.0, 0.738, 1.462, 0.0, 0.0, -0.738, -1.462, 0.0] # Pinnochio convention : [hip_L, thigh_L, knee_L, wheel_L, hip_R, thigh_R, knee_R, wheel_R]

class NSCNode(Node):
    def __init__(self):
        super().__init__('nsc_tester_node')

        # Pinocchio
        # urdf_path = '/home/oksusu/Repos/GOAT/Realworld/src/goat_description/urdf/WF_GOAT.urdf'
        urdf_path = '/home/grape4314/AISL-Isaac-ROS/src/goat_isaac_sim/assets/GOAT/WF_GOAT/urdf/WF_GOAT.urdf'
        self.model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        self.model_names = list(self.model.names)
        self.data = self.model.createData()

        # ========== Pinocchio Name Index ========== #
        print("model.names:")
        for i, name in enumerate(self.model.names):
            print(i, name)

        print("\nidx_qs:", self.model.idx_qs)
        print("idx_vs:", self.model.idx_vs)

        print("\nJoint list:")
        for i, j in enumerate(self.model.joints):
            print(i, j)

        # model.names:
            # universe
            # root_joint
            # hip_L_Joint
            # thigh_L_Joint
            # knee_L_Joint
            # wheel_L_Joint
            # hip_R_Joint
            # thigh_R_Joint
            # knee_R_Joint
            # wheel_R_Joint
        
        # generalized positions: 
            # base_position_xyz(3),
            # base_quaternion_xyzw(4),
            # hip_L,
            # thigh_L,
            # knee_L,
            # wheel_L,
            # hip_R,
            # thigh_R,
            # knee_R,
            # wheel_R
        
        # generalized velocities:
            # base_twist(6),
            # hip_L_dot,
            # thigh_L_dot,
            # knee_L_dot,
            # wheel_L_dot,
            # hip_R_dot,
            # thigh_R_dot,
            # knee_R_dot,
            # wheel_R_dot

        # ========= ROS Topic Name Index ========== #
        # name:
            # hip_L_Joint
            # hip_R_Joint
            # thigh_L_Joint
            # thigh_R_Joint
            # knee_L_Joint
            # knee_R_Joint
            # wheel_L_Joint
            # wheel_R_Joint

        # ========== Name Index Mapping =========== #
        self.ros_joint_names = [
            'hip_L_Joint', 'hip_R_Joint', 'thigh_L_Joint', 'thigh_R_Joint',
            'knee_L_Joint', 'knee_R_Joint', 'wheel_L_Joint', 'wheel_R_Joint'
        ]

        self.pin_joint_names = [
            'hip_L_Joint', 'thigh_L_Joint', 'knee_L_Joint', 'wheel_L_Joint',
            'hip_R_Joint', 'thigh_R_Joint', 'knee_R_Joint', 'wheel_R_Joint'
        ]
        self.ros_name_to_idx = {name: i for i, name in enumerate(self.ros_joint_names)}
        self.pin_name_to_idx = {name: i for i, name in enumerate(self.pin_joint_names)}

        self.ros_to_pin_ids = [0, 2, 4, 6, 1, 3, 5, 7] # ROS[self.ros_to_pin_ids] = Pin Ids
        self.pin_to_ros_ids = [0, 4, 1, 5, 2, 6, 3, 7] # Pin[self.pin_to_ros_ids] = ROS Ids

        # actuator ordering
        self.wheel_L_joint_id = self.pin_name_to_idx['wheel_L_Joint']   # 3 : Left wheel index in pinocchio-actuator-order
        self.wheel_R_joint_id = self.pin_name_to_idx['wheel_R_Joint']   # 7 : Right wheel index in pinocchio-actuator-order

        # pinocchio model ordering
        self.wheel_L_joint_pin_id = self.model_names.index('wheel_L_Joint') # 5 : Left wheel index in pinocchio-joint-order
        self.wheel_R_joint_pin_id = self.model_names.index('wheel_R_Joint') # 9 : Right wheel index in pinocchio-joint-order

        # generalized velocity ordering inside nv=14
        self.wheel_L_joint_nv_id = 6 + self.wheel_L_joint_id                 # 9
        self.wheel_R_joint_nv_id = 6 + self.wheel_R_joint_id                 # 13

        # Robot parameters
        self.wheel_radius = 72.75E-03
        self.nq = self.model.nq                         # Position dim (7 + n)
        self.nv = self.model.nv                         # Velocity dim (6 + n)
        self.n_joints = self.nv - 6                     # Num of Motors
        self.joint_tau_limit = 4.5                      # Nm
        self.wheel_tau_limit = 2.5                      # Nm
        self.theta_cmd_limit = math.radians(5.0)

        # Parameters
        self.dt = 1/200
        self.Kp = np.eye(self.n_joints) * 50.0
        self.Kd = np.eye(self.n_joints) * 3.0
        self.wheel_Kp_att = 3.0
        self.wheel_Kd_att = 0.3
        self.wheel_Kp_pos = 0.02
        self.wheel_Kd_pos = 0.02
        self.alpha = 1.0
        self.cascade_ratio = 1
        self.count_tick = 0

        # State variables
        self.S_leg = np.zeros((6, self.nv))
        self.S_leg[0, 6]  = 1.0   # hip_L
        self.S_leg[1, 7]  = 1.0   # thigh_L
        self.S_leg[2, 8]  = 1.0   # knee_L
        self.S_leg[3, 10] = 1.0   # hip_R
        self.S_leg[4, 11] = 1.0   # thigh_R
        self.S_leg[5, 12] = 1.0   # knee_R

        self.S_wheel = np.zeros((2, self.nv))
        self.S_wheel[0, self.wheel_L_joint_nv_id] = 1.0
        self.S_wheel[1, self.wheel_R_joint_nv_id] = 1.0

        self.q_curr = np.zeros(self.nq)                                     # Generalized position (Base 7 + Joints n)
        self.v_curr = np.zeros(self.nv)                                     # Generalized velocity (Base 6 + Joints n)
        self.joint_q_curr = np.zeros(self.n_joints)                         # Joint position
        self.joint_v_curr = np.zeros(self.n_joints)                         # Joint velocity
        self.base_q_curr = np.zeros(7)                                      # Base position state
        self.base_quat_curr = np.array([0.0, 0.0, 0.0, 1.0])                # Base quaternion (x, y, z, w)
        self.base_v_curr = np.zeros(6)                                      # Base velocity state
        self.base_linear_v_curr = np.zeros(3)                               # Base linear velocity
        self.base_w_curr = np.zeros(3)                                      # Base angular velocity
        self.base_a_curr = np.zeros(3)                                      # Base linear acceleration
        self.q_ref = np.zeros(self.nq)                                      # Reference joint position
        self.q_ref_traj = np.zeros((NUM_TRAJ_POINT, self.n_joints))               # Reference joint position trajectory
        self.a_ref = np.zeros(self.nv)                                      # Reference joint acceleration
        self.phi_ref = 0.0
        self.theta_ref = 0.0
        self.cur_theta_ref = 0.0

        # Reference assign
        for i, (start, end) in enumerate(zip(DEFAULT_JOINT_ANGLE, TARGET_JOINT_ANGLE)):
            self.q_ref_traj[:, i] = np.linspace(start, end, NUM_TRAJ_POINT)
        
        self.tau_cmd = np.zeros(self.n_joints)
        self.tau_applied = np.zeros(self.n_joints)

        # Sensor synchronization stamps
        self._stamp_joint = None
        self._stamp_imu   = None
        self._stamp_odom  = None

        
        self.contact_normal_world = np.array([0.0, 0.0, 1.0])   # ground normal
        self.contact_lateral_world = np.array([0.0, 1.0, 0.0])  # example

        self.coulomb_coeff = np.array([5.646268e-02, 5.646268e-02, 5.373143e-02, 0.0,
                                       5.646268e-02, 5.646268e-02, 5.373143e-02, 0.0]) # hip, thigh, knee, wheel(ignored)
        
        self.viscous_coeff = np.array([3.190248e-01, 3.190248e-01, 8.441387e-02, 0.0,
                                       3.190248e-01, 3.190248e-01, 8.441387e-02, 0.0]) # hip, thigh, knee, wheel(ignored)

        # TF subscriber
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher
        self.command_publisher = self.create_publisher(JointState, '/commands', 10)

        # Subscriber
        self.joint_state_subscriber = Subscriber(self, JointState, '/sim_joint_states', 10)
        self.imu_state_subscriber = Subscriber(self, Imu, '/sim_imu', 10)
        self.velocity_state_subscriber = Subscriber(self, Odometry, '/odom', 10)
        # self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        # self.imu_state_subscriber = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        # self.velocity_state_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.time_sync = ApproximateTimeSynchronizer([self.joint_state_subscriber, self.imu_state_subscriber, self.velocity_state_subscriber], 10, 0.01)
        self.time_sync.registerCallback(self.sync_callback)

        self.timer = self.create_timer(self.dt, self.control_loop)

        self.csv_data = {
            "com_pitch": [],
            "com_pitch_ref": [],
            "com_horizontal": [],
            "com_rel_horizontal": [],
            "wheel_axis": [],
            "joint_pos": [],
            "joint_torque": [],
            "joint_target": [],
            "joint_acc_nom": [],
            "joint_acc_con": [],
            "joint_pos_error": [],
            "joint_vel_error": [],
        }

    def sync_callback(self, joint_msg, imu_msg, odom_msg):
        self.joint_callback(joint_msg)
        self.imu_callback(imu_msg)
        self.odom_callback(odom_msg)

        # Sensor data syncronization
        # self.control_loop()


    def joint_callback(self, msg):
        # ROS Ids -> Pinocchio Ids
        q = np.array(msg.position)
        v = np.array(msg.velocity)
        tau = np.array(msg.effort)

        self.joint_q_curr = q[self.ros_to_pin_ids]
        self.joint_v_curr = v[self.ros_to_pin_ids]
        self.tau_applied = tau[self.ros_to_pin_ids]
        self._stamp_joint = msg.header.stamp


    def imu_callback(self, msg):
        self.base_a_curr = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        self.base_w_curr = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.base_quat_curr = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self._stamp_imu = msg.header.stamp


    def odom_callback(self, msg):
        self.base_linear_v_curr = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]) # In body frame
        self._stamp_odom = msg.header.stamp


    def control_loop(self):
        if None in (self._stamp_imu, self._stamp_joint, self._stamp_odom):
            return
        # Stack base + joint state
        self.base_q_curr = np.concatenate((np.zeros(3), self.base_quat_curr))                  # XYZ position fixed to 0
        self.base_v_curr = np.concatenate((self.base_linear_v_curr, self.base_w_curr))
        self.q_curr = np.concatenate((self.base_q_curr, self.joint_q_curr))
        self.v_curr = np.concatenate((self.base_v_curr, self.joint_v_curr))

        # Compute Dynamics matrix
        pin.computeAllTerms(self.model, self.data, self.q_curr, self.v_curr)
        M = self.data.M                                     # Mass matrix
        C = self.data.C                                     # Coriolis matrix
        G = self.data.g                                     # Gravity vector

        ## ============== Wheel control ================ ##
        # State
        theta, theta_dot, L = self.compute_com_and_theta(self.q_curr, self.v_curr)

        phi = (self.joint_q_curr[self.wheel_L_joint_id] - self.joint_q_curr[self.wheel_R_joint_id]) / 2.0
        ratio = L / self.wheel_radius
        phi_comp = phi + ratio * math.sin(theta)

        # Control logic
        if self.count_tick % self.cascade_ratio == 0:
            self.theta_ref = self.wheel_com_horizontal_position_control(theta, theta_dot,  self.phi_ref, L)
        self.cur_theta_ref = (1 - self.alpha) * self.cur_theta_ref + self.alpha * self.theta_ref
        wheel_tau = self.wheel_com_pitch_position_control(theta, theta_dot, self.cur_theta_ref)

        ## ============= Joint control ================ ##

        # Update reference
        # self.q_ref[7:] = self.q_ref_traj[min(self.count_tick, NUM_TRAJ_POINT-1), :]
        self.q_ref[7:] = np.array(TARGET_JOINT_ANGLE)

        # Error Feedback for desired generalized acceleration
        q_err = self.q_ref[7:] - self.q_curr[7:]
        v_err = -self.v_curr[6:].copy()
        self.a_ref[6:] = self.Kp @ q_err + self.Kd @ v_err
        self.a_ref[self.wheel_L_joint_nv_id] = 0.0
        self.a_ref[self.wheel_R_joint_nv_id] = 0.0

        # Contact point Jacobian, its time derivative, and nullspace basis
        Jc = self.compute_contact_jacobian()
        Jc_dot_v = self.compute_contact_jacobian_dot_times_v()
        Qu = self.compute_constraint_nullspace(Jc)

        # Constraints-consistent projection using nullspace method
        a_ref_constrained = self.project_to_contact_consistent_acceleration(self.a_ref, Jc, Jc_dot_v)

        # Torque from reduced dynamics
        tau_constrained = self.solve_leg_torque_reduced_dynamics(M, C @ self.v_curr + G, a_ref_constrained, np.array([wheel_tau, -wheel_tau]), Qu)
        tau_constrained = np.clip(tau_constrained, -self.joint_tau_limit, self.joint_tau_limit)
        tau_constrained_full = self.S_leg.T @ tau_constrained + self.S_wheel.T @ np.array([wheel_tau, -wheel_tau])

        ## ============= Cmd setting =============== ##
        self.tau_cmd = tau_constrained_full[6:]
        # self.tau_cmd = np.zeros(self.n_joints)
        # self.tau_cmd[self.wheel_L_joint_id] = wheel_tau
        # self.tau_cmd[self.wheel_R_joint_id] = -wheel_tau


        # print(f"tau_cmd : {self.tau_cmd}")

        # CSV logging
        self.csv_data["com_pitch"].append(theta)
        self.csv_data["com_pitch_ref"].append(self.cur_theta_ref)
        self.csv_data["com_horizontal"].append(phi_comp * self.wheel_radius)
        self.csv_data["com_rel_horizontal"].append(L * math.sin(theta))
        self.csv_data["wheel_axis"].append(self.wheel_radius * phi)


        self.csv_data["joint_pos"].append(self.q_curr[7:].copy().tolist())
        self.csv_data["joint_torque"].append(self.tau_cmd.copy().tolist())
        self.csv_data["joint_target"].append(self.q_ref[7:].copy().tolist())
        self.csv_data["joint_acc_nom"].append(self.a_ref[6:].copy().tolist())
        self.csv_data["joint_acc_con"].append(a_ref_constrained[6:].copy().tolist())
        self.csv_data["joint_pos_error"].append(q_err.copy().tolist())
        self.csv_data["joint_vel_error"].append(v_err.copy().tolist())


        # Publish joint command : Pin Ids -> ROS Ids
        joint_command = JointState()
        joint_command.header.stamp = self.get_clock().now().to_msg()
        joint_command.name = [
            'hip_L_Joint', 'hip_R_Joint', 'thigh_L_Joint', 'thigh_R_Joint', 
            'knee_L_Joint', 'knee_R_Joint', 'wheel_L_Joint', 'wheel_R_Joint'
        ]
        joint_command.position = self.q_ref[7:][self.pin_to_ros_ids].tolist()
        joint_command.effort = self.tau_cmd[self.pin_to_ros_ids].tolist()
        self.command_publisher.publish(joint_command)

        self.count_tick += 1

### =============================== Auxilary Functions =============================== ###
    def compute_com_and_theta(self, q: np.ndarray, v: np.ndarray):
        # COM calculation
        pin.centerOfMass(self.model, self.data, q, v, compute_subtree_coms=True)
        
        # Robot property
        M_total = self.data.mass[0]       # Total mass
        com_total = self.data.com[0]      # Total mass position
        vcom_total = self.data.vcom[0]    # Total mass velocity 

        m_wheel_L = self.data.mass[self.wheel_L_joint_pin_id]
        com_wheel_L  = self.data.oMi[self.wheel_L_joint_pin_id].act(
                           self.data.com[self.wheel_L_joint_pin_id])     # world frame
        vcom_wheel_L = self.data.oMi[self.wheel_L_joint_pin_id].rotation @ \
                           self.data.vcom[self.wheel_L_joint_pin_id]     # world frame

        m_wheel_R = self.data.mass[self.wheel_R_joint_pin_id]
        com_wheel_R  = self.data.oMi[self.wheel_R_joint_pin_id].act(
                           self.data.com[self.wheel_R_joint_pin_id])     # world frame
        vcom_wheel_R = self.data.oMi[self.wheel_R_joint_pin_id].rotation @ \
                           self.data.vcom[self.wheel_R_joint_pin_id]     # world frame

        # Body's property exclude wheels
        M_body = M_total - m_wheel_L - m_wheel_R
        com_body = (M_total * com_total - m_wheel_L * com_wheel_L - m_wheel_R * com_wheel_R) / M_body
        vcom_body = (M_total * vcom_total - m_wheel_L * vcom_wheel_L - m_wheel_R * vcom_wheel_R) / M_body

        # Center of wheels
        com_wheel = (com_wheel_L + com_wheel_R) / 2.0
        vcom_wheel = (vcom_wheel_L + vcom_wheel_R) / 2.0
  
        P_rel = com_body - com_wheel
        V_rel = vcom_body - vcom_wheel
        
        # Pitch calculation
        theta = math.atan2(P_rel[0], P_rel[2])
        # Angular velocity calculation
        theta_dot = (V_rel[0] * P_rel[2] - V_rel[2] * P_rel[0]) / (P_rel[0]**2 + P_rel[2]**2 + 1e-6)
        # Pendulum length
        L = math.hypot(P_rel[0], P_rel[2])

        return theta, theta_dot, L
    

    def wheel_com_horizontal_position_control(self, theta, theta_dot, target_phi, L):
        # Wheel state
        phi = (self.joint_q_curr[self.wheel_L_joint_id] - self.joint_q_curr[self.wheel_R_joint_id]) / 2.0
        phi_dot = (self.joint_v_curr[self.wheel_L_joint_id] - self.joint_v_curr[self.wheel_R_joint_id]) / 2.0
        ratio = L / self.wheel_radius
        
        # Dynamic decoupling
        phi_comp = (phi + ratio * math.sin(theta))
        phi_comp_dot = (phi_dot + ratio * math.cos(theta) * theta_dot)
        phi_err = target_phi - phi_comp
        
        # Pitch PD controller
        theta_cmd = self.wheel_Kp_pos * phi_err - self.wheel_Kd_pos * phi_comp_dot
        
        return np.clip(theta_cmd, -self.theta_cmd_limit, self.theta_cmd_limit)
    

    def wheel_com_pitch_position_control(self, theta, theta_dot, theta_cmd):
        # PD controller
        theta_err = theta - theta_cmd
        wheel_tau = self.wheel_Kp_att * theta_err + self.wheel_Kd_att * theta_dot
        
        return np.clip(wheel_tau, -self.wheel_tau_limit, self.wheel_tau_limit)


    def save_csv(self, filename="nsc_log.csv"):
        default_path = "/mnt/c/Users/grape/OneDrive/CAU/GOAT/data"
        with open(os.path.join(default_path, filename), "w", newline="") as f:
            writer = csv.writer(f)

            header = ["step", "com_pitch", "com_pitch_ref", "com_horizontal", "com_rel_horizontal", "wheel_axis"]

            for name in self.pin_joint_names:
                header.append(f"{name}_pos")
            for name in self.pin_joint_names:
                header.append(f"{name}_tau")
            for name in self.pin_joint_names:
                header.append(f"{name}_target")
            for name in self.pin_joint_names:
                header.append(f"{name}_acc_nom")
            for name in self.pin_joint_names:
                header.append(f"{name}_acc_con")
            for name in self.pin_joint_names:
                header.append(f"{name}_pos_error")
            for name in self.pin_joint_names:
                header.append(f"{name}_vel_error")

            writer.writerow(header)

            n = len(self.csv_data["com_pitch"])
            for k in range(n):
                row = [
                    k,
                    self.csv_data["com_pitch"][k],
                    self.csv_data["com_pitch_ref"][k],
                    self.csv_data["com_horizontal"][k],
                    self.csv_data["com_rel_horizontal"][k],
                    self.csv_data["wheel_axis"][k],
                ]
                row += self.csv_data["joint_pos"][k]
                row += self.csv_data["joint_torque"][k]
                row += self.csv_data["joint_target"][k]
                row += self.csv_data["joint_acc_nom"][k]
                row += self.csv_data["joint_acc_con"][k]
                row += self.csv_data["joint_pos_error"][k]
                row += self.csv_data["joint_vel_error"][k]
                writer.writerow(row)

    # ============== Related to Joint Torque Control ============== #  
    def compute_contact_jacobian(self) -> np.ndarray:
        pin.computeJointJacobians(self.model, self.data, self.q_curr)
        pin.updateFramePlacements(self.model, self.data)

        rf = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED

        # Use wheel center (Identity placement) — offset [0,0,-r] would rotate
        # with the wheel joint, drifting away from the true ground contact point.
        placement_L = pin.SE3.Identity()
        placement_R = pin.SE3.Identity()

        J6_L = pin.getFrameJacobian(
            self.model,
            self.data,
            self.wheel_L_joint_pin_id,
            placement_L,
            rf,
        )
        J6_R = pin.getFrameJacobian(
            self.model,
            self.data,
            self.wheel_R_joint_pin_id,
            placement_R,
            rf,
        )

        Jv_L = J6_L[:3, :]
        Jv_R = J6_R[:3, :]

        n = self.contact_normal_world.reshape(1, 3)
        t = self.contact_lateral_world.reshape(1, 3)

        Jc = np.vstack([
            n @ Jv_L,
            n @ Jv_R,
        ])
        # Jc = np.vstack([
        #     n @ Jv_L,
        #     t @ Jv_L,
        #     n @ Jv_R,
        #     t @ Jv_R,
        # ])
        return Jc


    def compute_contact_jacobian_dot_times_v(self) -> np.ndarray:
        """
        Compute Jdot_c(q,v) * v in R^4
        """
        rf = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED

        placement_L = pin.SE3.Identity()
        placement_R = pin.SE3.Identity()

        a_zero = np.zeros(self.model.nv)

        # second-order forward kinematics
        pin.forwardKinematics(self.model, self.data, self.q_curr, self.v_curr, a_zero)
        pin.updateFramePlacements(self.model, self.data)

        # classical acceleration of the contact point
        acc_L = pin.getFrameClassicalAcceleration(
            self.model,
            self.data,
            self.wheel_L_joint_pin_id,
            placement_L,
            rf
        )
        acc_R = pin.getFrameClassicalAcceleration(
            self.model,
            self.data,
            self.wheel_R_joint_pin_id,
            placement_R,
            rf
        )

        # linear part = Jdot(q,v) * v   when a = 0
        a_lin_L = acc_L.linear
        a_lin_R = acc_R.linear

        n = self.contact_normal_world
        t = self.contact_lateral_world

        Jcdot_v = np.array([
            n @ a_lin_L,   # left normal
            n @ a_lin_R,   # right normal
        ])
        # Jcdot_v = np.array([
        #     n @ a_lin_L,   # left normal
        #     t @ a_lin_L,   # left lateral
        #     n @ a_lin_R,   # right normal
        #     t @ a_lin_R,   # right lateral
        # ])

        return Jcdot_v


    def compute_constraint_nullspace(self, Jc: np.ndarray, tol: float = 1e-8) -> np.ndarray:
        """
        Compute nullspace basis Q_u such that J_c @ Q_u = 0
        Q_u shape: (nv, nv-rank(Jc))
        """
        U, S, Vt = np.linalg.svd(Jc, full_matrices=True)
        rank = np.sum(S > tol)
        Qu = Vt.T[:, rank:]   # nullspace basis
        return Qu


    def project_to_contact_consistent_acceleration(self,
                                                   qdd_nom: np.ndarray,
                                                   Jc: np.ndarray,
                                                   Jcdot_v: np.ndarray,
                                                   damping: float = 1e-6) -> np.ndarray:
        """
        Equality-constrained least squares projection:
            min ||qdd - qdd_nom||^2
            s.t. Jc qdd = -Jcdot_v
        """
        JJt = Jc @ Jc.T
        JJt_reg = JJt + damping * np.eye(JJt.shape[0])

        rhs = Jc @ qdd_nom + Jcdot_v
        correction = Jc.T @ np.linalg.solve(JJt_reg, rhs)
        qdd_d = qdd_nom - correction
        return qdd_d


    def solve_leg_torque_reduced_dynamics(self,
                                          M: np.ndarray,
                                          h: np.ndarray,
                                          qdd_d: np.ndarray,
                                          tau_w: np.ndarray,
                                          Qu: np.ndarray) -> np.ndarray:
        """
        Solve leg torque from reduced constrained dynamics:
            tau_j = (Qu^T Sj^T)^dagger Qu^T (M qdd_d + h - Sw^T tau_w)
        """
        rhs_full = M @ qdd_d + h - self.S_wheel.T @ tau_w
        A = Qu.T @ self.S_leg.T
        b = Qu.T @ rhs_full

        tau_j = np.linalg.pinv(A) @ b
        return tau_j


def main(args=None):
    rclpy.init(args=args)
    node = NSCNode()
    
    # Automatic control loop using ROS timer callback
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_csv()
        node.destroy_node()
        rclpy.shutdown()

    # Manual control loop to maintain fixed control frequency
    # dt = node.dt
    # next_t = time.monotonic()
    # try:
    #     while rclpy.ok():
    #         rclpy.spin_once(node, timeout_sec=0.0)
    #         node.control_loop()
    #
    #         next_t += dt
    #         sleep_time = next_t - time.monotonic()
    #         if sleep_time > 0:
    #             time.sleep(sleep_time)
    #         else:
    #             next_t = time.monotonic()

    # Sensor-synchronized control loop
    # control_loop() is triggered automatically inside _try_control_loop()
    # when all three sensor callbacks receive data with the same timestamp.
    # try:
    #     while rclpy.ok():
    #         rclpy.spin_once(node, timeout_sec=0.001)
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     node.save_csv()
    #     node.destroy_node()
    #     rclpy.shutdown()
    #     print(f"CSV saved")
    
        

if __name__ == '__main__':
    main()