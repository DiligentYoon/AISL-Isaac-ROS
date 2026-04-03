import rclpy
from rclpy.node import Node
from pinocchio.utils import *
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

import time
import numpy as np
import pinocchio as pin
import math
import csv

TARGET_JOINT_ANGLE = [0.0, 0.738, 1.462, 0.0, 0.0, -0.738, -1.462, 0.0] # Pinnochio convention : [hip_L, thigh_L, knee_L, wheel_L, hip_R, thigh_R, knee_R, wheel_R]

class NSCTesterNode(Node):
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

        self.wheel_L_joint_id = self.pin_name_to_idx['wheel_L_Joint']   # 3 : Left wheel index in pinocchio-actuator-order
        self.wheel_R_joint_id = self.pin_name_to_idx['wheel_R_Joint']   # 7 : Right wheel index in pinocchio-actuator-order

        self.wheel_L_joint_pin_id = self.model_names.index('wheel_L_Joint') # 5 : Left wheel index in pinocchio-joint-order
        self.wheel_R_joint_pin_id = self.model_names.index('wheel_R_Joint') # 9 : Right wheel index in pinocchio-joint-order

        # Robot parameters
        self.wheel_radius = 72.75E-03
        self.nv = self.model.nv                         # Velocity dim (6 + n)
        self.nq = self.model.nq                         # Position dim (7 + n)
        self.n_joints = self.nv - 6                     # Num of Motors
        self.joint_tau_limit = 4.5                      # Nm
        self.wheel_tau_limit = 2.5                      # Nm
        self.theta_cmd_limit = math.radians(5.0)

        # Parameters
        self.dt = 1/200
        self.Kp = np.eye(self.n_joints) * 100.0
        self.Kd = np.eye(self.n_joints) * 10.0
        self.Ko = np.eye(self.nv) * 0.1                                    # MOB gain (Base 6 + Joints n)
        self.wheel_Kp_att = 10.0
        self.wheel_Kd_att = 1.0
        self.wheel_Kp_pos = 5.0
        self.wheel_Kd_pos = 0.1
        self.cascade_ratio = 5
        self.count_tick = 0

        # State variables
        self.q_curr = np.zeros(self.nq)
        self.v_curr = np.zeros(self.nv)
        self.joint_q_curr = np.zeros(self.n_joints)                         # Joint position
        self.joint_v_curr = np.zeros(self.n_joints)                         # Joint velocity
        self.base_q_curr = np.zeros(7)                                      # Base position state
        self.base_quat_curr = np.array([0.0, 0.0, 0.0, 1.0])                # Base quaternion
        self.base_v_curr = np.zeros(6)                                      # Base velocity state
        self.base_linear_v_curr = np.zeros(3)                               # Base linear velocity
        self.base_w_curr = np.zeros(3)                                      # Base angular velocity
        self.base_a_curr = np.zeros(3)                                      # Base linear acceleration
        self.q_ref = np.zeros(self.nq)                                      # Reference joint position
        self.a_ref = np.zeros(self.nv)                                      # Reference joint acceleration
        self.phi_ref = 0.0
        self.theta_ref = 0.0

        # Reference assign
        self.q_ref[7:] = np.array(TARGET_JOINT_ANGLE)
        
        self.tau_cmd = np.zeros(self.n_joints)
        self.tau_applied = np.zeros(self.n_joints)

        # Sensor synchronization stamps
        self._stamp_joint = None
        self._stamp_imu   = None
        self._stamp_odom  = None

        # MOB(Momentum Observer) parameters
        self.mob_integral = np.zeros(self.nv)
        self.tau_external = np.zeros(self.nv)                               # Residual

        # TF subscriber
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher
        self.command_publisher = self.create_publisher(JointState, '/joint_command', 10)

        # Subscriber
        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.imu_state_subscriber = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.velocity_state_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # self.timer = self.create_timer(self.dt, self.control_loop)

        self.csv_data = {
            "com_pitch": [],
            "com_pitch_ref": [],
            "joint_pos": [],
            "joint_torque": [],
            "joint_target": [],
            "joint_torque_rnea": [],
            "joint_torque_external": [],
        }


    def joint_callback(self, msg):
        # ROS Ids -> Pinocchio Ids
        q = np.array(msg.position)
        v = np.array(msg.velocity)
        tau = np.array(msg.effort)

        self.joint_q_curr = q[self.ros_to_pin_ids]
        self.joint_v_curr = v[self.ros_to_pin_ids]
        self.tau_applied = tau[self.ros_to_pin_ids]
        self._stamp_joint = msg.header.stamp
        self._try_control_loop()


    def imu_callback(self, msg):
        self.base_a_curr = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        self.base_w_curr = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.base_quat_curr = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self._stamp_imu = msg.header.stamp
        self._try_control_loop()


    def odom_callback(self, msg):
        self.base_linear_v_curr = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]) # In body frame
        self._stamp_odom = msg.header.stamp
        self._try_control_loop()


    def _try_control_loop(self):
        if None in (self._stamp_joint, self._stamp_imu, self._stamp_odom):
            return
        self.control_loop()


    def control_loop(self):
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

        # Error Feedback for desired generalized acceleration
        q_err = self.q_ref[7:] - self.q_curr[7:]
        v_err = -self.v_curr[6:]
        self.a_ref[6:] = self.Kp @ q_err + self.Kd @ v_err
        self.a_ref[6 + self.wheel_L_joint_id] = 0.0
        self.a_ref[6 + self.wheel_R_joint_id] = 0.0

        # RNEA
        tau_rnea = pin.rnea(self.model, self.data, self.q_curr, self.v_curr, self.a_ref)
        tau_rnea_joint = tau_rnea[6:]                       # Extract joint torque

        # Generalized Momentum Observer ( tau_external = Ko * [Mv - int(tau + C.T*v - G + tau_external)dt] )
        tau_full = np.concatenate((np.zeros(6), self.tau_applied))              # base 6DOF unactuated
        integrand = tau_full + (C.T @ self.v_curr) - G + self.tau_external
        self.mob_integral += integrand * self.dt
        
        p_curr = M @ self.v_curr
        self.tau_external = self.Ko @ (p_curr - self.mob_integral)              # External torque for each joints
        self.joint_tau_external = self.tau_external[6:]                         # Extract joint torque    

        # # Total torque (RNEA + External)
        self.tau_cmd = tau_rnea_joint - self.joint_tau_external

        # Clipping
        self.tau_cmd = np.clip(self.tau_cmd, -self.joint_tau_limit, self.joint_tau_limit)

        ## ============== Wheel control ================ ##
        # State
        theta, theta_dot, L = self.compute_com_and_theta(self.q_curr, self.v_curr)

        # print(f"theta : {theta}, theta_dot : {theta_dot}, L : {L}")

        # Control logic
        # print(f"tick : {self.count_tick}")
        if self.count_tick % self.cascade_ratio == 0:
            self.theta_ref = self.wheel_com_horizontal_position_control(theta, theta_dot,  self.phi_ref, L)
        wheel_tau = self.wheel_com_pitch_position_control(theta, theta_dot, self.theta_ref)

        self.tau_cmd[self.wheel_L_joint_id] = wheel_tau
        self.tau_cmd[self.wheel_R_joint_id] = -wheel_tau

        self.tau_cmd[self.wheel_L_joint_id] = np.clip(self.tau_cmd[self.wheel_L_joint_id], -self.wheel_tau_limit, self.wheel_tau_limit)
        self.tau_cmd[self.wheel_R_joint_id] = np.clip(self.tau_cmd[self.wheel_R_joint_id], -self.wheel_tau_limit, self.wheel_tau_limit)

        # CSV logging
        self.csv_data["com_pitch"].append(theta)
        self.csv_data["com_pitch_ref"].append(self.theta_ref)
        self.csv_data["joint_pos"].append(self.joint_q_curr.copy().tolist())
        self.csv_data["joint_torque"].append(self.tau_cmd.copy().tolist())
        self.csv_data["joint_target"].append(self.q_ref[7:].copy().tolist())
        self.csv_data["joint_torque_rnea"].append(tau_rnea_joint.copy().tolist())
        self.csv_data["joint_torque_external"].append(self.joint_tau_external.copy().tolist())

        # Publish joint command : Pin Ids -> ROS Ids
        joint_command = JointState()
        joint_command.header.stamp = self.get_clock().now().to_msg()
        joint_command.name = [
            'hip_L_Joint', 'hip_R_Joint', 'thigh_L_Joint', 'thigh_R_Joint', 
            'knee_L_Joint', 'knee_R_Joint', 'wheel_L_Joint', 'wheel_R_Joint'
        ]
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
        phi_comp = phi + ratio * math.sin(theta)
        phi_comp_dot = phi_dot + ratio * math.cos(theta) * theta_dot
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
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)

            header = ["step", "com_pitch", "com_pitch_ref"]

            for name in self.pin_joint_names:
                header.append(f"{name}_pos")
            for name in self.pin_joint_names:
                header.append(f"{name}_tau")
            for name in self.pin_joint_names:
                header.append(f"{name}_target")
            for name in self.pin_joint_names:
                header.append(f"{name}_tau_rnea")
            for name in self.pin_joint_names:
                header.append(f"{name}_tau_external")

            writer.writerow(header)

            n = len(self.csv_data["com_pitch"])
            for k in range(n):
                row = [
                    k,
                    self.csv_data["com_pitch"][k],
                    self.csv_data["com_pitch_ref"][k],
                ]
                row += self.csv_data["joint_pos"][k]
                row += self.csv_data["joint_torque"][k]
                row += self.csv_data["joint_target"][k]
                row += self.csv_data["joint_torque_rnea"][k]
                row += self.csv_data["joint_torque_external"][k]
                writer.writerow(row)


def main(args=None):
    rclpy.init(args=args)
    node = NSCTesterNode()
    
    # Automatic control loop using ROS timer callback
    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()

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
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"Save CSV")
        # node.save_csv()
        node.destroy_node()
        rclpy.shutdown()
    
        

if __name__ == '__main__':
    main()