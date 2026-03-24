# GOAT ROS2 Package — Claude Reference

## Project Overview

ROS2 (`ament_python`) package for controlling a wheeled-legged robot (GOAT) in Isaac Sim.
Connects to Isaac Sim running on Windows via ROS2 bridge over WSL2.

The architecture is split into two layers:
- **`goat_sim/`** — Pure Python control library (no ROS2 dependency). Can be used standalone.
- **`goat/nodes/`** — ROS2 node entry points that wire `goat_sim` to ROS2 topics.

---

## Package Structure

```
src/goat/
├── package.xml                   # ROS2 manifest (ament_python build type)
├── setup.py                      # Python packaging + entry points
├── setup.cfg                     # script install directory config
├── resource/goat                 # ament_index marker (empty file, required by ROS2)
├── config/
│   └── sim_goat_config.yaml      # Robot parameters, PD/PI gains, safety limits
├── goat/                         # Python package root
│   ├── __init__.py
│   ├── nodes/                    # ROS2 node scripts (entry points)
│   │   ├── __init__.py
│   │   ├── control_node.py       # Main 200Hz control loop node
│   │   └── topic_io_node.py      # Phase 1 topic I/O verification tool
│   └── goat_sim/                 # ROS-independent core library
│       ├── control/
│       │   ├── control_pipeline.py   # Orchestrates PD + PI + safety
│       │   ├── pd_controller.py      # Position PD for joints 0~5
│       │   ├── pi_controller.py      # Speed PI (with anti-windup) for wheels 6~7
│       │   └── safety_limiter.py     # TorqueSafetyLimiter + JointSafetyLimiter
│       ├── estimation/
│       │   ├── state_types.py        # RobotState, ImuState, MotorStatesData dataclasses
│       │   └── filters.py            # FirstOrderLowPassFilter
│       └── model/
│           ├── goat_model.py         # GoatModel: central config + builder methods
│           └── model_builder.py      # YAML loader → GoatModel + ControlPipeline
└── test/
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

---

## Robot Joint Layout

8 joints total:

| Index | Name           | Controller | Notes              |
|-------|----------------|------------|--------------------|
| 0     | hip_L_Joint    | PD         | gear ratio 1       |
| 1     | hip_R_Joint    | PD         | gear ratio 1       |
| 2     | thigh_L_Joint  | PD         | gear ratio 1       |
| 3     | thigh_R_Joint  | PD         | gear ratio 1       |
| 4     | knee_L_Joint   | PD         | gear ratio 0.5     |
| 5     | knee_R_Joint   | PD         | gear ratio 0.5     |
| 6     | wheel_L_Joint  | PI         | speed control      |
| 7     | wheel_R_Joint  | PI         | speed control      |

- Joints 0~5: position PD control (target = natural_pos + delta_action)
- Joints 6~7: wheel speed PI control (with conditional integration anti-windup)

---

## Control Pipeline Flow

```
YAML config
    │
    ▼
build_control_pipeline_from_yaml()   ← model_builder.py
    │
    ▼
GoatModel  ──────────────────────────────────────────┐
    │                                                │
    ├─► PDJointController      (joints 0~5, PD)      │
    ├─► WheelPIController      (wheels 6~7, PI)      │
    ├─► TorqueSafetyLimiter    (LPF + torque clip)   │
    └─► JointSafetyLimiter     (pos/vel limit lock)  │
                                                     │
    ▼                                                │
ControlPipeline ◄────────────────────────────────────┘
    │
    ▼
compute_control(robot_state, targets, dt)
    1. JointSafetyLimiter.apply()  → clamp delta_pos if near joint limits
    2. PDJointController.compute() → PD torque for joints 0~5
    3. WheelPIController.compute() → PI torque for wheels 6~7
    4. sum raw torque
    5. TorqueSafetyLimiter.apply() → LPF + clip
    └─► returns (safe_torque, safe_targets, has_violation)
```

---

## ROS2 Topics

| Topic           | Type                    | Direction | Description                        |
|-----------------|-------------------------|-----------|------------------------------------|
| `/joint_states` | `sensor_msgs/JointState`| Subscribe | Joint pos/vel/effort from Isaac Sim |
| `/imu`          | `sensor_msgs/Imu`       | Subscribe | IMU orientation + gyro from sim    |
| `/joint_command`| `sensor_msgs/JointState`| Publish   | Torque effort commands to sim      |

QoS: `BEST_EFFORT`, `KEEP_LAST(1)` — matches Isaac Sim OmniGraph defaults.

---

## Entry Points (ros2 run)

```bash
# Phase 1: Verify topic I/O
ros2 run goat topic_io_node                  # listen only
ros2 run goat topic_io_node --test-single    # test each joint one by one
ros2 run goat topic_io_node --test-all       # test all joints simultaneously

# Phase 3: Full control loop (200 Hz)
ros2 run goat control_node                   # action mode (hold natural pose)
ros2 run goat control_node --mode nsc        # NSC mode (pitch-based balancing)
ros2 run goat control_node --config /path/to/config.yaml  # custom config
```

---

## Config File

Installed to: `share/goat/config/sim_goat_config.yaml`
Loaded via: `ament_index_python.packages.get_package_share_directory('goat')`

Sections:
- `robot`: joint names, indices, motor constants, natural position
- `pd`: proportional/derivative gains per joint (8 values each)
- `wheel_pi`: PI gains, integrator limit, output saturation per joint
- `safety`: LPF alpha, max torque, joint position/velocity limits with margins