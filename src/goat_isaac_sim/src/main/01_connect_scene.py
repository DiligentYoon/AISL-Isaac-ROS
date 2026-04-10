from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os
import time
import numpy as np
import omni.timeline
import isaacsim.core.utils.stage as stage_utils
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.api.physics_context import PhysicsContext
from isaacsim.core.prims import Articulation

# ======================================
# Examples without Standalone Env Class
# ======================================


# ===================
# 0) Hyperparameters 
# ===================
USD_PATH = os.path.abspath("src/assets/GOAT/WF_GOAT/usd/GOAT_ROS.usd")
PHYSICS_SCENE_PATH = "/World"
ROBOT_SCENE_PATH = "/World/World/Robot"
PHYSICS_DT = 1/200
SIM_SECONDS = 20.0

DEFAULT_JOINT_POS = np.array([0.0, 0.0, 0.738, -0.738, 1.462, -1.462, 0.0, 0.0])
DEFAULT_JOINT_VEL = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# =================================
# 1) ROS2 Bridge extension activate
# =================================
enable_extension("isaacsim.ros2.bridge")

# extension 로딩 반영
simulation_app.update()

# =========================
# 2) USD File Open
# =========================
ok = stage_utils.open_stage(USD_PATH)
if not ok:
    raise RuntimeError(f"Failed to open USD stage: {USD_PATH}")

while stage_utils.is_stage_loading():
    simulation_app.update()
    time.sleep(0.01)

stage = stage_utils.get_current_stage()
if stage is None:
    raise RuntimeError("Current stage is None after opening USD.")

# =============================
# 3) PhysicsContext Connect
# =============================
physics_context = PhysicsContext(prim_path="/World")
physics_context.set_physics_dt(PHYSICS_DT)

robot = Articulation(ROBOT_SCENE_PATH)

print(f"[Info] USD loaded: {USD_PATH}")
print(f"[Info] physics dt set to: {PHYSICS_DT}")

# ========================
# 4) Play
# ========================
timeline = omni.timeline.get_timeline_interface()
timeline.play()

simulation_app.update()

robot.initialize()
robot.set_joints_default_state(positions=DEFAULT_JOINT_POS,
                               velocities=DEFAULT_JOINT_VEL)
robot.post_reset()

num_frames = int(SIM_SECONDS / PHYSICS_DT)
for _ in range(num_frames):
    simulation_app.update()

timeline.stop()
simulation_app.close()