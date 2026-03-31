# CLAUDE.md

이 파일은 Claude Code (claude.ai/code)가 이 저장소에서 작업할 때 참고하는 가이드입니다.

## 협업 규칙

- **대화**: 한국어로 소통
- **코드 및 주석**: 영문으로 작성

## 프로젝트 개요

**GOAT (Wheel-Foot Bi-pedal)** — 6개의 다리 관절(각 다리 3개)과 2개의 바퀴를 가진 이족보행 로봇의 시뮬레이션 및 제어 스택.

## 개발 환경

- **Python**: 3.11+
- **Simulator**: NVIDIA Isaac Sim (via Omniverse); IsaacLab: `C:/IsaacLab/`
- **OS**: Windows (references/ 패키지는 빌드하지 않음 — 참고 전용)
- **Robot model**: `src/assets/GOAT/WF_GOAT/`

## 실행 명령어

### Isaac Sim 스크립트 (Windows)
```bash
python src/main/00_standalone_scene.py   # 시뮬레이션 실행 (200Hz 물리, 200Hz 렌더)
```

### WSL2 / Linux — 토픽 검증 및 제어
```bash
# 토픽 확인
ros2 topic list
ros2 topic hz /joint_states              # 200Hz 확인

# NSC 제어 노드 실행 (RNEA + MOB + 진자 바퀴 균형)
ros2 run goat nsc_control_node
```

## 아키텍처

### [핵심 참고] ROS2 제어 스택 (`references/src/`)

시뮬레이션에 그대로 구현해야 하는 로직. **core + nodes 분리 원칙** 준수: 모든 제어 로직은 ROS 의존성 없이 `goat_control/core/`에 위치하며, `goat_control/nodes/`는 이를 호출하는 얇은 ROS2 래퍼.

**제어 파이프라인** (`goat_control/core/control/control_pipeline.py`):
1. CAN을 통해 모터 상태 폴링 → `StateManager`
2. `RobotState` 구성 (관절 + IMU)
3. PD 컨트롤러(관절) + PI 컨트롤러(바퀴, anti-windup 포함) 실행
4. `SafetyLimiter`로 토크 안전 한계 적용 (LPF + 하드 리밋)
5. 모터에 토크 명령 전송

**노드 구성:**
- `state_estimation_node` — CAN/IMU 하드웨어 읽기, `MotorStates` + `BaseStates` 퍼블리시
- `control_node` — 상태/액션 구독, 제어 파이프라인 실행, 토크 명령 퍼블리시
- `motor_io_node` — ROS와 물리 모터 간 CAN 브리지
- `agent_node` — 정책/액션 입력 인터페이스

**주요 패키지:**
- `goat_control` — 메인 제어 스택 (8개 엔트리포인트 노드)
- `goat_description` — URDF, 메쉬, 시각화
- `goat_sysid` — 시스템 식별 유틸리티 (마찰, 관절 파라미터)
- `motor_interfaces` — 커스텀 ROS 메시지 (`MotorStates.msg`, `BaseStates.msg`)

**파라미터 설정:** 게인, 모터 스펙, 관절 한계, 캘리브레이션 오프셋 등 모든 튜닝 값은 `references/src/goat_control/config/goat_config.yaml`에 위치.

### NSC 제어 노드 (`src/goat/goat/nodes/nsc_control_node.py`)

시뮬레이터에서 실행하는 실제 제어 노드 (`NSCTesterNode`). 관절은 RNEA 기반 토크 제어, 바퀴는 Pinocchio COM으로 추정한 진자 상태 기반 균형 제어를 수행한다.

**제어 파이프라인 (`control_loop`, 200Hz):**
1. `/joint_states` + `/imu` + `/odom` 수신 → `q`, `v` 조립
2. `pin.rnea(model, data, q, v, 0)` → 중력/코리올리 보상 토크 `tau_rnea`
3. `pin.computeAllTerms(model, data, q, v)` → M, C, G 행렬
4. **Momentum Observer (MOB)**: `tau_ext = Ko * (Mv - ∫(tau + C^T v - G + tau_ext)dt)` → 외력 추정
5. PD 피드백: `tau_pd = Kp * q_err + Kd * v_err` (관절 위치 추종)
6. 합산: `tau_cmd = tau_rnea + tau_pd - tau_ext_joint`
7. **바퀴 균형**: `compute_com_and_theta(q, v)` → `wheel_position_control` → `wheel_attitude_control`
```

**ROS 토픽:**

| 토픽 | 방향 |
|------|------|
| `/joint_states` | 구독 (200Hz) |
| `/imu` | 구독 (200Hz) |
| `/odom` | 구독 (base linear velocity) |
| `/joint_command` | 퍼블리시 |

### Isaac Sim 통합 (`src/`)

- `src/main/00_standalone_scene.py` — `StandaloneEnv` 클래스: 시뮬레이션의 핵심 진입점.
- `src/main/01_connect_scene.py` — 기존 Isaac Sim 씬에 연결하고 ROS2 브리지를 활성화하는 최소 예제.
- USD 에셋은 `.gitignore`로 제외됨; URDF와 메쉬만 추적.

#### `StandaloneEnv` 상세

**생성자 파라미터:**
```python
StandaloneEnv(
    usd_mode,            # "Floating" | "Fixed" | "Fixed_ng"
    usd_path,            # GOAT_ROS.usd 경로
    robot_prim_path,     # "/World/World/Robot"
    physics_dt=1/200,    # 물리 스텝 주기 (200Hz)
    decimation=1,        # 렌더링 주기: 물리 스텝 N회당 1회 렌더 (현재 1=매 스텝 렌더)
)
```

**정상 운영 데이터 흐름:**
```
[Isaac Sim - Windows]
  simulation_context.step()
      ├─ OnPhysicsStep → ROS2Publish* → /joint_states, /imu, /tf (200Hz)
      └─ ros2_subscribe_joint_state → articulation_controller ← /joint_command

  rclpy.spin_once(timeout=remaining)   ← 200Hz 벽시계 고정 + drift 보상

[WSL2 - Linux ROS2]
  /joint_states + /imu 구독
      → 제어 파이프라인 (PD + PI + SafetyLimiter) @ 200Hz
      → /joint_command 퍼블리시
```

### 로봇 모델

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

## Pinocchio & ROS 간 Index Mapping Table
| joint name    | ROS-order | PIN-actuator-order | q idx | v idx | pin joint-tree idx |
| ------------- | --------: | -----------------: | ----: | ----: | -----------------: |
| hip_L_Joint   |         0 |                  0 |     7 |     6 |                  2 |
| hip_R_Joint   |         1 |                  4 |    11 |    10 |                  6 |
| thigh_L_Joint |         2 |                  1 |     8 |     7 |                  3 |
| thigh_R_Joint |         3 |                  5 |    12 |    11 |                  7 |
| knee_L_Joint  |         4 |                  2 |     9 |     8 |                  4 |
| knee_R_Joint  |         5 |                  6 |    13 |    12 |                  8 |
| wheel_L_Joint |         6 |                  3 |    10 |     9 |                  5 |
| wheel_R_Joint |         7 |                  7 |    14 |    13 |                  9 |

---

## Pinocchio 핵심 주의사항

### `data.com[i]` 프레임 규약

| 함수 | `data.com[0]` | `data.com[i > 0]` |
|---|---|---|
| `pin.centerOfMass(model, data, q [,v], True)` | **월드 프레임** | **관절 i 로컬 프레임** ← 함정 |
| `pin.jacobianCenterOfMass(model, data, q, True)` | **월드 프레임** | **월드 프레임** |

`centerOfMass` 호출 후 `data.com[i>0]`를 월드 프레임 값처럼 사용하면 안 된다.
(`data.com[0]`만 월드 프레임; 나머지는 각 관절의 로컬 프레임)

### 올바른 world-frame 변환 방법

```python
pin.centerOfMass(model, data, q, v, compute_subtree_coms=True)

# 위치 (점 변환): R * p_local + t
com_world = data.oMi[joint_pin_id].act(data.com[joint_pin_id])

# 속도 (방향벡터): R * v_local  (평행이동 없음)
vcom_world = data.oMi[joint_pin_id].rotation @ data.vcom[joint_pin_id]
```

### `data.oMi[i]` 이해

- **정체**: FK 실행 후 채워지는 SE3 데이터 객체 (메소드가 아님)
- **의미**: 관절 i의 joint frame이 월드 기준으로 어디에 어떤 방향으로 있는지
- `.rotation` → 3×3 회전행렬 R (방향벡터 변환 시 사용)
- `.translation` → 3×1 위치벡터 t (관절 원점의 월드 좌표)
- `.act(p)` → R*p + t (점 변환, 회전 + 평행이동 모두 적용)
- `computeAllTerms` 또는 `forwardKinematics` 호출 후 유효

### 관련 소스

- Pinocchio 3.9.0 헤더: `/home/grape4314/miniconda3/envs/env_ros/include/pinocchio/algorithm/center-of-mass.hpp`