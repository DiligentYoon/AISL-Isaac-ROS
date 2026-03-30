# CLAUDE.md

이 파일은 Claude Code (claude.ai/code)가 이 저장소에서 작업할 때 참고하는 가이드입니다.

## 협업 규칙

- **대화**: 한국어로 소통
- **코드 및 주석**: 영문으로 작성

## 프로젝트 개요

**GOAT (Wheel-Foot Bi-pedal)** — 6개의 다리 관절(각 다리 3개)과 2개의 바퀴를 가진 이족보행 로봇의 시뮬레이션 및 제어 스택.

두 가지 핵심 디렉토리:
- `references/` — **참고용** ROS2 제어 스택 패키지. 여기 있는 Control 로직, State Processing 등을 Isaac Sim 시뮬레이션에 그대로 옮겨 구현하는 것이 목표.
- `src/` — **실제 작업 공간**. Isaac Sim 통합 스크립트 및 USD/URDF 로봇 에셋.

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

# I/O 검증
python src/main/02_test_topic_io.py              # 수신 모니터링
python src/main/02_test_topic_io.py --test-all   # /joint_command 송신 테스트

# 제어 루프 실행 (PD + PI 파이프라인)
python src/main/03_sim_control_node.py
python src/main/03_sim_control_node.py --mode nsc  # natural standing + 바퀴 균형
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

**ROS2 토픽 I/O (USD 내 Action Graph `/World/World/ActionGraph`가 자동 처리):**

| 토픽 | 방향 | 담당 노드 |
|------|------|-----------|
| `/joint_states` | 퍼블리시 (200Hz) | `ros2_publish_joint_state` |
| `/imu` | 퍼블리시 (200Hz) | `ros2_publish_imu` |
| `/tf` | 퍼블리시 (200Hz) | `ros2_publish_transform_tree` |
| `/joint_command` | 구독 → 로봇 적용 | `ros2_subscribe_joint_state` → `articulation_controller` (force mode) |

**실시간 동기화 방식:**
- `OnPhysicsStep` 노드 기반 → 물리 스텝마다 토픽 퍼블리시 (렌더 독립)
- `step()` 내부에서 `rclpy.spin_once()` + 절대 목표 시각 기반 drift 보상으로 200Hz 벽시계 고정
- 200Hz 물리, 200Hz 렌더링 (decimation=1 기준)

**시작 시퀀스 (wait-for-command):**
```
reset() → pause() → wait_for_first_command()
  ├─ 물리 정지 상태에서 rclpy로 /joint_states 수동 발행 (default pose)
  ├─ 제어 노드가 /joint_states 수신 → /joint_command 발행
  ├─ /joint_command 수신 감지 → play() → 물리 재개
  └─ 이후 Action Graph가 /joint_states 자동 발행 이어받음
```
> **배경:** pause 상태에서는 Action Graph의 OnPhysicsStep이 발동하지 않아
> `/joint_states`가 발행되지 않음. 제어 노드는 `/joint_states` 없이는
> `/joint_command`를 보내지 않으므로 데드락 발생. 이를 깨기 위해
> 대기 중 rclpy publisher로 초기 상태를 수동 발행.

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

- **DOF**: 8개 — `hip_L/R`, `thigh_L/R`, `knee_L/R`, `wheel_L/R`
- **무릎 기어비**: 0.5:1 (나머지 관절 1:1)
- **토크 상수**: 0.2616 Nm/A (다리), 0.2478 Nm/A (바퀴)
- Joints 0~5: position PD control (target = natural_pos + delta_action)
- Joints 6~7: wheel speed PI control (with conditional integration anti-windup)
