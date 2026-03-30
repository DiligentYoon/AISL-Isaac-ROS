# Research: Isaac Sim ROS2 Topic I/O Architecture

## 1. 현재 `00_standalone_scene.py` 역할 및 한계

### 현재 담당하는 것
- `GOAT_ROS.usd` 로드 및 물리 컨텍스트(`PhysicsContext`) 초기화
- `Articulation` / `XFormPrim` 래퍼 생성
- `reset()` — 루트 포즈 + 관절 초기화
- `step()` — `simulation_app.update()` N회 호출

### 현재 없는 것 (추가 필요)
- ROS2 토픽 퍼블리싱 (`/joint_states`, `/imu`, `/clock`)
- ROS2 토픽 수신 (`/joint_command`) 및 로봇 적용
- OmniGraph(Action Graph) 구성

---

## 2. OmniGraph(Action Graph) — 토픽 퍼블리싱 Hz 제어

### 핵심 원리
Isaac Sim에서 토픽을 주기적으로 퍼블리싱하려면 **OmniGraph Action Graph**를 사용한다.
`OnPlaybackTick` 노드가 **매 `simulation_app.update()` 호출마다 1번** 실행되므로:

```
퍼블리싱 Hz = 1 / physics_dt
```

예: `physics_dt = 1/200` → 200Hz 퍼블리싱

### `00_tutorial_clock.py`에서 확인한 패턴

```python
og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime",    "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PublishClock",   "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick",        "PublishClock.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("PublishClock.inputs:topicName", "clock"),
        ],
    },
)
```

### 우리 시뮬레이션에 필요한 OmniGraph 노드

| 노드 타입 | 역할 | 출력 토픽 |
|-----------|------|-----------|
| `omni.graph.action.OnPlaybackTick` | 매 스텝 트리거 | — |
| `isaacsim.core.nodes.IsaacReadSimulationTime` | 시뮬 시간 읽기 | — |
| `isaacsim.ros2.bridge.ROS2PublishClock` | 시뮬 클록 퍼블리시 | `/clock` |
| `isaacsim.ros2.bridge.ROS2PublishJointState` | 관절 상태 퍼블리시 | `/joint_states` |
| `isaacsim.ros2.bridge.ROS2PublishImu` | IMU 퍼블리시 | `/imu` |

추가로 필요한 Isaac 유틸 노드:
- `isaacsim.core.nodes.IsaacReadSystemTime` — 실시간 타임스탬프
- 아티큘레이션 prim 경로 → 노드 입력으로 연결해 관절 데이터 자동 읽기

---

## 3. ROS2 수신 (WSL2 → Isaac Sim) — 두 가지 방법 비교

### 방법 A: OmniGraph `ROS2SubscribeJointState`

Action Graph 안에 `ROS2SubscribeJointState` 노드를 추가하고,
출력을 `ArticulationController` 노드 입력에 직결하는 방식.

**장점:** 코드 없이 노드 연결만으로 동작
**단점:** 커스텀 제어 로직(PD/PI 파이프라인 삽입 등) 불가, 디버깅 어려움

### 방법 B: `rclpy` subscriber (Python 코드 내, 권장)

```python
# Isaac Sim 내부 rclpy (Omniverse 컴파일 버전)
import rclpy
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node("isaac_goat")
node.create_subscription(JointState, "/joint_command", callback, qos)

# 메인 루프
while simulation_app.is_running():
    simulation_app.update()              # 물리 스텝 + OmniGraph → 토픽 퍼블리시
    rclpy.spin_once(node, timeout_sec=0.0)  # 수신 큐 소진 → 콜백 호출
    robot.set_joint_efforts(latest_torques)  # 받은 토크 적용
```

**장점:** 커스텀 로직 삽입 용이, WSL2 `03_sim_control_node.py`와 동일 인터페이스 유지
**단점:** 코드 복잡도 약간 증가

`01_subscriber.py` 튜토리얼이 이 패턴을 확인해 줌.

---

## 4. 전체 데이터 흐름

```
[Isaac Sim - Windows]
        │
        ▼
  simulation_app.update()   ← physics_dt = 1/200 (200Hz)
        │
        ├─→ [OmniGraph: OnPlaybackTick fires]
        │         ├─→ ROS2PublishJointState  →  /joint_states  (200Hz)
        │         ├─→ ROS2PublishImu         →  /imu           (200Hz)
        │         └─→ ROS2PublishClock       →  /clock         (200Hz)
        │
        └─→ rclpy.spin_once(timeout_sec=0.0)
                  └─→ /joint_command 콜백
                            └─→ robot.set_joint_efforts(torques)


[WSL2 / Linux ROS2 - 03_sim_control_node.py]
  /joint_states + /imu 구독
        │
        ▼
  제어 파이프라인 (PD + PI + SafetyLimiter)  @ 200Hz timer
        │
        ▼
  /joint_command 퍼블리시  →  Isaac Sim 수신
```

---

## 5. 주요 참고 소스

| 파일 | 참고 내용 |
|------|-----------|
| `src/tutorials/00_tutorial_clock.py` | OmniGraph 구성, OnPlaybackTick / OnImpulseEvent 패턴 |
| `src/tutorials/01_subscriber.py` | rclpy + World.step() 루프 패턴 |
| `src/main/02_test_topic_io.py` | WSL2 쪽 subscriber/publisher QoS 설정 |
| `src/main/03_sim_control_node.py` | 200Hz 제어 루프, /joint_command 퍼블리시 |

---

## 6. 미확인 사항 (구현 전 검증 필요)

1. `ROS2PublishJointState` 노드의 정확한 입력 핀 이름 및 아티큘레이션 prim 연결 방법
2. `ROS2PublishImu` 노드의 IMU prim 경로 설정 방식
3. Isaac Sim 내부 `rclpy`의 `spin_once` 동작 — WSL2 DDS 통신 시 latency 특성
4. `set_joint_efforts()` vs `set_joint_positions()` — effort 제어 모드 활성화 조건
