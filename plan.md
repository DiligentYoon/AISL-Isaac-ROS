# Plan: 제어 입력 수신 전 시뮬레이션 대기 기능 ✅ 완료

## 목표
`00_standalone_scene.py`에서 외부 WSL2 ROS2 제어 노드가 `/joint_command` 토픽을 보내기 전까지
물리 시뮬레이션 스텝을 실행하지 않고 대기하는 기능 추가.

---

## 해결한 문제들

### 1차: 대기 중 물리 엔진이 계속 동작 ✅
- **원인**: 타임라인 Play 상태에서 `simulation_app.update()` 호출 시 물리도 함께 진행됨
- **해결**: `wait_for_first_command()` 진입 시 `simulation_context.pause()`, 수신 후 `play()`

### 2차: 데드락 — 시뮬레이션과 제어 노드가 서로를 기다림 ✅
- **원인**: pause 상태에서 Action Graph의 `/joint_states` 발행 중단 → 제어 노드가 `/joint_command`를 보내지 못함 → 순환 의존
- **해결**: pause 상태에서 rclpy publisher로 `/joint_states`를 수동 발행하여 데드락 해소. 제어 입력 수신 후 수동 publisher `destroy` → Action Graph가 자동 발행 이어받음

---

## 최종 구현 내용

### 변경 파일
| 파일 | 변경 내용 |
|---|---|
| `src/main/00_standalone_scene.py` | `_command_received` 플래그, `/joint_command` rclpy 구독자, `wait_for_first_command()` 메서드 (pause + 수동 JointState 발행 + play 재개) |

### 최종 시작 시퀀스
```
load_scene() → initialize_handles() → reset() → wait_for_first_command() → step() 루프

wait_for_first_command() 내부:
  pause() → rclpy로 /joint_states 수동 발행 → /joint_command 수신 대기
  → 수신 시 destroy_publisher() → play() → step() 루프 진입
```
