# 구현 계획: NSC 제어 데이터 플로터

---

## 1. 목표

`nsc_control_node.py`가 계산하는 내부 신호(토크 분해, 진자 상태 등)와
`/joint_states`, `/imu` 토픽 데이터를 한 번에 수집하여 matplotlib으로 시각화한다.

---

## 2. 수정/생성 파일

| 파일 | 작업 |
|---|---|
| `src/goat/goat/nodes/nsc_control_node.py` | `/nsc_debug` 퍼블리셔 추가 |
| `src/goat/goat/nodes/nsc_plotter.py` | 신규 생성 — 수집 노드 + plot 함수들 |
| `src/goat/setup.py` | `nsc_plotter` entry point 등록 |

---

## 3. `/nsc_debug` 토픽 설계

**타입**: `std_msgs/Float64MultiArray`
**퍼블리시 시점**: `control_loop()` 마지막, 매 200Hz 스텝
**데이터 레이아웃** (총 37 floats, 토크 신호는 모두 PIN order):

| 인덱스 | 신호 | 단위 |
|---|---|---|
| 0 | `theta` — 진자 피치각 | rad |
| 1 | `theta_dot` — 진자 각속도 | rad/s |
| 2 | `theta_cmd` — 위치 제어기 출력 | rad |
| 3 | `L` — 진자 길이 | m |
| 4 | `wheel_tau` — 바퀴에 인가된 토크 | Nm |
| 5 ~ 12 | `tau_cmd[0:8]` — 최종 명령 토크 | Nm |
| 13 ~ 20 | `tau_rnea[0:8]` — RNEA 보상 성분 | Nm |
| 21 ~ 28 | `tau_pd[0:8]` — PD 피드백 성분 | Nm |
| 29 ~ 36 | `tau_ext[0:8]` — MOB 외력 추정 성분 | Nm |

### `nsc_control_node.py` 변경 위치

1. **import 추가**: `from std_msgs.msg import Float64MultiArray`
2. **`__init__`**: `self.debug_pub = self.create_publisher(Float64MultiArray, '/nsc_debug', 10)`
3. **`control_loop()` 끝** (주석 처리된 command publish 바로 아래):

```python
debug_msg = Float64MultiArray()
debug_msg.data = [
    float(theta), float(theta_dot), float(theta_cmd), float(L), float(wheel_tau),
    *self.tau_cmd.tolist(),
    *tau_rnea_joint.tolist(),
    *tau_pd.tolist(),
    *self.joint_tau_external.tolist(),
]
self.debug_pub.publish(debug_msg)
```

> **주의**: `tau_rnea_joint`, `tau_pd`는 `control_loop` 내 로컬 변수이지만 Python 함수 스코프상 끝까지 유효. 별도 멤버 변수 불필요.

---

## 4. `nsc_plotter.py` 설계

### 4.1 실행 방식

```bash
ros2 run goat nsc_plotter                    # 30초 수집 후 plot
ros2 run goat nsc_plotter --duration 15      # 15초 수집
ros2 run goat nsc_plotter --save             # PNG 파일 저장 (WSL2 X11 없을 때 권장)
```

**종료 트리거가 두 가지이지만 항상 같은 plot 경로를 탄다:**

```
[수집 루프]
  try:
      while rclpy.ok() and not node.done:   ← done = duration 타이머가 set
          rclpy.spin_once(timeout=0.1)       ← 데이터 계속 버퍼에 누적
  except KeyboardInterrupt:
      pass                                   ← Ctrl-C도 조용히 통과

[공통 종료 처리]
  data = node.arrays()   ← list → numpy 변환
  plot_all(data, save)   ← 두 경로 모두 여기서 합류
  plt.show()
```

- `--duration` 만료: 타이머 콜백이 `node.done = True` → 루프 정상 탈출 → plot
- Ctrl-C: `KeyboardInterrupt` catch → 루프 탈출 → **동일하게** plot
- 두 경우 모두 수집된 데이터 전량을 한 번에 플롯한다.

### 4.2 구독 토픽

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | q, v, tau_meas (ROS order) |
| `/nsc_debug` | `std_msgs/Float64MultiArray` | 내부 신호 전체 |
| `/imu` | `sensor_msgs/Imu` | orientation → Euler 각도 |

### 4.3 데이터 버퍼 구조

```python
buf = {
    # /joint_states (ROS order, shape (N, 8))
    't_js', 'q', 'v', 'tau_meas',

    # /nsc_debug (scalar 또는 shape (N, 8), 토크는 ROS order로 변환 저장)
    't_dbg',
    'theta', 'theta_dot', 'theta_cmd', 'L', 'wheel_tau',
    'tau_cmd', 'tau_rnea', 'tau_pd', 'tau_ext',

    # /imu → Euler 변환
    't_imu', 'roll', 'pitch', 'yaw',
}
```

`_dbg_cb`에서 PIN order → ROS order 변환 적용:
```python
PIN_TO_ROS = [0, 4, 1, 5, 2, 6, 3, 7]
tau_cmd_ros = np.array(d[5:13])[PIN_TO_ROS]
```

### 4.4 Figure 구성 (5개)

각 Figure는 독립 함수로 구현:

| 함수 | Figure | 구성 | 주요 내용 |
|---|---|---|---|
| `plot_joint_positions(data)` | Fig 1 | 4×2 | q (실선) vs q_ref (빨간 점선), 8 관절 |
| `plot_joint_velocities(data)` | Fig 2 | 4×2 | v_curr, 8 관절 |
| `plot_torques(data)` | Fig 3 | 4×2 | tau_cmd / tau_rnea / tau_pd / tau_ext 중첩, 8 관절 |
| `plot_wheel_balance(data)` | Fig 4 | 2×2 | theta+theta_cmd, theta_dot, L, wheel_tau |
| `plot_imu(data)` | Fig 5 | 3×1 | roll, pitch, yaw (deg) |

#### subplot 레이아웃 규칙 (8 관절 → 4×2 grid)

```
axes[i//2, i%2]  (i = 0..7, ROS order)

[0,0] hip_L    [0,1] hip_R
[1,0] thigh_L  [1,1] thigh_R
[2,0] knee_L   [2,1] knee_R
[3,0] wheel_L  [3,1] wheel_R
```

#### TARGET_Q_ROS (q_ref 수평선용)

```python
# PIN order: [hip_L=0, thigh_L=0.738, knee_L=1.462, wheel_L=0, hip_R=0, thigh_R=-0.738, knee_R=-1.462, wheel_R=0]
# → ROS order:
TARGET_Q_ROS = [0.0, 0.0, 0.738, -0.738, 1.462, -1.462, 0.0, 0.0]
```

### 4.5 `plot_all()` 통합 함수

- plot 할 때, 당연히 Reference 신호도 그려야 합니다. 아마 Reference 신호는 빨간색 점선 등으로 표시하면 좋을 듯 합니다.

```python
def plot_all(data: dict, save: bool = False) -> list[Figure]:
    """모든 figure 생성. save=True면 nsc_*.png로 저장."""
    figs = [
        ('joint_positions',  plot_joint_positions(data)),
        ('joint_velocities', plot_joint_velocities(data)),
        ('torques',          plot_torques(data)),
        ('wheel_balance',    plot_wheel_balance(data)),
        ('imu',              plot_imu(data)),
    ]
    if save:
        for name, fig in figs:
            fig.savefig(f'nsc_{name}.png', dpi=150, bbox_inches='tight')
    return [fig for _, fig in figs]
```

### 4.6 main() 흐름

```
argparse (--duration, --save)
  ↓
rclpy.init()
NSCPlotNode(duration) 생성
  ↓
while rclpy.ok() and not node.done:
    rclpy.spin_once(timeout=0.1)
  ↓ (Ctrl-C 또는 duration 만료)
data = node.arrays()   # list → numpy
  ↓
plot_all(data, save=args.save)
plt.show()
```

---

## 5. Launch 파일 (`src/goat/launch/nsc.launch.py`)

두 노드를 동시에 기동하는 ROS2 XML launch 파일을 신규 작성한다. -> .launch.py로 다시 작성합시다.

### 5.1 launch argument

| argument | 기본값 | 설명 |
|---|---|---|
| `duration` | `60.0` | plotter 수집 시간 (초) |
| `save` | `false` | `true` 시 PNG 파일 저장 |

### 5.2 파일 내용
- .launch.py 형식으로 다시 만듭시다.  .xml 형식 안써도 될 것 같아요.
```xml
<launch>
  <arg name="duration" default="10.0"
       description="Plotter data collection duration [s]"/>
  <arg name="save"     default="false"
       description="Save plots as PNG files (true/false)"/>

  <node pkg="goat" exec="nsc_control_node"
        name="nsc_control_node" output="screen"/>

  <node pkg="goat" exec="nsc_plotter"
        name="nsc_plotter" output="screen"
        args="--duration $(var duration) --save $(var save)"/>
</launch>
```

### 5.3 실행

```bash
# 기본 (60초 수집)
ros2 launch goat nsc.launch.xml

# 30초 수집 + PNG 저장
ros2 launch goat nsc.launch.xml duration:=30.0 save:=true
```

> **Ctrl-C 동작**: launch가 SIGINT를 두 노드에 동시에 전달한다.
> `nsc_plotter`는 `KeyboardInterrupt`를 catch하여 수집된 데이터를 플롯한 뒤 종료.
> `nsc_control_node`는 즉시 종료.

---

## 6. `setup.py` 변경

```python
# entry_points
'nsc_plotter = goat.nodes.nsc_plotter:main',

# data_files — launch 디렉토리 추가
('share/' + package_name + '/launch', ['launch/nsc.launch.xml']),
```

---

## 7. 구현 순서

1. `nsc_control_node.py` — import, publisher, debug publish 3곳 추가
2. `nsc_plotter.py` — 신규 작성
3. `launch/nsc.launch.py` — 신규 작성
4. `setup.py` — entry point + data_files 2줄 추가
5. 빌드: `colcon build --packages-select goat --symlink-install`

---

## 8. 고려사항

- **matplotlib 백엔드 (WSL2)**: X11 forwarding 없으면 `Agg`(파일 저장 전용)로 폴백 필요. `--save true` 옵션 사용 권장.
- **`--save` 인자 파싱**: launch 파일에서 문자열 `'true'`/`'false'`로 전달되므로 plotter에서 `args.save.lower() == 'true'`로 처리.
- **토픽 미수신 처리**: 각 plot 함수는 빈 배열(`len == 0`)을 조용히 통과하도록 작성.
- **타임스탬프**: `/joint_states`는 헤더 stamp 사용, `/nsc_debug`는 수신 시각 기준 (`self._now()`).
- **데이터 정렬**: 서로 다른 토픽은 각자의 타임스탬프로 독립 플롯 — 보간 불필요.
