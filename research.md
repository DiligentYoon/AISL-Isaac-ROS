# `nsc_control_node_2.py` Pinocchio API & Coordinate Frame Analysis

> Date: 2026-04-03
> Target file: `src/goat/goat/nodes/nsc_control_node_2.py`
> Pinocchio version: 3.9.0
> Algorithm: Contact-consistent inverse dynamics via nullspace projection

---

## 0. Algorithm Overview

`nsc_control_node_2.py` implements a constrained inverse dynamics controller:
1. Contact Jacobian `Jc` computation at wheel-ground contact points
2. Constraint-consistent acceleration projection: `min ||qdd - qdd_nom||^2  s.t. Jc qdd = -Jdot_c v`
3. Nullspace projection to eliminate contact forces: `Qu^T (M qdd + h) = Qu^T S_leg^T tau_leg + Qu^T S_wheel^T tau_wheel`
4. Least-squares solve for leg torques

---

## 1. Index Mapping Verification

### 1.1 ROS <-> Pinocchio Mapping (lines 87-100)

CLAUDE.md index table 대비 검증:

| joint name    | ROS idx | PIN-actuator idx | `ros_to_pin_ids` | `pin_to_ros_ids` |
|---------------|---------|-----------------|-------------------|-------------------|
| hip_L_Joint   | 0       | 0               | 0→0 ✅            | 0→0 ✅            |
| hip_R_Joint   | 1       | 4               | 1→2 ?             | 4→1 ?             |
| thigh_L_Joint | 2       | 1               | 2→4 ?             | 1→4 ?             |
| thigh_R_Joint | 3       | 5               | 3→6 ?             | 5→5 ?             |
| knee_L_Joint  | 4       | 2               | 4→1 ?             | 2→2 ?             |
| knee_R_Joint  | 5       | 6               | 5→3 ?             | 6→6 ?             |
| wheel_L_Joint | 6       | 3               | 6→5 ?             | 3→3 ?             |
| wheel_R_Joint | 7       | 7               | 7→7 ✅            | 7→7 ✅            |

코드의 mapping:
```python
ros_to_pin_ids = [0, 2, 4, 6, 1, 3, 5, 7]
```
의미: `pin_array = ros_array[ros_to_pin_ids]`

검증 — ROS 배열 `[hip_L(0), hip_R(1), thigh_L(2), thigh_R(3), knee_L(4), knee_R(5), wheel_L(6), wheel_R(7)]`에서:
- pin[0] = hip_L    → ros[0] = hip_L ✅
- pin[1] = thigh_L  → ros[2] = thigh_L ✅
- pin[2] = knee_L   → ros[4] = knee_L ✅
- pin[3] = wheel_L  → ros[6] = wheel_L ✅
- pin[4] = hip_R    → ros[1] = hip_R ✅
- pin[5] = thigh_R  → ros[3] = thigh_R ✅
- pin[6] = knee_R   → ros[5] = knee_R ✅
- pin[7] = wheel_R  → ros[7] = wheel_R ✅

**결론: ROS <-> Pinocchio index mapping 정상** ✅

### 1.2 Wheel Joint ID (lines 103-112)

```python
wheel_L_joint_id = pin_name_to_idx['wheel_L_Joint']   # 3 (pin-actuator-order)
wheel_R_joint_id = pin_name_to_idx['wheel_R_Joint']   # 7 (pin-actuator-order)

wheel_L_joint_pin_id = model_names.index('wheel_L_Joint')  # 5 (joint-tree index)
wheel_R_joint_pin_id = model_names.index('wheel_R_Joint')  # 9 (joint-tree index)

wheel_L_joint_nv_id = 6 + wheel_L_joint_id  # 6 + 3 = 9  (nv index)
wheel_R_joint_nv_id = 6 + wheel_R_joint_id  # 6 + 7 = 13 (nv index)
```

CLAUDE.md table 대비:
| joint | pin-actuator | v idx | pin joint-tree idx |
|-------|-------------|-------|--------------------|
| wheel_L | 3 | 9 | 5 |
| wheel_R | 7 | 13 | 9 |

**결론: 모든 wheel index 정상** ✅

---

## 2. Selection Matrix Verification (lines 135-145)

### 2.1 `S_leg` (6×14)

```python
S_leg[0, 6]  = 1.0   # hip_L     → v idx 6  ✅
S_leg[1, 7]  = 1.0   # thigh_L   → v idx 7  ✅
S_leg[2, 8]  = 1.0   # knee_L    → v idx 8  ✅
S_leg[3, 10] = 1.0   # hip_R     → v idx 10 ✅
S_leg[4, 11] = 1.0   # thigh_R   → v idx 11 ✅
S_leg[5, 12] = 1.0   # knee_R    → v idx 12 ✅
```

### 2.2 `S_wheel` (2×14)

```python
S_wheel[0, 9]  = 1.0   # wheel_L  → v idx 9  ✅
S_wheel[1, 13] = 1.0   # wheel_R  → v idx 13 ✅
```

**결론: Selection matrix 정상** ✅

---

## 3. COM & Theta Computation (lines 314-354)

### 3.1 World-frame 변환

```python
pin.centerOfMass(self.model, self.data, q, v, compute_subtree_coms=True)

com_wheel_L = self.data.oMi[self.wheel_L_joint_pin_id].act(
    self.data.com[self.wheel_L_joint_pin_id])          # oMi.act() = R*p + t ✅
vcom_wheel_L = self.data.oMi[self.wheel_L_joint_pin_id].rotation @ \
    self.data.vcom[self.wheel_L_joint_pin_id]          # rotation @ v ✅
```

`data.com[i>0]`는 joint i 로컬 프레임이므로 `oMi.act()` 변환 필요 — 이전 분석에서 확인된 수정이 올바르게 적용됨.

**결론: COM world-frame 변환 정상** ✅

### 3.2 `computeAllTerms` 호출 순서

`control_loop()`에서 `computeAllTerms()`가 먼저 호출되므로, `data.oMi`가 이미 유효하다.
이후 `centerOfMass()` 호출로 `data.com`, `data.vcom`, `data.mass`가 채워진다.
`centerOfMass()`는 내부적으로 FK를 재계산하지 않고 subtree COM만 계산하므로 `data.oMi`는 보존된다.

**결론: 호출 순서 정상** ✅

---

## 4. Contact Jacobian — `compute_contact_jacobian()` (lines 410-447)

### 4.1 API 호출 정합성

```python
pin.computeJointJacobians(self.model, self.data, self.q_curr)
pin.updateFramePlacements(self.model, self.data)

rf = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED

placement_L = pin.SE3(np.eye(3), np.array([0.0, 0.0, -self.wheel_radius]))

J6_L = pin.getFrameJacobian(
    self.model, self.data,
    self.wheel_L_joint_pin_id,   # JointIndex (= 5)
    placement_L,                  # SE3 offset from joint frame
    rf,
)
```

**Pinocchio 3.9.0 헤더 확인 (`frames.hpp` line 269-307):**

```cpp
Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Options> getFrameJacobian(
    const ModelTpl & model,
    DataTpl & data,
    const JointIndex joint_id,          // ← JointIndex (NOT FrameIndex)
    const SE3Tpl & placement,           // ← SE3 offset
    const ReferenceFrame reference_frame);
```

5-argument overload는 `JointIndex + SE3 placement`를 받는다.
`self.wheel_L_joint_pin_id = 5`는 `model.names.index('wheel_L_Joint')`로 얻은 **JointIndex** 값이다.

**결론: API 호출 자체는 정상 — JointIndex + placement 오버로드에 정확히 dispatch** ✅

### 4.2 ⚠️ CRITICAL — Placement가 바퀴와 함께 회전하는 문제

**내부 구현 (`frames.hxx` line 172):**
```cpp
const typename Data::SE3 oMframe = data.oMi[joint_id] * placement;
```

`oMi[wheel_joint]`은 바퀴 회전을 포함하는 SE3 변환이다. 따라서:

```
oMframe = oMi[wheel] * SE3(I, [0,0,-r])
```

바퀴가 각도 θ만큼 회전했을 때 (y축 회전):
```
oMi[wheel].rotation = R_parent * R_y(θ)
```

`placement = SE3(I, [0,0,-r])`이 바퀴 joint frame 기준이므로, world에서의 offset 위치:
```
contact_world = R_parent * R_y(θ) * [0, 0, -r]^T + t_wheel
              = R_parent * [r*sin(θ), 0, -r*cos(θ)]^T + t_wheel
```

**θ = 0 (정지)**: offset = `[0, 0, -r]` → 바퀴 바닥 ✅
**θ = π/2 (90° 회전)**: offset = `[r, 0, 0]` → 바퀴 옆면 ❌
**θ = π (180° 회전)**: offset = `[0, 0, r]` → 바퀴 꼭대기 ❌

**문제**: 바퀴가 회전하면 placement가 따라 돌아 실제 지면 접촉점과 어긋난다.
WBR은 바퀴가 지속적으로 회전하므로, 이 오차는 무시할 수 없다.

### 4.3 물리적 분석 — 접촉 구속 조건의 의미

WBR의 wheel-ground 접촉 구속 조건:

**(a) Normal 구속 (지면 관통 방지):**
실제로 필요한 것: `v_z(wheel_center) = 0` (바퀴 중심이 수직으로 움직이지 않음)
→ 바퀴 중심의 Jacobian만으로 충분, offset이 필요 없음.

**(b) Lateral 구속 (횡방향 미끄러짐 방지):**
실제 접촉점 속도: `v_contact = v_center + ω × r_ground`
여기서 `r_ground = [0, 0, -r]` (world z-down 방향, 항상 아래를 가리킴)

바퀴 spin `ω_y`에 대해:
```
[0, ω_y, 0] × [0, 0, -r] = [-ω_y*r, 0, 0]
```
→ spin은 forward(x) 방향에만 기여하고, lateral(y) 방향에는 기여하지 않음.
따라서 lateral 구속에도 바퀴 중심 속도만 필요.

**결론: Normal/Lateral 구속 모두에서 contact offset `[0,0,-r]`이 바퀴와 함께 회전하면 안 된다.**

### 4.4 수정 방안

**방법 A — 바퀴 중심 Jacobian 사용 (offset 제거):**
```python
placement_L = pin.SE3.Identity()  # offset 없음
placement_R = pin.SE3.Identity()

J6_L = pin.getFrameJacobian(self.model, self.data,
    self.wheel_L_joint_pin_id, placement_L, rf)
```
Normal 구속: `n^T @ Jv_L @ qdot = 0` → 바퀴 중심의 수직 속도 = 0은 접촉 유지 조건과 동치
Lateral 구속: `t^T @ Jv_L @ qdot = 0` → 바퀴 spin은 y 방향에 기여하지 않으므로 바퀴 중심 = 접촉점

바퀴 중심이 지면에서 항상 r만큼 위에 있으므로, 바퀴 중심의 수직 속도가 0이면 접촉이 유지된다.
횡방향은 바퀴 spin이 기여하지 않으므로 바퀴 중심 속도 = 접촉점 속도.

**방법 B — 부모 링크(calf)에 고정 접촉 프레임 정의:**
calf_L_Link 기준으로 `[0, offset_y, -(200mm + r)]` 위치에 placement를 정의.
이 경우 바퀴 회전의 영향을 받지 않는다.

**방법 C — 실시간으로 바퀴 역회전 보정:**
```python
theta_w = self.joint_q_curr[self.wheel_L_joint_id]
R_inv = pin.SE3(pin.exp3(np.array([0, -theta_w, 0])), np.array([0,0,-r]))
# 바퀴 회전을 역으로 상쇄하여 항상 아래를 가리키게 함
```

**추천: 방법 A** — 가장 단순하고 물리적으로 정확.

---

## 5. Contact Jacobian Time Derivative — `compute_contact_jacobian_dot_times_v()` (lines 450-498)

### 5.1 API 호출 정합성

```python
pin.computeJointJacobiansTimeVariation(self.model, self.data, self.q_curr, self.v_curr)
pin.updateFramePlacements(self.model, self.data)

a_zero = np.zeros(self.model.nv)
pin.forwardKinematics(self.model, self.data, self.q_curr, self.v_curr, a_zero)
pin.updateFramePlacements(self.model, self.data)

acc_L = pin.getFrameClassicalAcceleration(
    self.model, self.data,
    self.wheel_L_joint_pin_id,
    placement_L,
    rf
)
```

**Pinocchio 3.9.0 헤더 (`frames.hpp` line 193-199):**
```cpp
MotionTpl getFrameClassicalAcceleration(
    const ModelTpl & model,
    const DataTpl & data,
    const JointIndex joint_id,
    const SE3Tpl & placement,
    const ReferenceFrame rf = LOCAL);
```

헤더 주석 (`frames.hpp` line 157-160):
> In the context of a frame placement constraint `J(q) a + Jdot(q,v) v = 0`,
> one way to compute `Jdot(q,v) v` is to call second-order forwardKinematics
> with a zero acceleration, then read the remaining `Jdot(q,v) v` by calling this function.

**결론: API 사용 패턴 정상** ✅

### 5.2 ⚠️ 동일한 Placement 회전 문제

Section 4.2와 동일한 문제가 여기에도 적용된다. `getFrameClassicalAcceleration`도 내부적으로 `oMi[joint_id] * placement`를 사용하므로, 바퀴와 함께 회전하는 점의 가속도를 계산한다.

**Section 4의 수정과 동시에 수정 필요.**

### 5.3 `computeJointJacobiansTimeVariation` 호출의 불필요성

`getFrameClassicalAcceleration`은 `forwardKinematics(q, v, a)`의 결과만 필요하다.
`computeJointJacobiansTimeVariation`의 결과(`dJ`)는 사용되지 않는다.

불필요한 호출이 성능에만 영향을 미치고 결과에는 영향 없음.

**결론: 기능 오류 없음, 성능 최적화 가능** ⚠️ (minor)

---

## 6. Constraint-Consistent Acceleration Projection (lines 512-528)

### 6.1 수학적 검증

목표: `min ||qdd - qdd_nom||^2  s.t. Jc @ qdd = -Jcdot_v`

코드:
```python
JJt_reg = Jc @ Jc.T + damping * I
rhs = Jc @ qdd_nom + Jcdot_v
correction = Jc.T @ np.linalg.solve(JJt_reg, rhs)
qdd_d = qdd_nom - correction
```

검증:
```
Jc @ qdd_d = Jc @ qdd_nom - Jc @ Jc^T @ (Jc @ Jc^T)^{-1} @ (Jc @ qdd_nom + Jcdot_v)
            = Jc @ qdd_nom - (Jc @ qdd_nom + Jcdot_v)
            = -Jcdot_v  ✅
```

이 해는 `qdd_nom`에 가장 가까우면서 구속 조건을 만족하는 least-norm correction이다.

**결론: Projection 수학 정확** ✅

---

## 7. Nullspace Projection & Reduced Dynamics (lines 501-546)

### 7.1 Nullspace 계산 (lines 501-509)

```python
U, S, Vt = np.linalg.svd(Jc, full_matrices=True)   # Jc: 4×14
rank = np.sum(S > tol)
Qu = Vt.T[:, rank:]   # (14, 14-rank) = (14, 10)
```

`Jc`는 4×14 (4개 구속, nv=14). `Vt.T[:, rank:]`는 Jc의 null space basis.
`Jc @ Qu = 0` 보장.

**결론: Nullspace 계산 정확** ✅

### 7.2 Reduced Dynamics Solve (lines 531-546)

운동방정식: `M qdd + h = S_leg^T tau_leg + S_wheel^T tau_wheel + Jc^T lambda`

Nullspace 투영 (`Qu^T Jc^T = 0`이므로 contact force 소거):
```
Qu^T (M qdd + h) = Qu^T S_leg^T tau_leg + Qu^T S_wheel^T tau_wheel
```

코드:
```python
rhs_full = M @ qdd_d + h - S_wheel.T @ tau_w
A = Qu.T @ S_leg.T        # (10, 6)
b = Qu.T @ rhs_full       # (10,)
tau_j = np.linalg.pinv(A) @ b   # (6,) — least-squares solution
```

`A`가 10×6으로 overdetermined → `pinv`가 least-squares 해를 준다.
이는 constrained dynamics와 일관된 leg torque를 계산한다.

**결론: Reduced dynamics 수학 정확** ✅

### 7.3 Torque Assembly (lines 280-281)

```python
tau_constrained_full = S_leg.T @ tau_constrained + S_wheel.T @ np.array([wheel_tau, -wheel_tau])
```

- `S_leg.T`: (14, 6) — 6개 다리 토크를 nv 공간에 배치
- `S_wheel.T`: (14, 2) — 2개 바퀴 토크를 nv 공간에 배치
- 결과: (14,) 전체 generalized force

`tau_cmd = tau_constrained_full[6:]`: base 6DOF 제거 → 8개 관절 토크 (pin-actuator order)

**결론: Torque assembly 정상** ✅

---

## 8. Wheel Control (lines 252-279, 357-379)

### 8.1 Wheel Torque 부호

```python
wheel_tau = self.wheel_Kp_att * theta_err + self.wheel_Kd_att * theta_dot
# ...
np.array([wheel_tau, -wheel_tau])   # [wheel_L, wheel_R]
```

URDF 축: `wheel_L axis="0 1 0"`, `wheel_R axis="0 -1 0"`.
같은 방향으로 굴리려면 반대 부호 토크 필요 → `[wheel_tau, -wheel_tau]` ✅

### 8.2 PD 에서 Wheel 항 제거 (lines 264-268)

```python
self.a_ref[6:] = self.Kp @ q_err + self.Kd @ v_err
self.a_ref[6 + self.wheel_L_joint_id] = 0.0   # nv idx 9 ← 6+3
self.a_ref[6 + self.wheel_R_joint_id] = 0.0   # nv idx 13 ← 6+7
```

바퀴는 별도의 균형 제어로 관리하므로 PD 대상에서 제외. 정상. ✅

---

## 9. 종합 요약

| # | 항목 | 상태 | 심각도 |
|---|------|------|--------|
| 1 | ROS <-> Pinocchio index mapping | ✅ 정상 | — |
| 2 | Selection matrix (S_leg, S_wheel) | ✅ 정상 | — |
| 3 | COM world-frame 변환 | ✅ 정상 (v1 버그 수정 반영) | — |
| 4 | `getFrameJacobian` API overload dispatch | ✅ 정상 (JointIndex + SE3) | — |
| 5 | **Contact placement 바퀴 회전 동기화** | ❌ 버그 | **CRITICAL** |
| 6 | `getFrameClassicalAcceleration` 동일 문제 | ❌ 버그 | **CRITICAL** |
| 7 | Constraint-consistent projection 수학 | ✅ 정상 | — |
| 8 | Nullspace + reduced dynamics 수학 | ✅ 정상 | — |
| 9 | Torque assembly + publish | ✅ 정상 | — |
| 10 | Wheel 부호 규약 | ✅ 정상 | — |
| 11 | `computeJointJacobiansTimeVariation` 미사용 | ⚠️ 불필요 | minor |

### CRITICAL 버그 상세

**`compute_contact_jacobian()` 및 `compute_contact_jacobian_dot_times_v()`의 placement `SE3(I, [0,0,-r])`이 바퀴 joint frame 기준이어서, 바퀴가 회전하면 접촉점이 지면에서 벗어남.**

Pinocchio 내부에서 `oMi[joint] * placement`로 world-frame 좌표를 계산하므로,
placement offset이 바퀴의 회전 행렬에 의해 함께 회전한다.

**추천 수정: placement를 `SE3.Identity()`로 변경하여 바퀴 중심의 Jacobian을 사용.**
- Normal 구속 (`v_z = 0`): 바퀴 중심의 수직 속도 = 0은 접촉 유지 조건과 동치
- Lateral 구속 (`v_y = 0`): 바퀴 spin은 y 방향에 기여하지 않으므로 바퀴 중심 = 접촉점

---

## Appendix: Pinocchio 함수 시그니처 (3.9.0 기준, 설치된 헤더에서 확인)

### `getFrameJacobian` — JointIndex + Placement overload
```
Source: pinocchio/algorithm/frames.hpp, line 269-307
Signature: getFrameJacobian(model, data, JointIndex, SE3 placement, ReferenceFrame) → Matrix6xN
Prerequisite: computeJointJacobians(model, data, q)
Implementation: oMframe = oMi[joint_id] * placement  (frames.hxx:172)
```

### `getFrameClassicalAcceleration` — JointIndex + Placement overload
```
Source: pinocchio/algorithm/frames.hpp, line 193-199
Signature: getFrameClassicalAcceleration(model, data, JointIndex, SE3 placement, ReferenceFrame) → Motion
Prerequisite: forwardKinematics(model, data, q, v, a)
Note: With a=0, returns Jdot(q,v)*v (as per header comment, line 157-160)
```
