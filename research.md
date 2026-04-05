# NSC Control Node v2 — Pinocchio Gravity Compensation & Dynamics Analysis

> Date: 2026-04-05
> Target file: `src/goat/goat/nodes/nsc_control_node_2.py`
> Pinocchio version: 3.9.0
> Focus: 중력 보상 정확성, Pinocchio 사용 절차 검증, Wheel-Leg 정합성 분석

---

## 1. Control Architecture Overview

```
[Sensor Sync Callback @ 200Hz]
    ├─ joint_callback → q_joint, v_joint (Pinocchio ordering)
    ├─ imu_callback   → quaternion, angular_vel, linear_acc
    └─ odom_callback  → base linear velocity (body frame)
         │
         v
[control_loop]
    │
    ├─ pin.computeAllTerms(q, v)  → M, C, G
    │
    ├─ [Wheel Controller]
    │   Outer (100Hz): phi_err → theta_ref
    │   Inner (200Hz): theta_err → wheel_tau
    │
    ├─ [Leg Controller]
    │   PD: q_err, v_err → a_ref
    │   Contact-consistent projection → a_ref_constrained
    │   Reduced dynamics solve (wheel_tau as input) → tau_leg
    │
    └─ tau_cmd = S_leg.T @ tau_leg + S_wheel.T @ [wheel_tau, -wheel_tau]
         │
         v
    [Publish /joint_command]
```

---

## 2. URDF Joint Axis Summary

| Joint | Axis | Positive rotation direction |
|-------|------|-----------------------------|
| hip_L_Joint | [-1, 0, 0] | abduction (outward) |
| hip_R_Joint | [-1, 0, 0] | abduction (outward) |
| thigh_L_Joint | [0, 1, 0] | forward flexion |
| thigh_R_Joint | [0, -1, 0] | forward flexion (mirrored) |
| knee_L_Joint | [0, -1, 0] | extension |
| knee_R_Joint | [0, 1, 0] | extension (mirrored) |
| wheel_L_Joint | [0, 1, 0] | forward rolling |
| wheel_R_Joint | [0, -1, 0] | forward rolling (mirrored) |

TARGET_JOINT_ANGLE (Pin order):
```
hip_L=0.0, thigh_L=0.738, knee_L=1.462, wheel_L=0.0
hip_R=0.0, thigh_R=-0.738, knee_R=-1.462, wheel_R=0.0
```

L/R 미러링 (부호 반전 + 축 반전) → 물리적으로 동일한 좌우 대칭 자세. ✅

---

## 3. Pinocchio Computation Chain — Step-by-Step 검증

### 3.1 State Assembly (lines 257-260)

```python
self.base_q_curr = np.concatenate((np.zeros(3), self.base_quat_curr))
self.base_v_curr = np.concatenate((self.base_linear_v_curr, self.base_w_curr))
self.q_curr = np.concatenate((self.base_q_curr, self.joint_q_curr))
self.v_curr = np.concatenate((self.base_v_curr, self.joint_v_curr))
```

**Pinocchio FreeFlyer 기대 형식:**
- `q[0:3]` = base position XYZ (world frame)
- `q[3:7]` = base quaternion [x, y, z, w] → `R_world_base`
- `q[7:]` = joint positions (model joint order)
- `v[0:3]` = base linear velocity (LOCAL/body frame)
- `v[3:6]` = base angular velocity (LOCAL/body frame)
- `v[6:]` = joint velocities

**검증 항목:**

| 항목 | 기대값 | 코드 | 판정 |
|------|--------|------|------|
| base position | world XYZ | `np.zeros(3)` (고정) | ⚠️ 아래 3.2 참고 |
| base quaternion | [x,y,z,w] = R_world_base | IMU orientation | ⚠️ 아래 3.3 참고 |
| base linear vel | body frame | odom twist.linear | ✅ (ROS Odom 표준) |
| base angular vel | body frame | IMU angular_velocity | ✅ (IMU 표준) |
| velocity order | [linear, angular] | `concat(linear_v, w)` | ✅ |
| joint ordering | Pin model order | `ros_to_pin_ids` 적용 | ✅ (이전 검증) |

### 3.2 Base Position = Zero

```python
self.base_q_curr = np.concatenate((np.zeros(3), self.base_quat_curr))
```

Base XYZ가 항상 `[0, 0, 0]`. 실제로는 지면 위 ~0.3m에 위치.

**중력 보상에 대한 영향:**
- `G = ∂V/∂q` where `V = -Σ m_i g z_i(q)`
- `G_joint[j] = ∂V/∂q_j = -g Σ m_i ∂z_i/∂q_j`
- 관절 중력 토크는 **관절 각도와 base orientation에만 의존**, base position에는 무관
- `G_base[0:3]` = [-m*g*0, -m*g*0, -m*g*1] (body frame) — position 무관

**결론: base position이 0이어도 중력 보상에는 영향 없음** ✅

### 3.3 ⚠️ [CRITICAL CHECK] Quaternion Convention

코드:
```python
self.base_quat_curr = np.array([msg.orientation.x, y, z, w])
```

**Pinocchio 기대**: `q[3:7] = [x, y, z, w]` → rotation `R_world_base` (base를 world로 변환)

**Isaac Sim IMU 출력**: 일반적으로 `R_world_sensor` (sensor frame → world frame)

만약 IMU가 `R_sensor_world` (world → sensor)를 출력한다면:
- Pinocchio가 계산하는 R이 전치됨
- `data.g`의 base wrench 부분 `g[0:6]`이 잘못된 좌표계에서 계산됨
- FK를 통해 `data.g[6:]` (관절 중력)도 **부호가 뒤바뀜**
- → 중력 보상이 반대 방향으로 작용 → **로봇 붕괴**

**이것이 가장 유력한 root cause입니다.**

**진단 방법:**
```python
# 로봇이 직립 상태일 때 quaternion 출력
print(f"Quaternion (xyzw): {self.base_quat_curr}")
# 기대값: [0, 0, 0, 1] (identity) 또는 이에 가까운 값
# 만약 [0, 0, ±1, 0] 등이면 convention 불일치
```

### 3.4 `computeAllTerms` 호출 및 출력

```python
pin.computeAllTerms(self.model, self.data, self.q_curr, self.v_curr)
M = self.data.M   # Mass matrix (nv × nv)
C = self.data.C   # Coriolis matrix (nv × nv)
G = self.data.g   # Gravity vector (nv,)
```

**Pinocchio 3.x `computeAllTerms` 내부 실행 순서:**
1. `rnea(model, data, q, v, 0)` → `data.nle = data.tau` (nonlinear effects)
2. `crba(model, data, q)` → `data.M` (upper triangular)
3. **M 대칭화**: `M.lower = M.upper.T` (Pinocchio 3.x에서 computeAllTerms가 처리)
4. `computeCoriolisMatrix(model, data, q, v)` → `data.C`
5. `computeGeneralizedGravity(model, data, q)` → `data.g`
6. `computeJointJacobians(model, data, q)` → Jacobian 데이터

**관계 확인:**
- `data.nle = C @ v + g` (at the given q, v)
- `data.g = rnea(model, data, q, 0, 0)` (static gravity torque)
- `tau_static = data.g` 이면 로봇 정지 (EOM: `M @ qdd + C @ v + g = tau`)

### 3.5 Subsequent API Calls — M, C, G 무효화 여부

`computeAllTerms` 이후 호출되는 Pinocchio 함수들:

| 호출 (순서) | 목적 | M 수정? | C 수정? | G 수정? |
|-------------|------|---------|---------|---------|
| `centerOfMass(model, data, q, v, True)` | COM 계산 | ✗ | ✗ | ✗ |
| `computeJointJacobians(model, data, q)` | 접촉 야코비안 | ✗ | ✗ | ✗ |
| `updateFramePlacements(model, data)` | 프레임 배치 | ✗ | ✗ | ✗ |
| `forwardKinematics(model, data, q, v, a_zero)` | Jdot*v 계산 | ✗ | ✗ | ✗ |

**결론**: M, C, G는 `computeAllTerms` 이후 변경되지 않음. Python 변수 `M`, `C`, `G`는 numpy view(참조)이므로 Pinocchio가 내부적으로 수정하지 않는 한 유효. ✅

단, `M`이 reference이므로 copy 없이 사용 중 — **`computeAllTerms`가 M을 대칭화하지 않는 버전이면 문제**. 아래 진단에서 확인 필요.

---

## 4. Reduced Dynamics에서의 중력 보상 수학적 검증

### 4.1 운동 방정식

Pinocchio 운동방정식:
```
M(q) @ qdd + C(q,v) @ v + g(q) = S_leg.T @ tau_leg + S_wheel.T @ tau_wheel + Jc.T @ lambda
```

### 4.2 Nullspace Projection

Contact force `lambda`를 소거하기 위해 `Qu.T` (Jc의 nullspace basis) 곱:
```
Qu.T @ (M @ qdd + C @ v + g) = Qu.T @ (S_leg.T @ tau_leg + S_wheel.T @ tau_wheel)
```

### 4.3 Code에서의 Solver

```python
h = C @ v + G                                    # nonlinear effects
rhs_full = M @ qdd_d + h - S_wheel.T @ tau_w     # RHS after subtracting wheel
A = Qu.T @ S_leg.T                                # (12, 6)
b = Qu.T @ rhs_full                               # (12,)
tau_leg = pinv(A) @ b                              # (6,) least-squares
```

**정적 상태** (v=0, qdd_d=0, wheel_tau=0):
```
rhs_full = G
b = Qu.T @ G
tau_leg = pinv(Qu.T @ S_leg.T) @ (Qu.T @ G)
```

**물리적 의미**: 접촉 구속을 만족하면서 중력을 보상하는 최소 노름 leg torque.

### 4.4 시스템 일관성 (Consistency)

`A = Qu.T @ S_leg.T` shape = (12, 6). 12 equations, 6 unknowns → **overdetermined**.

물리적으로 로봇이 해당 자세에서 정적 평형을 유지할 수 있다면 (가능한 `tau_leg`이 존재), `b ∈ col(A)`이고 `pinv(A) @ b`는 정확해.

그러나 **`b ∉ col(A)`이면 pinv는 잔차가 있는 근사해를 반환**하고, 중력 보상이 불완전:
```
residual = ||A @ tau_leg - b|| > 0
→ 일부 중력 성분이 보상되지 않음
→ 관절이 중력 방향으로 가속
```

**진단**: `||A @ pinv(A) @ b - b||`를 출력하여 잔차 크기 확인.

---

## 5. 잠재적 오류 분석

### 5.1 [CRITICAL] Quaternion Convention 불일치 가능성

**증상**: "joint가 굽혀지는 방향으로 토크가 걸림" = 중력 보상 방향이 반대.

**원인 가설**: IMU가 `R_base_world` (world→base)를 출력하는데, Pinocchio는 `R_world_base` (base→world)를 기대.

**영향**:
- 중력 벡터가 Pinocchio 내부에서 **반대 방향**으로 해석됨
- `g(q)` 전체의 부호가 뒤바뀜
- 중력 보상 토크가 중력 **방향**으로 작용 → 로봇 붕괴

**진단 코드:**
```python
# control_loop 시작 부분에 추가
R = pin.Quaternion(self.base_quat_curr[3], self.base_quat_curr[0],
                    self.base_quat_curr[1], self.base_quat_curr[2]).matrix()
gravity_in_base = R.T @ np.array([0, 0, -9.81])
print(f"Quat: {self.base_quat_curr}")
print(f"Gravity in base frame: {gravity_in_base}")
# 직립 시 기대값: [0, 0, -9.81] (z-down)
# 만약 [0, 0, +9.81]이면 quaternion convention 반전됨
```

### 5.2 [CRITICAL] `data.g` 직접 검증 필요

RNEA를 통한 직접 중력 보상과 reduced dynamics solver 결과를 비교:

```python
# Method 1: Direct RNEA gravity compensation
tau_grav_rnea = pin.rnea(self.model, self.data, self.q_curr,
                          np.zeros(self.nv), np.zeros(self.nv))

# Method 2: data.g from computeAllTerms (이미 계산됨)
tau_grav_g = self.data.g.copy()

# Method 3: Reduced dynamics solver output
tau_reduced = tau_constrained_full  # 현재 코드의 출력

print(f"RNEA joint gravity: {tau_grav_rnea[6:]}")
print(f"data.g joint:       {tau_grav_g[6:]}")
print(f"Reduced dynamics:   {tau_reduced[6:]}")
print(f"RNEA == data.g?     {np.allclose(tau_grav_rnea, tau_grav_g)}")
```

**기대 결과**: v=0, qdd=0 일 때 세 방법 모두 동일한 중력 보상 토크.

만약 Method 1 ≠ Method 2: `computeAllTerms` 후 다른 API 호출이 `data.g`를 오염시킴.
만약 Method 1 = Method 2 ≠ Method 3: Nullspace projection에서 잔차 발생 (§4.4).
만약 모두 동일하지만 로봇 붕괴: Quaternion convention 문제 (§5.1) 또는 Isaac Sim 인터페이스 문제.

### 5.3 [HIGH] Mass Matrix 대칭성 검증

Pinocchio `crba()`는 **상삼각만** 계산. `computeAllTerms`는 3.x에서 대칭화하지만, 버전에 따라 다를 수 있음.

```python
M = self.data.M.copy()
asymmetry = np.max(np.abs(M - M.T))
print(f"M asymmetry: {asymmetry}")
# 0이어야 함. > 1e-10이면 대칭화 안 됨 → M @ qdd 결과 오류
```

**비대칭 시 수정:**
```python
M = self.data.M.copy()
M = np.triu(M) + np.triu(M, 1).T  # 대칭화
```

### 5.4 [HIGH] URDF dynamics 태그 — 시뮬레이터와의 불일치

URDF에서 확인된 joint dynamics:

| Joint | damping | stiffness | friction |
|-------|---------|-----------|----------|
| hip_L/R | 0.0024 | 0.0 | 0.065 |
| thigh_L/R | 0.0024 | 0.0 | 0.065 |
| knee_L/R | 0.299 | 0.0 | 0.443 |
| **wheel_L/R** | **100.0** | **10000.0** | 0.01 |

**Pinocchio**: URDF의 `<dynamics>` 태그를 **무시**. RNEA/CRBA에 damping, stiffness, friction이 포함되지 않음.

**Isaac Sim**: URDF의 dynamics를 **적용**할 수 있음. 만약 적용 중이라면:
- wheel stiffness = 10000 N·m/rad → 바퀴가 사실상 **스프링 잠금**
- wheel damping = 100 N·m·s/rad → 엄청난 점성 마찰
- 제어기가 보내는 wheel_tau (max 2.5 Nm)로는 스프링력을 극복 불가

**진단**: Isaac Sim에서 바퀴 stiffness/damping 설정 확인. 이 값이 simulation에 반영되는지 확인 필요.
> 만약 Isaac Sim이 USD 프로퍼티를 URDF dynamics 대신 사용한다면 문제없을 수 있음.
> 하지만 URDF import 시 이 값을 그대로 적용한다면, 바퀴 밸런싱이 불가능.

### 5.5 [MEDIUM] Reduced Dynamics Solver 잔차 (Overdetermined System)

`A = Qu.T @ S_leg.T`: (12, 6) overdetermined.

정적 평형이 물리적으로 가능하므로 이론적으로 `b ∈ col(A)`. 하지만 **수치적으로 잔차가 발생**할 수 있음:

```python
residual = np.linalg.norm(A @ tau_leg - b)
print(f"Solver residual: {residual}")
# > 0.1 이면 유의미한 중력 보상 오류
```

### 5.6 [MEDIUM] Pinocchio `model.gravity` 확인

```python
print(f"model.gravity: {self.model.gravity}")
# 기대값: linear=[0, 0, -9.81], angular=[0, 0, 0]
# 만약 다르면 (예: +9.81 또는 y 방향) G 전체가 잘못됨
```

### 5.7 [LOW] Contact Jacobian — 이전 분석에서 수정 완료

`placement = SE3.Identity()` (바퀴 중심) 사용 — §4.2 이전 분석에서 확인된 버그가 수정됨. ✅

---

## 6. 추천 진단 절차

아래 코드를 `control_loop()` 최상단에 **첫 tick에만** 실행하도록 추가:

```python
if self.count_tick == 0:
    # 1. Quaternion 확인
    R_base = pin.Quaternion(
        self.base_quat_curr[3], self.base_quat_curr[0],
        self.base_quat_curr[1], self.base_quat_curr[2]
    ).matrix()
    g_in_base = R_base.T @ np.array([0, 0, -9.81])
    print(f"[DIAG] Quaternion (xyzw): {self.base_quat_curr}")
    print(f"[DIAG] Gravity in base frame: {g_in_base}")

    # 2. Model gravity
    print(f"[DIAG] model.gravity: {self.model.gravity}")

    # 3. M 대칭성
    M_copy = self.data.M.copy()
    print(f"[DIAG] M asymmetry: {np.max(np.abs(M_copy - M_copy.T))}")

    # 4. G 비교 (computeAllTerms vs RNEA)
    G_from_data = self.data.g.copy()
    G_from_rnea = pin.rnea(self.model, self.data, self.q_curr,
                            np.zeros(self.nv), np.zeros(self.nv))
    print(f"[DIAG] data.g[6:]:  {G_from_data[6:]}")
    print(f"[DIAG] rnea(q,0,0): {G_from_rnea[6:]}")
    print(f"[DIAG] g match: {np.allclose(G_from_data, G_from_rnea)}")

    # 5. 직접 RNEA 중력 보상 vs Reduced dynamics
    # (이후 control_loop에서 tau_cmd 출력과 비교)
    print(f"[DIAG] RNEA gravity joint torques: {G_from_rnea[6:]}")

    # 6. q_curr 확인
    print(f"[DIAG] q_curr: {self.q_curr}")
    print(f"[DIAG] v_curr: {self.v_curr}")

    # 주의: rnea 호출이 data 내부 상태를 일부 변경하므로,
    # 진단 후 computeAllTerms를 다시 호출해야 안전
    pin.computeAllTerms(self.model, self.data, self.q_curr, self.v_curr)
```

### 판독 가이드

| 진단 항목 | 정상 | 이상 시 의미 |
|-----------|------|-------------|
| Quaternion | ≈ [0,0,0,1] 직립 시 | Convention 불일치 → G 부호 반전 |
| Gravity in base | ≈ [0,0,-9.81] | 방향 오류 → 전체 dynamics 틀림 |
| model.gravity | linear=[0,0,-9.81] | URDF 로드 시 gravity override 확인 |
| M asymmetry | < 1e-10 | M이 상삼각만 → M@qdd 오류 |
| g match | True | computeAllTerms 후 data 오염 |
| RNEA gravity signs | thigh: 부호 확인 | 부호 ↔ 관절 collapse 방향 비교 |

---

## 7. 간이 검증 — RNEA 직접 보상 테스트

중력 보상만 단독 테스트하려면, control_loop의 `tau_cmd`를 직접 RNEA 값으로 교체:

```python
# ===== Bypass: Direct RNEA gravity compensation (nullspace 미사용) =====
tau_grav = pin.rnea(self.model, self.data, self.q_curr,
                     np.zeros(self.nv), np.zeros(self.nv))
self.tau_cmd = tau_grav[6:]  # 관절 부분만
self.tau_cmd[self.wheel_L_joint_id] = 0.0  # 바퀴 제외
self.tau_cmd[self.wheel_R_joint_id] = 0.0
```

**결과 해석:**
- 로봇이 자세를 유지 → Pinocchio 모델 + G 정확. 문제는 nullspace solver.
- 로봇이 여전히 붕괴 → Pinocchio 모델 또는 입력(quaternion)에 오류.
- 붕괴 방향이 반대 → G 부호 반전 확인 (quaternion convention).

---

## 8. Wheel-Leg Coupling & Oscillation Analysis

(이전 분석 요약 — 중력 보상 정상 확인 후 적용)

### 8.1 [CRITICAL] Leg PD ↔ Wheel Controller Conflict

Leg PD(Kp=300)는 관절을 고정 목표로 유지, Wheel 제어기는 body pitch를 변경.
접촉 구속 하에서 pitch 변화 → 관절 각도 변화 → 두 제어기 충돌.

### 8.2 [HIGH] Kp/Kd 비율

| | Kp | Kd | Kd/√Kp | alpha |
|---|---|---|--------|-------|
| 현재 v2 | 300 | 10 | 0.577 | 1.0 (필터 없음) |
| 이전 v2 | 50 | 1 | 0.141 | 0.4 |
| v1 | 20 | 3 | 0.671 | 0.1 |

alpha=1.0은 LPF 완전 비활성화 — outer loop 노이즈가 직접 inner loop으로 전파.

### 8.3 Contact Model 제한

Normal constraint만 (2 rows). Rolling constraint 부재.
Nullspace 크기: 12D (nv=14 - rank 2).
Reduced dynamics solver: 12 equations, 6 unknowns → overdetermined.

### 8.4 Oscillation 메커니즘

```
Wheel controller: pitch tilt → wheel_tau
    → body pitch 변화 → 관절 각도 q_ref 이탈
    → Leg PD (Kp=300): 강한 복원 토크
    → 복원 토크가 wheel 의도와 충돌
    → 양쪽 모두 목표 미달 → overshoot → oscillation
```

---

## 9. 해결 우선순위

### Phase 1: 중력 보상 기본 정확성 확인

1. §6 진단 코드 실행 → quaternion, G 부호, M 대칭성 확인
2. §7 RNEA 직접 보상 테스트 → Pinocchio 모델 정상 여부 확인
3. Isaac Sim의 wheel joint dynamics(stiffness=10000, damping=100) 반영 여부 확인

### Phase 2: 중력 보상 정상 확인 후

4. Reduced dynamics solver 잔차 확인 (§5.5)
5. Coupled reference (q_ref를 theta_ref에 종속) 구현
6. 또는 Whole-body QP 도입
