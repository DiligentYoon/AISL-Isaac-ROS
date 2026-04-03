# 계획: `nsc_control_node_2.py` Contact Placement 버그 수정

> Date: 2026-04-03
> 근거: `research.md` Section 4.2, 5.2 분석 결과

---

## 목표

`compute_contact_jacobian()` 및 `compute_contact_jacobian_dot_times_v()`에서 사용하는 contact placement가 바퀴와 함께 회전하는 CRITICAL 버그를 수정한다.

---

## 버그 요약

```python
# 현재 (버그)
placement_L = pin.SE3(np.eye(3), np.array([0.0, 0.0, -self.wheel_radius]))
```

Pinocchio 내부: `oMframe = oMi[wheel_joint] * placement`
→ `oMi`에 바퀴 회전이 포함되어 있어, placement offset `[0,0,-r]`이 바퀴와 함께 회전.
→ 바퀴가 θ만큼 회전하면 접촉점이 `[r*sin(θ), 0, -r*cos(θ)]`로 이동 → 지면에서 벗어남.

---

## 수정 방안: 바퀴 중심 Jacobian 사용

placement를 `SE3.Identity()`로 변경하여 바퀴 중심(joint origin)의 Jacobian/가속도를 사용한다.

### 물리적 근거

| 구속 조건 | 수식 | 바퀴 중심으로 충분한 이유 |
|-----------|------|--------------------------|
| Normal (z) | `v_z(center) = 0` | 바퀴 중심 높이 불변 = 접촉 유지 |
| Lateral (y) | `v_y(center) = 0` | spin `ω_y × [0,0,-r]` = `[-ωr, 0, 0]` → y 기여 없음 |

---

## 변경 파일

`src/goat/goat/nodes/nsc_control_node_2.py` — 이 파일만 수정.

---

## 구현 체크리스트

### 1. `compute_contact_jacobian()` 수정 (lines 417-418)

```python
# Before
placement_L = pin.SE3(np.eye(3), np.array([0.0, 0.0, -self.wheel_radius]))
placement_R = pin.SE3(np.eye(3), np.array([0.0, 0.0, -self.wheel_radius]))

# After
placement_L = pin.SE3.Identity()
placement_R = pin.SE3.Identity()
```

### 2. `compute_contact_jacobian_dot_times_v()` 수정 (lines 459-460)

```python
# Before
placement_L = pin.SE3(np.eye(3), np.array([0.0, 0.0, -self.wheel_radius]))
placement_R = pin.SE3(np.eye(3), np.array([0.0, 0.0, -self.wheel_radius]))

# After
placement_L = pin.SE3.Identity()
placement_R = pin.SE3.Identity()
```

### 3. `compute_contact_jacobian_dot_times_v()`에서 불필요한 호출 제거 (line 454)

```python
# Before
pin.computeJointJacobiansTimeVariation(self.model, self.data, self.q_curr, self.v_curr)
pin.updateFramePlacements(self.model, self.data)

# After — 두 줄 제거
# (getFrameClassicalAcceleration은 forwardKinematics(q,v,a)만 필요)
```

---

## 변경하지 않는 항목

- Index mapping, Selection matrix, COM 계산, nullspace/projection 수학, torque assembly, wheel 부호 — 모두 정상 확인됨 (research.md Section 1-3, 6-8).
- `compute_contact_jacobian()`의 `computeJointJacobians` 호출 — `computeAllTerms` 이후 중복이지만, 함수의 독립성을 위해 유지.
