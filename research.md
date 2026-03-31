# Pinocchio `data.com[i]` 프레임 규약 분석 — `P_rel.z` 음수 원인 조사

> 작성 일자: 2026-03-31
> 대상 파일: `src/goat/goat/nodes/nsc_control_node.py`
> Pinocchio 버전: 3.9.0 (설치 경로: `/home/grape4314/miniconda3/envs/env_ros`)

---

## 1. 문제 요약

`compute_com_and_theta()` 안의 로그에서 `P_rel.z` 값이 음수로 출력된다.

```python
P_rel = com_body - com_wheel
print(f"... P_rel | ... : {P_rel} | ...")
```

직립 상태에서는 **몸통 COM이 바퀴 COM보다 위에** 있어야 하므로 `P_rel.z > 0`이어야 맞다.
그러나 실제로는 `P_rel.z < 0`이 나온다 — 이는 **Pinocchio API의 좌표계 혼용**에 기인한 버그다.

---

## 2. 근본 원인 — `pin.centerOfMass`의 `data.com[i>0]`는 로컬 프레임

### 2.1 공식 헤더 문서 (설치된 소스 기준)

**파일:** `/home/grape4314/miniconda3/envs/env_ros/include/pinocchio/algorithm/center-of-mass.hpp`, line 56-59

```cpp
/// \brief Computes the center of mass position of a given model ...
///        The result is accessible through data.com[0] for the full body com and data.com[i] for
///        the subtree supported by joint i (expressed in the joint i frame).
```

> `data.com[i]` (i > 0)은 **"joint i frame"**, 즉 관절 i의 **로컬 프레임**으로 표현된다.

### 2.2 구현 확인 (`center-of-mass.hxx`)

**Forward Step (line 159):**
```cpp
data.com[i].noalias() = mass * lever;
//  lever = model.inertias[i].lever() → 링크 i의 관성 중심, joint i 로컬 프레임
```

**Backward Step (line 180):**
```cpp
data.com[parent] += (liMi.rotation() * data.com[i] + data.mass[i] * liMi.translation());
//  liMi = data.liMi[i] : 부모→자식의 "로컬-상대" 변환 (oMi 아님!)
```

`oMi`(월드 프레임 절대 변환)가 아닌 `liMi`(부모-자식 상대 변환)를 사용하므로
**모든 `data.com[i]`(i > 0) 값은 각자의 관절 i 로컬 프레임에 남는다.**

`data.com[0]`만 최종적으로 월드 프레임:
```cpp
data.com[0] /= data.mass[0];  // line 200 → 월드 프레임
```

### 2.3 프레임 규약 요약표

| 함수 | `data.com[0]` | `data.com[i>0]` |
|---|---|---|
| `pin.centerOfMass(model, data, q [,v], True)` | **월드 프레임** | **관절 i 로컬 프레임** ← 버그 원인 |
| `pin.jacobianCenterOfMass(model, data, q, True)` | **월드 프레임** | **월드 프레임** ← 올바른 API |

`jacobianCenterOfMass` 헤더 명세(line 202-208):
```cpp
/// data.com[i] gives the center of mass of the subtree supported by joint i
/// (expressed in the world frame).
```

---

## 3. 버그가 발생하는 메커니즘

```python
# nsc_control_node.py:254
pin.centerOfMass(self.model, self.data, q, v, compute_subtree_coms=True)

com_total  = self.data.com[0]   # ✓ 전체 COM : 월드 프레임
com_wheel_L = self.data.com[5]  # ✗ wheel_L_Joint 로컬 프레임 (≈ 관절 원점 근방 소량 오프셋)
com_wheel_R = self.data.com[9]  # ✗ wheel_R_Joint 로컬 프레임 (≈ 관절 원점 근방 소량 오프셋)
```

이후 `com_body` 계산:
```python
com_body = (M_total * com_total - m_wheel_L * com_wheel_L - m_wheel_R * com_wheel_R) / M_body
```
월드 프레임 벡터에서 **로컬 프레임 벡터**를 빼는 것으로 물리적으로 무의미한 결과가 된다.
`vcom_wheel_L/R`도 동일하게 로컬 프레임이므로 `V_rel`(`theta_dot` 계산)도 오염된다.

---

## 4. 왜 `P_rel.z`가 음수인가 — 수치 분석

### URDF에서 직립 자세의 예상 월드 프레임 위치 (관절각=0 기준)

```
base frame origin              z =  0.000 m  (월드 원점)
  hip_L_Joint                  z = -0.153 m  (URDF: z=-152.597mm)
    thigh_L_Joint               z = -0.153 m  (부모로부터 z=0)
      knee_L_Joint              z = -0.358 m  (부모로부터 z=-205mm)
        wheel_L_Joint           z = -0.558 m  (부모로부터 z=-200mm)
          wheel_L_Link COM      z ≈ -0.558 m  (URDF 로컬 오프셋 z=-0.002mm, 무시 가능)
base_Link COM (URDF inertial)   z = -0.072 m
```

### 올바른 경우 (월드 프레임)

| 변수 | z 성분 (근사) |
|---|---|
| `com_total.z` (월드) | ≈ −0.15 m |
| `com_wheel.z` (월드, 정답) | ≈ −0.558 m |
| `com_body.z` (정답) | ≈ −0.10 m |
| **`P_rel.z` (정답)** | **≈ +0.46 m (양수)** |

### 버그 발생 시 (로컬 프레임 혼용)

`data.com[5]`(wheel_L_Joint 로컬)에서 z 값은 URDF `<inertial><origin xyz="0 7.264E-03 -0.002E-03"/>`
→ z ≈ **−0.000002 m ≈ 0**

| 변수 | z 성분 (버그 상태) |
|---|---|
| `com_total.z` (월드) | ≈ −0.15 m |
| `com_wheel_L.z` (로컬 프레임) | ≈ −0.000002 m ≈ **0** |
| `com_body.z` (혼합 연산 결과) | ≈ −0.166 m |
| `com_wheel.z` (평균, 로컬 기준) | ≈ **0** m |
| **`P_rel.z` (버그 결과)** | **≈ −0.166 m (음수)** |

결론: `com_wheel`이 로컬 프레임 기준 거의 0에 가까운 오프셋 값으로 잘못 읽히고,
`com_body`도 혼합 연산으로 오염되어 `P_rel.z < 0`이 나온다.

---

## 5. 수정 방안

### 방법 A — `data.oMi`로 로컬→월드 변환 (centerOfMass 결과 재활용)

> **Q. `oMi` 객체가 정확히 무엇인가 ? 프레임 변환 메소드인가 ?**
>
> 메소드가 아니라 **데이터 객체(SE3)**다. FK 호출(`forwardKinematics` 또는 `computeAllTerms`) 후 `data.oMi[i]`에 채워진다.
>
> 이름 분해: `o`(origin = 월드) + `M`(placement = SE3 변환) + `i`(joint index)
>
> 내용: 관절 i 의 joint frame이 월드 기준으로 어디에 어떤 방향으로 있는지를 담은 **SE3 객체**.
> SE3는 회전(R, 3×3)과 평행이동(t, 3×1)으로 구성된다:
> - `data.oMi[i].rotation`    → R (3×3 회전 행렬)
> - `data.oMi[i].translation` → t (3×1 위치 벡터, 월드 기준)

> **Q. `oMi.act(p)`와 `oMi.rotation @ v`가 다른점은 무엇인가 ?**
>
> SE3 Rotation Transformation: `p_world = R * p_local + t`
>
> - **`oMi.act(p)`** → R × p + t 전체 적용 (Rotation + Translation)
>   - 사용 대상: **점(position)** — 원점 기준 절대 위치이므로 t를 더해야 월드 좌표가 됨
>   - 예) `com_wheel_L` (바퀴 COM의 위치)
>
> - **`oMi.rotation @ v`** → R × v 만 적용 (Only Rotation)
>   - 사용 대상: **방향벡터(velocity/direction)** — 속도는 기준 원점과 무관하게 방향만 돌리면 됨
>   - 예) `vcom_wheel_L` (바퀴 COM의 속도)
>

```python
pin.centerOfMass(self.model, self.data, q, v, compute_subtree_coms=True)

# 위치: oMi.act(p_local) = R * p_local + t
com_wheel_L = self.data.oMi[self.wheel_L_joint_pin_id].act(
    self.data.com[self.wheel_L_joint_pin_id]
)
com_wheel_R = self.data.oMi[self.wheel_R_joint_pin_id].act(
    self.data.com[self.wheel_R_joint_pin_id]
)

# 속도: 평행이동은 속도 변환에 없으므로 rotation만 적용
R_L = self.data.oMi[self.wheel_L_joint_pin_id].rotation
vcom_wheel_L = R_L @ self.data.vcom[self.wheel_L_joint_pin_id]

R_R = self.data.oMi[self.wheel_R_joint_pin_id].rotation
vcom_wheel_R = R_R @ self.data.vcom[self.wheel_R_joint_pin_id]
```

---

## 6. 추가 고려사항

### 6.1 `computeAllTerms`와의 상호작용

현재 코드는 `pin.computeAllTerms(model, data, q, v)` 후 다시 `pin.centerOfMass`를 호출한다.
`computeAllTerms`는 내부적으로 FK(`data.oMi`)를 계산하므로, 방법 C 사용 시
`centerOfMass` 재호출 없이 `data.oMi`를 바로 쓸 수 있다.

### 6.2 `data.liMi` vs `data.oMi`

| 변수 | 의미 | 프레임 |
|---|---|---|
| `data.liMi[i]` | 부모 관절 → 관절 i 의 상대 SE3 | 부모 상대 |
| `data.oMi[i]` | 관절 i 의 월드 절대 SE3 | 월드 프레임 |

subtree COM을 월드 프레임으로 올릴 때 반드시 `data.oMi`를 사용해야 한다.

> **Q. Pinocchio Library로부터 얻는 Subtree COM과 월드 프레임 COM 간의 차이는 무엇인가?**
>
> "subtree COM"은 **무엇의 COM인가** (값의 의미)이고,
> "월드 프레임"은 **그 값을 어떤 기준으로 표현하는가** (좌표계)다. 서로 독립된 개념이다.
>
> **subtree of joint i**: 관절 i를 루트로 하는 하위 링크 집합.
> - `data.com[5]` subtree = wheel_L_Joint 루트 = {wheel_L_Link} 하나뿐 (leaf)
> - `data.com[1]` subtree = root_joint 루트 = 로봇 전체 (base + 모든 다리)
>
> **같은 subtree COM이라도 좌표계에 따라 숫자가 달라진다:**
>
> ```
> 바퀴 COM이 공간에서 실제로 [x=0.000, y=0.119, z=-0.558 m] 에 있다고 하면:
>
>   월드 프레임으로 표현  →  [ 0.000,  0.119, -0.558 ]   (베이스/지면 기준 절대 위치)
>   wheel_L_Joint 로컬    →  [ 0.000,  0.007, -0.000 ]   (바퀴 관절 원점 기준 오프셋)
> ```
>
> 같은 공간상의 한 점이지만 각 객체를 정의하는 기준 좌표계가 다름.
> `pin.centerOfMass`는 로컬 기준값을 주고, `data.oMi[i].act()`로 월드 기준으로 변환해야 한다.

### 6.3 `theta` 계산 좌표계

```python
theta = math.atan2(P_rel[0], P_rel[2])
```

이 수식은 `P_rel`이 **월드 프레임**일 때 x-z 평면의 피치각(y축 회전)을 구한다.
- 직립: `P_rel ≈ [0, 0, +L]` → `theta ≈ 0` ✓
- 앞으로 기울면: `P_rel.x > 0` → `theta > 0` ✓
수정 후에야 이 해석이 의미를 갖는다.

### 6.4 바퀴 phi 계산 부호 규약

```python
phi = (self.joint_q_curr[self.wheel_L_joint_id] - self.joint_q_curr[self.wheel_R_joint_id]) / 2.0
```

URDF: `wheel_L_Joint axis="0 1 0"`, `wheel_R_Joint axis="0 -1 0"`.
전진 시 두 바퀴가 같은 부호의 각도 변화를 갖도록 설계되어 있으며,
제어 출력도 `tau_R = -tau_L`을 적용 중이므로 부호 규약 자체는 일관적이다.

### 6.5 `q[0:3] = zeros` 이슈

base position을 `[0,0,0]`으로 고정 → 월드 원점 = base frame 원점.
이 상태에서 수정 후 `P_rel`은 올바른 월드 프레임 상대 벡터를 반환하며,
pendulum length L ≈ 0.4~0.46 m 수준이 나올 것으로 예상된다.
