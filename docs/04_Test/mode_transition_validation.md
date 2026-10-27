# CUBIC Mapping–Navigation 모드 전환 최적화 및 안정성 검증

## 1. 검증 목적

CUBIC은 동일한 로봇에서 **SLAM 기반 Mapping 모드**와 **AMCL/Nav2 기반 Navigation 모드**를 전환하여 사용할 수 있도록 구성하였다.

초기 구현에서는 모드 변경 시 관련 ROS 2 노드와 서비스를 종료한 뒤 다시 실행하는 방식을 사용하였다. 이 방식은 구현은 단순했지만, 실제 운용 과정에서 다음과 같은 문제가 발생하였다.

- 모드 전환 시간이 길고 편차가 큼
- 경우에 따라 전환에 수십 초에서 약 1분까지 소요
- Lifecycle 노드 일부만 전환되는 partial state 발생 가능
- 전환 실패 시 사용자가 서비스를 직접 재시작해야 하는 경우 발생
- SLAM과 AMCL의 상태가 불분명해질 경우 `map → odom` 변환 관리가 어려움

따라서 단순한 전환 속도 향상보다 **반복 가능한 모드 전환과 실패 시 안전한 복구**를 주요 개선 목표로 설정하였다.

---

## 2. 시험 환경

| 구분 | 구성 |
|---|---|
| 메인 컴퓨터 | Raspberry Pi 5 |
| 운영체제 / 미들웨어 | Linux / ROS 2 Jazzy |
| 자율주행 | Nav2 |
| 위치 추정 | AMCL |
| Mapping | SLAM Toolbox |
| Odometry | Wheel Odometry + IMU + EKF |
| LiDAR | 전·후방 2개 LiDAR 병합 |
| ROS 2 통신 | Fast DDS, UDPv4 |
| 모드 관리 | 자체 `cubic_mode_manager` 노드 |
| 서비스 관리 | systemd |
| Navigation 관리 대상 | controller, smoother, planner, route, behavior, velocity smoother, collision monitor, BT navigator |
| Localization 관리 대상 | map_server, AMCL |

Navigation과 Localization 프로세스는 모드를 변경할 때마다 제거하지 않고 **상시 실행 상태로 유지**하고, ROS 2 Lifecycle 상태만 변경하는 Resident Lifecycle 구조를 적용하였다.

---

## 3. 초기 방식

초기 구현은 Mapping과 Navigation 전환 시 필요한 프로세스를 `systemctl stop/start` 방식으로 직접 종료·재실행하였다.

```text
Mapping
   ↓
기존 노드 종료
   ↓
Navigation 관련 프로세스 실행
   ↓
Lifecycle 초기화
   ↓
센서·TF·Localization 준비
   ↓
Navigation 시작
```

이 방식에서는 매번 ROS 2 프로세스 생성, DDS discovery, plugin 초기화, Lifecycle configure 등의 과정이 반복되었다.

실제 운용 과정에서는 전환 시간이 수십 초 이상 걸리는 경우가 있었으며, 상태가 꼬인 경우 약 1분 가까이 전환을 기다리거나 서비스를 재시작해야 하는 문제도 확인하였다.

---

## 4. 개선 과정

### 4.1 Resident Lifecycle 구조 적용

Navigation, Localization, SLAM 관련 프로세스를 상시 실행하고 실제 모드 변경 시에는 Lifecycle 상태만 변경하도록 수정하였다.

### Mapping 모드

```text
Navigation   → INACTIVE
Localization → INACTIVE
SLAM         → ACTIVE
```

### Navigation 모드

```text
SLAM         → INACTIVE
Localization → ACTIVE
Navigation   → ACTIVE
```

이를 통해 매 전환마다 프로세스를 새로 생성하는 비용을 제거하였다.

---

### 4.2 Persistent Map Saver 적용

기존에는 Navigation으로 전환할 때마다 `map_saver_cli` 프로세스를 새로 실행하였다.

초기 Map Save 시간은 약 **2~4초**, 평균 약 **3.2초** 수준이었다.

이후 `map_saver_server`를 systemd 서비스로 상시 실행하고 `/map_saver/save_map` 서비스를 직접 호출하도록 변경하였다.

개선 후 Map Save 시간은 일반적으로 약 **0.1~0.2초** 수준으로 감소하였다.

---

### 4.3 Lifecycle Manager 응답과 실제 상태 분리

Nav2 Lifecycle Manager에서 명령에 대해 실패 응답이 발생했음에도 실제 관리 노드들은 목표 상태에 도달해 있는 경우가 확인되었다.

따라서 다음과 같이 변경하였다.

```text
Lifecycle 명령 실행
       ↓
성공 응답
       └─ 정상 진행

실패/Timeout
       ↓
실제 Lifecycle 상태 확인
       ├─ 목표 상태 도달 → 정상으로 인정
       └─ 목표 상태 미도달 → 전환 실패 처리
```

즉 Lifecycle Manager의 응답만을 기준으로 판단하지 않고 **실제 각 노드의 Lifecycle 상태를 최종 기준으로 사용**하도록 수정하였다.

---

### 4.4 전체 Lifecycle 상태 검증

초기에는 `controller_server` 하나의 상태를 Navigation 전체 상태로 간주하였다.

하지만 테스트 중 일부 노드만 ACTIVE이고 나머지는 INACTIVE인 partial state가 발생할 수 있음을 확인하였다.

이에 Navigation의 관리 대상 전체와 Localization의 두 노드를 개별 확인하도록 변경하였다.

예를 들어 Mapping 모드의 정상 상태는 다음 조건을 모두 만족해야 한다.

```text
controller_server      inactive
smoother_server        inactive
planner_server         inactive
route_server           inactive
behavior_server        inactive
velocity_smoother      inactive
collision_monitor      inactive
bt_navigator           inactive

map_server             inactive
amcl                   inactive

slam_toolbox           active
```

---

### 4.5 Rollback 기능 추가

모드 전환 도중 일부 단계에서 실패하면 그대로 중단하지 않고 **전환 전 모드로 복구**하도록 수정하였다.

예를 들어 Mapping → Navigation 도중 Navigation 활성화 직전에 강제로 오류를 발생시키는 시험을 수행하였다.

```text
Mapping
  ↓
SLAM 비활성화
  ↓
Localization 활성화
  ↓
Navigation 활성화 직전 강제 실패
  ↓
Rollback 실행
  ↓
Navigation INACTIVE
Localization INACTIVE
SLAM ACTIVE
  ↓
Mapping 복구
```

강제 실패 후 실제 서비스 응답에서도 다음과 같이 복구가 확인되었다.

```text
TEST: forced Navigation activation failure;
rolled back to MAPPING
```

복구 후 Lifecycle 상태를 다시 확인한 결과 Mapping 모드의 정상 상태로 되돌아왔으며, 이후 Navigation 전환도 다시 정상적으로 수행할 수 있었다.

---

## 5. Cold Boot 검증

전원 인가 및 서비스 재시작 이후에도 Mode Manager가 현재 Lifecycle 상태를 확인하고 정상 모드로 수렴하는지 확인하였다.

Cold Boot 후 최종 상태는 다음과 같았다.

```text
Navigation 8 nodes    → INACTIVE
Localization 2 nodes → INACTIVE
SLAM                  → ACTIVE
Current Mode          → MAPPING
```

초기화 과정에서 Navigation 서비스가 아직 준비되지 않은 경우 Mode Manager가 즉시 실패하지 않고 재시도하며, 서비스 준비 후 자동으로 Lifecycle 초기화를 수행하였다.

---

## 6. 반복 전환 시험

최종 구조에서 Mapping ↔ Navigation 왕복 전환을 연속 5회 수행하였다.

### Mapping → Navigation

| 회차 | 전환 시간 |
|---:|---:|
| 1 | 6.26 s |
| 2 | 3.99 s |
| 3 | 3.83 s |
| 4 | 4.26 s |
| 5 | 3.51 s |
| **평균** | **4.37 s** |

첫 전환을 제외한 이후 4회의 평균은 약 **3.90초**였다.

### Navigation → Mapping

| 회차 | 전환 시간 |
|---:|---:|
| 1 | 1.85 s |
| 2 | 4.26 s |
| 3 | 4.22 s |
| 4 | 2.58 s |
| 5 | 4.11 s |
| **평균** | **3.40 s** |

총 **10회의 모드 전환이 모두 성공**하였다.

---

## 7. 추가 최적화 시험

Lifecycle 전체 상태 검증 시간을 줄이기 위해 여러 노드의 `GetState` 요청을 병렬 처리하는 방법도 시험하였다.

해당 방식에서는 Mapping → Navigation 전환 시간이 약 **3.05초**까지 감소하였다.

그러나 이후 Navigation → Mapping 전환 중 Lifecycle Manager 응답이 정지하면서 일부 노드만 ACTIVE/INACTIVE 상태로 남는 문제가 발생하였다.

따라서 약간의 속도 향상보다 반복 동작의 안정성이 중요하다고 판단하여 병렬 검증 방식은 사용하지 않고, **순차 검증 방식으로 복귀하였다.**

---

## 8. 최종 결과

| 항목 | 초기 | 최종 |
|---|---|---|
| 모드 전환 방식 | 프로세스 종료·재실행 | Resident Lifecycle |
| Map 저장 | 매번 CLI 실행 | Persistent Map Saver |
| 상태 판단 | 대표 노드 위주 | 전체 Lifecycle 검증 |
| 실패 처리 | 전환 중단 / 수동 재시작 | 자동 Rollback |
| Cold Boot | 상태 의존 | 자동 상태 확인 및 복구 |
| 전환 시간 | 수십 초~최악 약 1분 | 일반적으로 약 3~4초 |
| 반복 전환 | 상태 꼬임 가능 | 5회 왕복 / 10회 전환 성공 |

최종적으로 CUBIC의 Mapping–Navigation 전환은 단순히 시간을 단축하는 것뿐 아니라, **현재 시스템 상태를 검증하고 실패 시 기존 모드로 복구할 수 있는 구조**로 개선하였다.

실제 운용에서는 Mapping → Navigation 약 4초, Navigation → Mapping 약 3초 수준의 전환 시간을 보였으며, 초기 방식과 비교해 전환 대기 시간을 크게 줄이면서도 시스템 안정성을 높일 수 있었다.