# 04_Test

CUBIC C1 개발 과정에서는 모터 신호, 통신 지연, 위치추정 오차, 센서 정합 등
하드웨어와 소프트웨어가 복합적으로 얽힌 문제가 발생하였다.

각 문제는 실제 신호 측정, 로그 분석, 반복 주행 및 비정상 조건 시험을 통해 원인을 확인하고,
해결 이후 별도의 Test 문서에서 결과를 검증하였다.

---

## 주요 문제 해결

### Motor FG 분석 및 검출 개선

모터 데이터시트에 FG 출력 방식과 PPR 정보가 부족하여
오실로스코프로 출력 전압과 주파수를 직접 측정하였다.

이후 PC817 절연 과정에서 FG 파형이 왜곡되는 문제가 발생하여,
RP2350 ADC와 Software Hysteresis를 적용하였다.

최종 검출 오차:

- 100% Duty: **0.32%**
- 80% Duty: **0.12%**

상세 내용은 [`FG_Test.md`](./FG_Test.md)에 정리하였다.

---

### Communication / Control 분리

초기에는 ROS 2 통신과 모터 제어가 동시에 수행되면서
명령 응답과 제어 주기가 영향을 받는 문제가 있었다.

RP2350의 Dual-core 구조를 이용하여

- Core 0: micro-ROS, FG 측정, 명령 처리
- Core 1: 속도 제어, 가속 제한, PWM 출력

으로 역할을 분리하여 통신과 실시간 제어를 독립적으로 처리하도록 개선하였다.

---

### Motor Boot Safety

MCU 초기화 중 PWM 입력이 불확정 상태가 되면서
모터가 순간적으로 움직이는 현상이 발생하였다.

초기 제어 출력을 정지 상태로 설정하고,
PWM 입력에 10 kΩ Pull-down을 추가하여 비정상 구동 가능성을 줄였다.

---

### Odometry / Navigation 오차

정지 시 관성 및 Slip으로 인해 실제 이동거리와 휠 오도메트리 사이에 오차가 발생하였다.

PWM 감속 제어와 LiDAR 기반 위치 보정을 사용하였으며,
IMU 가속도 보정은 지도 정합을 악화시키는 경우가 있어 최종 구성에서 제외하였다.

실제 반복 주행 결과는 [`Drive_Test.md`](./Drive_Test.md)에 기록하였다.

---

### LiDAR / TF 정합

전·후방 LiDAR의 장착 방향과 TF 정의가 맞지 않아
초기 SLAM에서 스캔이 반전되거나 어긋나는 문제가 발생하였다.

CAD 기준으로 센서 위치와 방향을 다시 반영하고,
로봇 자체 구조물을 감지하는 영역에는 Scan Masking을 적용하였다.

---

### ROS 2 Startup

micro-ROS, LiDAR, IMU, Foxglove 및 Navigation 서비스의 시작 시점 차이로
일부 노드가 정상적으로 연결되지 않는 문제가 있었다.

각 기능을 독립 서비스로 분리하고,
ROS 2 Topic 생성 여부를 기준으로 시스템 준비 상태를 확인하도록 구성하였다.

---

## Validation Summary

| 검증 항목 | 결과 |
|---|---|
| FG 검출 / 100% Duty | 1.240 kHz → 1.236 kHz, **0.32% 오차** |
| FG 검출 / 80% Duty | 830 Hz → 829 Hz, **0.12% 오차** |
| 직선 반복 주행 | 1.25 m × 10회 → 평균 약 1.30 m |
| Emergency Stop | **10/10 정상 정지** |
| Communication Loss | **5/5 정상 정지** |
| Nav2 장애물 대응 | 장애물 진입 시 재계획, 제거 후 경로 갱신 |
| Localization 교란 | 목표점 도달 실패, 충돌 없이 정지/주행 |
| Ethernet Interface | 고정 IP 기반 SSH 및 File 접근 확인 |
| XT60 Power | Battery Voltage 출력 확인 |

---

## Test Documents

| 문서 | 내용 |
|---|---|
| [`FG_Test.md`](./FG_Test.md) | FG 역설계 및 ADC + Hysteresis 검증 |
| [`Drive_Test.md`](./Drive_Test.md) | 직선 반복 주행 및 목표점 보정 |
| [`Nav2_Test.md`](./Nav2_Test.md) | 장애물 재계획 및 Localization 교란 |
| [`Safety_Test.md`](./Safety_Test.md) | E-stop 및 통신 단절 Fail-Safe |
| [`System_Integration_Test.md`](./System_Integration_Test.md) | Bring-up, ROS 2, Ethernet, XT60 |

시험 이미지와 캡처 자료는 [`pic/`](./pic/)에 정리하였다.

---

## Conclusion

CUBIC C1의 주요 문제는 단일 부품보다
Mechanical, Electrical, Firmware, ROS 2 및 Navigation 계층이 서로 영향을 주면서 발생하였다.

개발 과정에서는 실제 신호와 로그를 기반으로 원인을 확인하고,
효과가 확인되지 않은 방식은 제거하며 최종 구조를 결정하였다.

또한 해결 이후에는 반복 주행, 신호 비교, 안전 시험 및 비정상 조건 시험을 통해
실제 시스템에서 동작을 다시 검증하였다.