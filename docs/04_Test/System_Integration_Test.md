# System Integration & External Interface Test

CUBIC C1은 SBC, MCU, 센서 및 외부 인터페이스를 하나의 시스템으로 통합하여,
전원 인가 이후 상태 확인부터 외부 모듈 연결까지 일관된 운용이 가능하도록 구성하였다.

본 시험에서는 다음 항목을 확인하였다.

1. System Bring-up
2. ROS 2 Topic / Foxglove Monitoring
3. SBC Backup Power
4. External Ethernet Interface
5. External Power Interface

---

# 1. System Bring-up Test

## 1.1 시험 과정

CUBIC C1의 Main Power를 인가하면 다음 순서로 시스템이 준비된다.

```text
Main Power ON
      │
      ▼
SBC / Controller Boot
      │
      ▼
PCU Boot Buzzer
      │
      ▼
LiDAR / Sensor Start
      │
      ▼
ROS 2 Nodes / Topics Start
      │
      ▼
Foxglove Monitoring
````

전원 인가 후 PCU Buzzer와 LiDAR 동작을 통해 하위 장치의 초기화를 확인하였으며,
ROS 2 Node가 실행되면 각 센서 및 제어 Topic이 Monitoring 환경에 표시되었다.

## 1.2 반복 운용 결과

SLAM, Navigation 및 모터 제어 시험을 수행할 때마다 동일한 Bring-up 과정을 반복적으로 확인하였다.

* Controller Boot
* LiDAR / Sensor 동작
* ROS 2 Topic 생성
* Foxglove 연결 및 상태 확인

별도의 Boot Time 측정은 수행하지 않았으므로 정량적인 부팅 시간은 기록하지 않았다.

---

# 2. ROS 2 Integration & Monitoring

CUBIC의 하위제어기와 센서는 ROS 2 Topic을 통해 상위 시스템에서 상태를 확인할 수 있도록 구성하였다.

![ROS 2 Topics](pic/topic.png)

**Fig. 1. 정상 운용 상태에서 확인한 ROS 2 Topic 및 시스템 상태**

Foxglove에서는 다음 정보를 확인하였다.

* LiDAR Scan
* Robot TF
* Odometry
* Navigation 상태
* Power / Controller Topic
* Sensor 동작 상태

이를 통해 각 MCU에 별도의 Serial Debugger를 연결하지 않고도
외부에서 로봇 전체의 상태를 확인할 수 있음을 검증하였다.

---

# 3. SBC Backup Power Test

CUBIC C1의 Raspberry Pi 5와 SSD에는 X1208 UPS HAT을 적용하여,
메인 전원 차단 시 SBC가 즉시 비정상 종료되지 않도록 구성하였다.

## 3.1 시험 목적

메인 전원 차단 상황에서 X1208이 Raspberry Pi 5와 SSD의 전원을 유지하고,
시스템이 정상적으로 종료될 수 있는지 확인하였다.

## 3.2 시험 방법

정상 부팅 및 동작 상태에서 메인 전원을 차단하는 시험을 총 5회 반복하였다.

각 시험에서 Raspberry Pi 5가 전원 단절 직후 비정상 종료되지 않고,
UPS 전원으로 전환된 뒤 정상 종료되는지 확인하였다.

## 3.3 시험 결과

| 시험 항목 | 결과 |
|---|---:|
| 반복 횟수 | 5회 |
| 정상 종료 | **5회** |
| 비정상 종료 | 0회 |

총 5회의 전원 차단 시험에서 **5회 모두 정상 종료되는 것을 확인하였다.**

이를 통해 X1208 UPS가 메인 전원 차단 시
Raspberry Pi 5와 SSD의 전원 보호 계층으로 정상 동작함을 확인하였다.

> 본 시험에서는 UPS 전환시간과 Backup 유지시간을 별도 계측하지 않았으므로
> 해당 값은 정량적으로 평가하지 않았다.

---

# 4. External Ethernet Interface Test

## 4.1 시험 목적

상단 RJ45 Port를 통해 외부 연구용 컴퓨터 또는 Module이
CUBIC의 Raspberry Pi와 직접 통신할 수 있는지 확인하였다.

## 4.2 구성

Raspberry Pi의 Ethernet Interface를 외부 RJ45 Port로 연결하고,
고정 IP 기반의 직접 연결이 가능하도록 설정하였다.

노트북을 RJ45 Port에 연결하여 실제 통신을 시험하였다.

## 4.3 결과

외부 노트북에서 다음 기능을 확인하였다.

* 고정 IP Network 연결
* SSH 접속
* Raspberry Pi 내부 File 접근
* Remote Command 실행

따라서 별도의 Wi-Fi AP가 없어도
유선 Ethernet을 통해 CUBIC의 SBC에 직접 접근할 수 있음을 확인하였다.

> RJ45 Port는 DDS 전용 인터페이스로 제한하지 않고,
> SSH, ROS 2 및 기타 Ethernet 기반 통신을 사용할 수 있는 범용 Network Interface로 구성하였다.

---

# 5. External Power Interface Test

## 5.1 시험 목적

상단 Module용 XT60 Port가 CUBIC의 전원계통과 정상적으로 연결되어 있는지 확인하였다.

## 5.2 시험 방법

Multimeter로 XT60 Port의 출력 전압을 측정하였다.

## 5.3 결과

XT60 Port에서 Battery Voltage가 정상적으로 출력되는 것을 확인하였다.

이를 통해 외부 Module이 별도의 Battery를 추가하지 않고
CUBIC의 전원계통을 사용할 수 있음을 확인하였다.

본 시험에서는 최대 부하전류에 대한 Load Test는 수행하지 않았으며,
전압 출력 및 전기적 연결 여부만 확인하였다.

---

# 6. Module Interface

CUBIC의 상부 Module Interface는 기구 장착부와 함께
Network 및 Power Interface를 제공한다.

```text
External Module
      │
      ├── Mechanical Mounting
      │
      ├── RJ45
      │     └── Ethernet / SSH / ROS 2
      │
      └── XT60
            └── Battery Power
```

이를 통해 외부 Module을 추가할 때
로봇 내부 Wiring이나 SBC 연결을 직접 수정하지 않고
상부에서 전원과 통신을 연결할 수 있도록 구성하였다.

---

# 7. 시험 결과 요약

| 검증 항목 | 결과 |
|---|---|
| System Bring-up | 반복 운용에서 Controller / Sensor 초기화 확인 |
| ROS 2 Integration | 주요 Topic 생성 및 데이터 확인 |
| Foxglove | Robot / Sensor / Navigation 상태 Monitoring |
| X1208 UPS | 메인 전원 차단 시험 5/5 정상 종료 |
| RJ45 Ethernet | Laptop 직접 연결 및 SSH 접속 성공 |
| File Access | Ethernet을 통한 SBC File 접근 확인 |
| XT60 Power | Multimeter로 Battery Voltage 출력 확인 |

---

# 8. 결론

CUBIC C1의 반복 운용을 통해
전원 인가 이후 Controller, Sensor 및 ROS 2 Software가 하나의 시스템으로 동작하고,
Foxglove에서 전체 상태를 확인할 수 있음을 검증하였다.

또한 X1208 UPS를 통해 SBC 전원 백업 계층을 구성하고,
상단 RJ45와 XT60 Interface를 이용해 외부 Module에 Network와 Power를 제공할 수 있음을 확인하였다.

따라서 CUBIC의 상부 확장 구조는 단순한 기구 장착부가 아니라

* Mechanical Mounting
* Network Communication
* Power Supply

를 함께 제공하는 실제 Module Interface로 구성되어 있다.
