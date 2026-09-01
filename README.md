# CUBIC

> **로봇 개발에서 반복되는 이동 플랫폼 설계를 줄이기 위한 ROS 2 기반 모듈형 연구·개발용 모바일 로봇 플랫폼**

CUBIC C1은 새로운 로봇 프로젝트를 시작할 때마다 차체, 구동부, 전원부, 하위 제어기, 센서 및 ROS 2 환경을 처음부터 구축해야 하는 문제를 줄이기 위해 개발한 **범용 모바일 로봇 플랫폼**이다.

특정 목적을 수행하는 완성형 로봇이 아니라, 사용자가 필요한 센서·컴퓨팅 장치·상부 모듈과 알고리즘을 추가하여 새로운 로봇 시스템을 빠르게 구성할 수 있는 **재사용 가능한 연구·개발 기반**을 목표로 한다.

![CUBIC](docs/05_Pic/demo.jpg)

---

## 🔗 바로가기

| 바로가기                                                             | 내용                                    |
| ---------------------------------------------------------------- | ------------------------------------- |
| **[프로젝트 개요](./docs/01_Project%20Overview/)**                     | 개발 배경 · 목표 · 시스템 구성 · 역할 및 일정         |
| **[기구 설계](./docs/02_Architecture/01_Mechanical%20Integration/)** | 프레임 · 모듈 장착 · 기구 통합                   |
| **[전기 설계](./docs/02_Architecture/02_Electrical/)**               | 전원 · 제어기 · 센서 · 결선                    |
| **[소프트웨어 구조](./docs/02_Architecture/03_Software/)**              | ROS 2 · 하위 제어 · 통신 구조                 |
| **[ROS 2 Workspace](./cubic_ws/)**                               | Bring-up · SLAM · Localization · Nav2 |
| **[MCU Firmware](./firmware/)**                                  | Motor Controller · Power Controller   |
| **[System Services](./systemd/)**                                | 부팅 및 ROS 2 systemd 서비스                |
| **[Test & Troubleshooting](./docs/04_Test/)**                    | 실측 검증 · 문제 분석 · 개선 과정                 |
| **[하드웨어 목록](./docs/03_Datasheet/)**                              | 주요 부품 및 하드웨어 구성                       |
| **[3D Model](./docs/06_3Dmodel/)**                               | 전체 조립 및 개별 STEP 모델                    |

> ROS 2 패키지, MCU Firmware 및 systemd 서비스의 세부 파일 구성과 사용 방법은 각 디렉터리의 README에서 확인할 수 있다.

---

## 📌 Project Overview

로봇 프로젝트에서는 핵심 기능을 개발하기 전에 **이동 플랫폼 자체를 구축하는 데 많은 시간과 비용이 소모된다.**

직접 제작할 경우 프레임, 모터, 배터리, 전원 회로, 제어기, 배선, 센서 장착 구조와 ROS 2 환경까지 반복적으로 구축해야 한다. 반대로 상용 모바일 플랫폼은 높은 비용이나 제한적인 기구·전기적 확장성 때문에 새로운 장치와 목적에 맞게 수정하기 어려운 경우가 있다.

CUBIC은 이러한 문제에서 출발하였다.

> **이동과 기본 자율주행에 필요한 기반 시스템을 미리 구축하고, 새로운 프로젝트에서는 그 위에 필요한 기능만 개발한다.**

이를 위해 CUBIC C1에는 다음 요소를 하나의 플랫폼으로 통합하였다.

* 알루미늄 프로파일 기반 모듈형 프레임
* 24 V Differential Drive 구동계
* 독립적인 Motor / Power Controller
* Raspberry Pi 기반 ROS 2 상위 제어 환경
* Dual LiDAR 기반 360° 환경 인식
* IMU · Wheel Odometry · EKF 기반 상태 추정
* SLAM · Localization · Navigation
* Emergency Stop · Watchdog 기반 안전 구조
* 외부 모듈용 Ethernet / Power Interface

상세 설계와 개발 과정은 상단 바로가기 또는 [`docs/`](./docs/) 디렉터리를 참고한다.

---

## ⚙️ 주요 사양

| 항목              | 사양 / 운용 기준                          |
| --------------- | ----------------------------------- |
| **상위 제어기**      | Raspberry Pi 5                      |
| **하위 제어기**      | RP2350 기반 Motor / Power Controller  |
| **ROS 2**       | ROS 2 Jazzy                         |
| **구동 방식**       | 24 V Differential Drive             |
| **최고 주행 속도**    | 수동 주행 기준 최대 약 **1 m/s**             |
| **자율주행 속도**     | 안정적인 운용을 위해 최대 약 **0.5 m/s**로 제한    |
| **환경 인식**       | RPLIDAR C1 ×2, Front / Rear Scan 병합 |
| **LiDAR 측정 범위** | 제조사 기준 최대 **12 m** (70% 반사율 물체)     |
| **권장 적재 하중**    | 평탄면 기준 약 **30 kg**                  |
| **배터리**         | 7S Li-ion · 24 V Class · 20 Ah      |
| **외부 인터페이스**    | Gigabit Ethernet · XT60 Power       |

> LiDAR 측정 범위는 RPLIDAR C1 제조사 사양이며, 실제 인식 거리는 대상의 반사율과 주변 환경에 따라 달라질 수 있다.

---

## 🧪 테스트 및 검증

CUBIC C1은 단순 기능 구현에 그치지 않고, 실제 신호 측정과 반복 주행, 비정상 조건 시험을 통해 주요 하드웨어·펌웨어·ROS 2 기능을 검증하였다.

### Validation Summary

| 검증 항목                    | 시험 조건                   | 결과                                       |
| ------------------------ | ----------------------- | ---------------------------------------- |
| **Motor FG 검출**          | 100% Duty FG 주파수 비교     | 1.240 kHz → 1.236 kHz, **0.32% 오차**      |
| **Motor FG 검출**          | 80% Duty FG 주파수 비교      | 830 Hz → 829 Hz, **0.12% 오차**            |
| **직선 반복 주행**             | 목표 1.25 m × 10회         | 평균 약 **1.30 m**, 목표 대비 약 **+5 cm / +4%** |
| **Emergency Stop**       | 주행 중 E-Stop 동작 10회      | **10/10 정상 정지**                          |
| **Communication Loss**   | 제어 명령 통신 단절 5회          | **5/5 Watchdog 기반 정상 정지**                |
| **Nav2 장애물 대응**          | 주행 경로에 장애물 진입 및 제거      | 장애물 감지 시 재계획, 제거 후 경로 갱신 확인              |
| **Localization 교란**      | 위치 추정이 불안정한 조건에서 목표점 주행 | 목표점 도달 실패 시 충돌 없이 정지 또는 제한적 주행           |
| **Ethernet Interface**   | 외부 RJ45 인터페이스 연결        | 고정 IP 기반 SSH 및 File 접근 확인                |
| **XT60 Power Interface** | 외부 전원 포트 출력 확인          | Battery Voltage 출력 확인                    |

직선 반복 주행 시험에서는 동일한 조건으로 **1.25 m를 10회 주행**한 결과 평균 약 **1.30 m**를 기록하여 약 **+5 cm의 초과 주행 경향**을 확인하였다.

이 값은 이상적인 센서 또는 FG 자체의 위치 측정 정확도가 아니라, **타이어 변형, 구동륜 Slip, 정지 관성 및 실제 하위 제어계를 모두 포함한 로봇 전체 시스템의 반복 주행 결과**이다.

Nav2 반복 주행에서는 동일한 시작 위치와 목표점을 지정했을 때 대체로 유사한 Global Path가 생성되었으며, 목표점 접근 후 위치 또는 방향 오차가 남은 경우 추가 이동·회전·방향 재조정을 통해 목표 Pose를 보정하는 동작을 확인하였다.

---

## 🔧 문제 해결 및 검증 과정

개발 과정에서는 모터 신호, 통신 지연, 위치 추정, 센서 정합 및 시스템 시작 순서 등 하드웨어와 소프트웨어가 함께 영향을 미치는 문제를 확인하였다.

주요 개선 항목은 다음과 같다.

* Motor FG 출력 직접 측정 및 PPR 분석
* PC817 절연 이후 FG 파형 왜곡에 대한 ADC + Software Hysteresis 적용
* RP2350 Dual-Core 기반 Communication / Control 처리 분리
* MCU Boot 과정의 비정상 모터 구동 방지
* 감속 제어 및 LiDAR 기반 위치 보정을 통한 Odometry 오차 대응
* Dual LiDAR TF 재정의 및 Scan Masking
* ROS 2 기능별 systemd 서비스 분리 및 Topic 기반 Ready Check

각 문제의 **발생 원인, 측정 과정, 적용한 해결 방법과 최종 검증 결과**는 [`docs/04_Test/`](./docs/04_Test/)에 정리하였다.

| 문서                                                                        | 내용                                       |
| ------------------------------------------------------------------------- | ---------------------------------------- |
| [`FG_Test.md`](./docs/04_Test/FG_Test.md)                                 | FG 역설계 및 ADC + Hysteresis 검증             |
| [`Drive_Test.md`](./docs/04_Test/Drive_Test.md)                           | 직선 반복 주행 및 Navigation 동작 검증              |
| [`Nav2_Test.md`](./docs/04_Test/Nav2_Test.md)                             | 장애물 재계획 및 Localization 교란 시험             |
| [`Safety_Test.md`](./docs/04_Test/Safety_Test.md)                         | Emergency Stop 및 통신 단절 Fail-Safe 검증      |
| [`System_Integration_Test.md`](./docs/04_Test/System_Integration_Test.md) | Bring-up · ROS 2 · Ethernet · XT60 통합 검증 |

---

## 📄 License

본 프로젝트의 사용 및 배포 조건은 [`LICENSE`](./LICENSE)를 참고한다.
