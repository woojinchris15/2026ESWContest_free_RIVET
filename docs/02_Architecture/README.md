# CUBIC Architecture

`02_Architecture` 디렉터리는 CUBIC C1의 **기구, 전기, 소프트웨어 시스템 설계**를 세 영역으로 구분하여 정리한 문서 모음이다.

각 문서는 실제 제작 구조와 구현 내용을 기준으로 작성되어 있으며, 세 영역을 함께 보면 CUBIC의 전체 시스템 구성을 이해할 수 있다.

---

## 📚 Architecture Index

| 구분                                                            | 내용                             |
| ------------------------------------------------------------- | ------------------------------ |
| [`01_Mechanical Integration`](./01_Mechanical%20Integration/) | 프레임 · 구동부 · 모듈 장착 · 기구 통합      |
| [`02_Electrical`](./02_Electrical/)                           | 전원 · 제어기 · 센서 · 결선 · Pin Map   |
| [`03_Software`](./03_Software/)                               | ROS 2 · 통신 · 하위 제어 · 시스템 동작 구조 |

---

## 01. Mechanical Integration

CUBIC의 기구 구조와 실제 부품 장착 방식을 설명한다.

주요 내용:

* 알루미늄 프로파일 기반 프레임
* 구동부 및 캐스터 배치
* 상단 모듈 장착 구조
* 외부 RJ45 / XT60 인터페이스
* 센서 및 주요 부품의 기구 통합

→ [`Mechanical Integration`](./01_Mechanical%20Integration/Mechanical%20Integration.md)

---

## 02. Electrical

CUBIC의 전원 및 전기적 연결 구조를 설명한다.

주요 내용:

* 배터리 및 전원 분배
* Motor / Power Controller 연결
* 센서 및 SBC 연결
* UART / GPIO Pin Map
* 안전 회로 및 전체 결선

→ [`Electrical Architecture`](./02_Electrical/README.md)

---

## 03. Software

CUBIC의 ROS 2 상위 제어와 RP2350 기반 하위 제어 구조를 설명한다.

주요 문서:

* [`ROS 2 Integration`](./03_Software/ROS%202%20Integration.md)
* [`하위제어 상세설명`](./03_Software/하위제어%20상세설명.md)

포함 내용:

* ROS 2 Node / Topic 구성
* 센서 및 제어 데이터 흐름
* Mapping / Navigation 구성
* Motor / Power Controller 역할
* RP2350 Core 분리 및 하위 제어 로직

---

## 🧩 Architecture Overview

```text
Mechanical
   │
   ├── Frame / Drive / Module Mount
   │
Electrical
   │
   ├── Power / Controller / Sensor / Safety
   │
Software
   │
   └── ROS 2 / micro-ROS / Control / Navigation
```

CUBIC은 기구, 전기, 소프트웨어를 독립적으로 설계하면서도 각 계층의 인터페이스를 명확히 연결하는 구조를 사용한다.

---

## 🔗 관련 자료

| 구분                 | 경로                                                     |
| ------------------ | ------------------------------------------------------ |
| 프로젝트 개요            | [`../01_Project Overview/`](../01_Project%20Overview/) |
| 하드웨어 목록            | [`../03_Datasheet/`](../03_Datasheet/)                 |
| 문제 해결 및 검증         | [`../04_Test/`](../04_Test/)                           |
| 3D Model           | [`../06_3Dmodel/`](../06_3Dmodel/)                     |
| ROS 2 Workspace    | [`../../cubic_ws/`](../../cubic_ws/)                   |
| MCU / PCU Firmware | [`../../firmware/`](../../firmware/)                   |

---

> 각 영역의 세부 구현과 설계 근거는 해당 하위 문서를 참고한다.
