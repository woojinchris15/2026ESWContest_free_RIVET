# CUBIC Firmware

`firmware`는 CUBIC C1의 **RP2350 기반 하위 제어기 펌웨어**를 포함한다.

모터 제어와 전원 관리를 Raspberry Pi의 ROS 2 상위 제어와 분리하여 수행하며, 각 제어기는 micro-ROS를 통해 ROS 2 시스템과 통신한다.

---

## 📦 구성

| Firmware        | Target                       | 역할                                   |
| --------------- | ---------------------------- | ------------------------------------ |
| `motor control` | Raspberry Pi Pico 2 / RP2350 | 모터 구동, FG 측정, 속도 제어, `/cmd_vel` 처리   |
| `power control` | Raspberry Pi Pico 2 / RP2350 | 배터리 전압·전류 측정, SOC 계산, Fault 및 시스템 알림 |

### ⚙️Motor Controller

`motor control/mcu.cpp`

* Differential Drive 제어
* FG 기반 Wheel Speed 측정
* PWM / Direction 출력
* 가속 제한 및 속도 제어
* micro-ROS 통신
* Command Watchdog

### 🔋Power Controller

`power control/pcu.cpp`

* 배터리 전압 및 전류 측정
* SOC 계산
* Fault 감시
* 시스템 상태 알림
* micro-ROS 통신

> Core 분리, FG 처리, Motor Control 및 Power Management의 상세 구조는
> [`하위제어 상세설명`](../docs/02_Architect/03_Software/하위제어%20상세설명.md)을 참고한다.

---

# ⚠️ 빌드 전 필수 요구사항

CUBIC Firmware를 빌드하려면 다음 항목이 별도로 준비되어 있어야 한다.

## 1. Pico SDK

컴퓨터에 `pico-sdk`가 설치되어 있어야 하며, 설치 후 **시스템 환경 변수에 SDK 경로를 등록해야 한다.**

```bash
export PICO_SDK_PATH=/path/to/pico-sdk
```

확인:

```bash
echo $PICO_SDK_PATH
```

정상적인 SDK 경로가 출력되어야 한다.

---

## 2. micro-ROS Jazzy Library

CUBIC MCU / PCU는 micro-ROS를 사용하므로 **ROS 2 Jazzy 기준 `libmicroros`가 필요하다.**

```text
libmicroros/
```

해당 폴더에는 Pico/RP2350 환경에서 사용할 수 있도록 준비된 micro-ROS Library와 Header가 포함되어 있어야 한다.

> `libmicroros`는 본 저장소에 포함되어 있지 않으며 별도로 준비해야 한다.

---

## 3. Pico UART Transport

UART 기반 micro-ROS 통신을 위해 다음 파일이 필요하다.

```text
pico_uart_transport.c
pico_uart_transport.h
```

따라서 실제 빌드 환경에는 다음 파일이 추가되어 있어야 한다.

```text
motor control/
├─ CMakeLists.txt
├─ mcu.cpp
├─ pico_sdk_import.cmake
├─ pico_uart_transport.c
├─ pico_uart_transport.h
└─ libmicroros/
```

Power Controller도 동일한 구성이 필요하다.

> `libmicroros` 및 `pico_uart_transport.c/.h`가 없으면 micro-ROS 관련 Library 또는 Transport를 찾지 못해 빌드되지 않는다.

---

# 🔨 Build

## Motor Controller

```bash
cd "firmware/motor control"
mkdir build
cd build

cmake ..
make -j
```

## Power Controller

```bash
cd "firmware/power control"
mkdir build
cd build

cmake ..
make -j
```

빌드가 정상적으로 완료되면 Raspberry Pi Pico 2에 업로드할 `.uf2` 파일이 생성된다.

---

## 🔌 ROS 2 연결

MCU와 PCU는 UART 기반 micro-ROS Transport를 사용하며, Raspberry Pi에서는 각각의 UART 인터페이스에 대해 micro-ROS Agent가 실행되어야 한다.

관련 systemd 구성은 다음 문서를 참고한다.

[`../systemd/README.md`](../systemd/README.md)

---

## 📚 관련 문서

* 하위 제어 구조 및 알고리즘
  → [`하위제어 상세설명`](../docs/02_Architect/03_Software/하위제어%20상세설명.md)

* ROS 2 Topic 및 통신 구조
  → [`ROS 2 Integration`](../docs/02_Architect/03_Software/ROS%202%20Integration.md)

* 개발 과정의 문제 분석 및 해결
  → [`문제해결 및 트러블슈팅`](../docs/04_Test/문제해결%20및%20트러블슈팅.md)

* UART, 센서 및 제어기 핀맵 / 결선 정보
  → [`Electrical Architecture`](../docs/02_Architect/02_Eletrical/README.md)
---

> ROS 2 기반 상위 제어 소프트웨어는 [`../cubic_ws/`](../cubic_ws/)를 참고한다.
