# CUBIC Pin Map & Electrical Interface

CUBIC의 전장 시스템은 **상위 제어기(SBC), 하위 제어기(MCU/PCU), 센서, 모터 구동부**를 기능별로 분리하여 구성하였다.

각 장치에는 독립적인 통신 채널을 할당하고, 모터와 같이 전기적 노이즈가 큰 계통에는 신호 절연과 전원 보호 회로를 적용하였다. 또한 유지보수성을 높이기 위해 UART 채널과 LiDAR Device Name을 고정적으로 할당하였다.

---

# 1. 시스템 인터페이스 구성

| 구분          | 장치              | 연결 방식          | 역할                  |
| ----------- | --------------- | -------------- | ------------------- |
| 상위 제어기      | Raspberry Pi 5  | -              | ROS 2 및 자율주행 시스템 실행 |
| 모터 제어기      | Pico 2          | UART           | 모터 제어 및 FG 속도 측정    |
| 전원 제어기      | Pico 2          | UART           | 배터리 전압·전류·SOC 감시    |
| IMU         | BNO055          | UART           | 자세 및 각속도 측정         |
| Front LiDAR | RPLIDAR C1      | USB            | 전방 거리 측정            |
| Rear LiDAR  | RPLIDAR C1      | USB            | 후방 거리 측정            |
| Motor ×2    | BLDC Geared Motor | PWM / DIR / FG | 좌·우 독립 구동           |

---

# 2. Raspberry Pi 5 Interface

Raspberry Pi 5는 CUBIC의 상위 제어기로 사용하며, 하위 제어기와 IMU에 각각 독립적인 UART 채널을 할당하였다.

| 인터페이스 | 연결 장치             | Raspberry Pi GPIO | Device                   | 용도           |
| ----- | ----------------- | ----------------- | ------------------------ | ------------ |
| UART0 | Motor Controller  | GPIO14 / GPIO15   | `/dev/ttyAMA0`           | 모터 제어 MCU 통신 |
| UART1 | Debug / Expansion | -                 | `/dev/ttyAMA1`           | 디버깅 및 향후 확장  |
| UART2 | BNO055            | GPIO4 / GPIO5     | `/dev/ttyAMA2`           | IMU 데이터 수신   |
| UART3 | Power Controller  | GPIO8 / GPIO9     | `/dev/ttyAMA3`           | 전원 관리 PCU 통신 |
| USB   | Front LiDAR       | USB               | `/dev/cubic_lidar_front` | 전방 LaserScan |
| USB   | Rear LiDAR        | USB               | `/dev/cubic_lidar_rear`  | 후방 LaserScan |

UART는 기능별로 채널을 고정하여 각 통신 계통을 독립적으로 관리하도록 구성하였다.

LiDAR는 `/dev/ttyUSB0`, `/dev/ttyUSB1` 대신 고정 Device Name을 사용한다.

```text
/dev/cubic_lidar_front
/dev/cubic_lidar_rear
```

이를 통해 USB 인식 순서가 변경되더라도 전·후방 LiDAR가 뒤바뀌는 것을 방지하였다.

---

# 3. Motor Controller Unit — Pico 2

Motor Controller Unit은 좌·우 모터의 속도 및 방향 제어와 FG 피드백 측정을 담당한다.

## Pin Map

| 기능           |          GPIO | 방향     | 설명                  |
| ------------ | ------------: | ------ | ------------------- |
| Motor 0 FG   | GPIO26 / ADC0 | Input  | Motor 0 회전속도 피드백    |
| Motor 0 DIR  |        GPIO21 | Output | Motor 0 회전방향        |
| Motor 0 PWM  |        GPIO19 | Output | Motor 0 출력 제어       |
| Motor 1 FG   | GPIO27 / ADC1 | Input  | Motor 1 회전속도 피드백    |
| Motor 1 DIR  |        GPIO20 | Output | Motor 1 회전방향        |
| Motor 1 PWM  |        GPIO18 | Output | Motor 1 출력 제어       |
| Host UART TX |         GPIO1 | Output | Raspberry Pi 통신     |
| Host UART RX |         GPIO2 | Input  | Raspberry Pi 통신     |
| PCU UART TX  |         GPIO6 | Output | Power Controller 통신 |
| PCU UART RX  |         GPIO7 | Input  | Power Controller 통신 |

## PWM / DIR

좌·우 모터에는 각각 독립적인 PWM과 DIR 신호를 할당하였다.

PWM 입력에는 **Pull-down 저항**을 적용하여 MCU 부팅 또는 리셋 과정에서 GPIO 상태가 확정되기 전에도 모터 드라이버 입력이 LOW 상태를 유지하도록 하였다.

이를 통해 초기화 과정에서 발생할 수 있는 의도치 않은 순간 구동을 억제한다.

## FG Feedback

각 모터의 FG 신호는 다음 경로로 처리한다.

```text
Motor FG
   ↓
 PC817
   ↓
  ADC
   ↓
Hysteresis Detection
   ↓
Wheel Speed
```

실제 FG 출력에서 전압 변동과 파형 왜곡이 확인되어 단순 디지털 입력 대신 **ADC 기반 히스테리시스 검출**을 적용하였다.

PC817은 모터측 FG 신호와 MCU 입력을 전기적으로 분리하며, ADC에서는 상·하한 임계값을 이용해 안정적으로 펄스를 판정한다.

> FG 처리 및 Motor Control 알고리즘은
> [`하위제어 상세설명`](../03_Software/하위제어%20상세설명.md)을 참고한다.

---

# 4. Motor Protection Circuit

모터의 감속 및 정지 과정에서 발생할 수 있는 역기전력과 전원 이상으로부터 구동부를 보호하기 위해 별도의 보호 요소를 적용하였다.

각 모터 출력단에는 **MBR10100 Schottky Diode**를 적용하여 역기전력에 의한 전압 스파이크를 억제한다.

| 보호 요소              | 목적                     |
| ------------------ | ---------------------- |
| PWM Pull-down      | 부팅 및 리셋 시 비정상 모터 입력 억제 |
| MBR10100           | 역기전력에 의한 전압 스파이크 억제    |
| Motor Fuse 10 A ×2 | 좌·우 모터 계통 과전류 및 단락 보호  |
| PC817              | FG 신호 절연 및 MCU 보호      |
| Emergency Stop     | 모터 전원의 하드웨어 차단         |

---

# 5. Power Controller Unit — Pico 2

Power Controller Unit은 배터리의 전압과 시스템 전류를 측정하고 SOC 및 전원 상태를 상위 시스템으로 전달한다.

## Pin Map

| 기능           |          GPIO | 방향           | 설명                  |
| ------------ | ------------: | ------------ | ------------------- |
| INA226 SCL   |         GPIO2 | I/O          | 전압 센서 I²C Clock     |
| INA226 SDA   |         GPIO3 | I/O          | 전압 센서 I²C Data      |
| INA226 ALERT |         GPIO6 | Input        | INA226 Alert 신호     |
| WCS1800 AOUT | GPIO26 / ADC0 | Input        | 시스템 전류 아날로그 측정      |
| WCS1800 DOUT |        GPIO22 | Input        | 전류 센서 디지털 출력        |
| Buzzer       |        GPIO15 | Output / PWM | 상태 및 경고음 출력         |
| Host UART TX |         GPIO0 | Output       | Raspberry Pi 통신     |
| Host UART RX |         GPIO1 | Input        | Raspberry Pi 통신     |
| MCU UART TX  |         GPIO4 | Output       | Motor Controller 통신 |
| MCU UART RX  |         GPIO5 | Input        | Motor Controller 통신 |

## Voltage / Current Monitoring

* **INA226**: 배터리 전압 측정 및 전원 상태 감시
* **WCS1800**: 시스템 전류 측정

WCS1800의 아날로그 출력은 Pico 2 ADC에서 직접 측정하며, 측정된 값을 기반으로 SOC와 전원 상태를 계산한다.

> PCU의 SOC 계산 및 제어 로직은
> [`하위제어 상세설명`](../03_Software/하위제어%20상세설명.md)을 참고한다.

---

# 6. Sensor Interface

## BNO055 IMU

BNO055는 Raspberry Pi 5의 UART2에 연결한다.

```text
GPIO4 / GPIO5
/dev/ttyAMA2
```

IMU 통신을 MCU 및 PCU와 분리하여 독립적으로 관리한다.

## Dual LiDAR

| 위치    | Device                   | ROS 2 Topic   |
| ----- | ------------------------ | ------------- |
| Front | `/dev/cubic_lidar_front` | `/scan_front` |
| Rear  | `/dev/cubic_lidar_rear`  | `/scan_rear`  |

전·후방 RPLIDAR C1의 Scan은 상위 ROS 2 시스템에서 하나의 `/scan`으로 병합하여 SLAM 및 Navigation에 사용한다.

센서의 위치와 방향은 URDF/Xacro의 TF를 기준으로 정의한다.

> LiDAR Filtering, Scan Merge 및 TF 구성은
> [`ROS 2 Integration`](../03_Software/ROS%202%20Integration.md)을 참고한다.

---

# 7. Ground & Signal Reference

직접 전압 레벨을 사용하는 UART, PWM, DIR 신호는 송수신 장치 간 동일한 기준 전위가 필요하므로 **Common GND**를 사용한다.

반면 모터 구동부에서 발생하는 FG 신호는 노이즈 영향을 줄이기 위해 PC817을 이용해 제어 계통과 분리하였다.

| 신호 구분            | 구성                      |
| ---------------- | ----------------------- |
| UART / PWM / DIR | Common GND              |
| Motor FG         | PC817 Optical Isolation |
| Motor Power      | 별도 전원 경로 및 보호 회로        |

---

# 8. Power Distribution & Fuse Protection

CUBIC C1은 7S Li-ion 배터리를 주 전원으로 사용하며, 시스템 전체와 좌·우 모터, 외부 모듈 계통에 각각 보호 퓨즈를 적용하였다.

```text
7S Li-ion Battery
        │
     35 A Main Fuse
        │
        ├──── 10 A Fuse ── Motor 0 Power
        │
        ├──── 10 A Fuse ── Motor 1 Power
        │
        └────  5 A Fuse ── External Module Power
```

| 보호 계통        |       정격 | 역할                  |
| ------------ | -------: | ------------------- |
| Main Power   | **35 A** | 전체 주 전원 및 배선 보호     |
| Motor 0      | **10 A** | Motor 0 전원 계통 보호    |
| Motor 1      | **10 A** | Motor 1 전원 계통 보호    |
| Module Power |  **5 A** | XT60 외부 모듈 전원 계통 보호 |

좌·우 모터는 각각 독립적인 DC-DC 전원 계통과 퓨즈를 사용하며, 외부 모듈 전원도 별도 5 A 보호 계통으로 분리하였다.

이를 통해 특정 부하의 과전류나 단락이 다른 전원 계통으로 확산되는 것을 줄였다.

Main Switch와 Emergency Stop을 통해 소프트웨어 상태와 관계없이 모터 전원을 직접 차단할 수 있도록 구성하였다.

---

# 9. Design Summary

CUBIC의 전장 구조는 **통신 독립성, 유지보수성, 노이즈 대응 및 다중 안전 계층**을 중심으로 구성하였다.

| 설계 요소   | 적용                                     |
| ------- | -------------------------------------- |
| 통신 분리   | MCU · PCU · IMU 독립 UART                |
| 장치 식별   | LiDAR Device Name 고정                   |
| 부팅 안전   | PWM Pull-down                          |
| FG 보호   | PC817 + ADC Hysteresis                 |
| 역기전력 보호 | MBR10100                               |
| 과전류 보호  | Main 35 A / Motor 10 A ×2 / Module 5 A |
| 비상 정지   | Hardware Emergency Stop                |

---

## 관련 자료

* 전체 회로 및 결선도
  → `CUBIC_schem.png`

* ROS 2 통신 및 Topic 구조
  → [`ROS 2 Integration`](../03_Software/ROS%202%20Integration.md)

* Motor / Power Controller 상세 제어 구조
  → [`하위제어 상세설명`](../03_Software/하위제어%20상세설명.md)

* 사용 부품 및 주요 하드웨어
  → [`하드웨어 사용 목록표`](../../03_Datasheet/하드웨어%20사용%20목록표.md)

* 개발 과정의 문제 분석 및 해결
  → [`문제해결 및 트러블슈팅`](../../04_Test/문제해결%20및%20트러블슈팅.md)
