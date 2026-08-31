# CUBIC Pin Map & Electrical Interface

CUBIC의 전장 시스템은 **상위 제어기(SBC), 하위 제어기(MCU/PCU), 센서, 모터 구동부**를 기능별로 분리하여 구성하였다.

각 장치에는 독립적인 통신 채널을 할당하였으며, 모터와 같이 전기적 노이즈가 큰 장치는 보호 회로와 신호 절연을 적용하였다. 또한 유지보수 시 장치를 쉽게 식별할 수 있도록 UART 및 LiDAR 인터페이스를 고정적으로 할당하였다.

---

## 1. 시스템 인터페이스 구성

| 구분          | 장치              | 연결 방식          | 역할                  |
| ----------- | --------------- | -------------- | ------------------- |
| 상위 제어기      | Raspberry Pi 5  | -              | ROS 2 및 자율주행 시스템 실행 |
| 모터 제어기      | Pico 2      | UART           | 모터 제어 및 FG 속도 측정    |
| 전원 제어기      | Pico 2      | UART           | 배터리 전압·전류·SOC 감시    |
| IMU         | BNO055          | UART           | 자세 및 각속도 측정         |
| Front LiDAR | RPLIDAR C1      | USB            | 전방 거리 측정            |
| Rear LiDAR  | RPLIDAR C1      | USB            | 후방 거리 측정            |
| Motor ×2    | BLDC Geared Motor | PWM / DIR / FG | 좌·우 독립 구동           |

---

# 2. Raspberry Pi 5 Interface

Raspberry Pi 5는 CUBIC의 상위 제어기로 사용하며, 하위 제어기 및 센서에 각각 독립적인 UART 채널을 할당하였다.

| 인터페이스 | 연결 장치             | Raspberry Pi GPIO | Device                   | 용도           |
| ----- | ----------------- | ----------------- | ------------------------ | ------------ |
| UART0 | Motor Controller  | GPIO14 / GPIO15   | `/dev/ttyAMA0`           | 모터 제어 MCU 통신 |
| UART1 | Debug / Expansion | -                 | `/dev/ttyAMA1`           | 디버깅 및 향후 확장  |
| UART2 | BNO055            | GPIO4 / GPIO5     | `/dev/ttyAMA2`           | IMU 데이터 수신   |
| UART3 | Power Controller  | GPIO8 / GPIO9     | `/dev/ttyAMA3`           | 전원 관리 PCU 통신 |
| USB   | Front LiDAR       | USB               | `/dev/cubic_lidar_front` | 전방 LaserScan |
| USB   | Rear LiDAR        | USB               | `/dev/cubic_lidar_rear`  | 후방 LaserScan |

UART 장치는 기능별로 채널을 고정하여 하나의 장치 또는 통신 계통에 문제가 발생하더라도 다른 장치의 통신에 직접적인 영향을 주지 않도록 구성하였다.

LiDAR는 일반적인 `/dev/ttyUSB0`, `/dev/ttyUSB1` 대신 각각 `/dev/cubic_lidar_front`, `/dev/cubic_lidar_rear`의 고정 장치명을 사용한다. 이를 통해 USB 인식 순서가 변경되더라도 ROS 2 설정에서 전·후방 LiDAR가 뒤바뀌는 것을 방지하였다.

---

# 3. Motor Controller Unit — Pico 2

Motor Controller Unit은 좌·우 DC 모터의 속도 및 방향 제어와 FG 피드백 측정을 담당한다.

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

---

## PWM / DIR Control

각 모터에는 별도의 PWM 및 DIR 신호를 할당하여 좌·우 모터의 속도와 회전 방향을 독립적으로 제어할 수 있도록 구성하였다.

PWM 신호에는 **풀다운 저항**을 적용하였다.

MCU의 전원이 인가되거나 리셋되는 과정에서는 GPIO 출력 상태가 완전히 초기화되기 전까지 핀이 일시적으로 부유 상태가 될 수 있다. 이때 모터 드라이버가 잘못된 PWM 신호를 인식하여 모터가 순간적으로 동작하는 것을 방지하기 위해, 하드웨어적으로 PWM 입력의 기본 상태가 LOW가 되도록 구성하였다.

따라서 소프트웨어가 아직 실행되지 않은 상태에서도 모터의 기본 상태는 정지 상태를 유지한다.

---

## FG Feedback

각 모터에서 출력되는 FG 신호는 **PC817 포토커플러**를 거쳐 Pico 2의 ADC 입력으로 전달된다.

실제 모터의 FG 출력을 측정한 결과 신호의 전압 변동과 파형 왜곡이 존재하여 단순 GPIO 디지털 입력만으로 안정적인 펄스 검출이 어려웠다.

이에 따라 다음과 같이 구성하였다.

**Motor FG → PC817 → ADC → Hysteresis Detection**

PC817을 사용하여 모터측 신호와 MCU측 신호의 직접적인 전기적 영향을 줄였으며, MCU에서는 ADC로 파형을 측정한 뒤 상·하한 임계값을 이용한 히스테리시스 방식으로 펄스를 판정한다.

이를 통해 노이즈가 포함된 FG 신호에서도 안정적으로 모터 회전 속도를 계산할 수 있도록 하였다.

---

# 4. Motor Protection Circuit

모터는 인덕턴스 성분을 가진 부하이기 때문에 감속 또는 정지 과정에서 역기전력에 의한 순간적인 전압 상승이 발생할 수 있다.

이를 억제하기 위해 각 모터 출력단에 **MBR10100 Schottky Diode**를 적용하였다.

MBR10100은 모터에서 발생하는 역기전력에 의한 전압 스파이크를 클램핑하여 모터 드라이버와 전원 회로에 가해지는 전기적 스트레스를 줄이는 역할을 한다.


### 주요 보호 요소

| 보호 요소            | 목적                        |
| ---------------- | ------------------------- |
| PWM Pull-down    | 부팅 및 리셋 시 의도치 않은 모터 동작 억제 |
| MBR10100         | 모터 역기전력에 의한 전압 스파이크 억제    |
| Motor Fuse       | 과전류 발생 시 모터 계통 보호         |
| PC817            | FG 신호의 전기적 분리 및 MCU 보호    |
| Emergency Stop   | 비상 상황에서 모터 전원 하드웨어 차단     |


---

# 5. Power Controller Unit — Pico 2

Power Controller Unit은 배터리의 전압과 전류를 측정하고 SOC를 계산하며, 시스템의 전원 상태를 Raspberry Pi에 전달한다.

## Pin Map

| 기능           |          GPIO | 방향           | 설명                  |
| ------------ | ------------: | ------------ | ------------------- |
| INA226 SCL   |         GPIO2 | I/O          | 전압·전류 센서 I²C Clock  |
| INA226 SDA   |         GPIO3 | I/O          | 전압·전류 센서 I²C Data   |
| INA226 ALERT |         GPIO6 | Input        | INA226 Alert 신호     |
| WCS1800 AOUT | GPIO26 / ADC0 | Input        | 배터리 전류 아날로그 측정      |
| WCS1800 DOUT |        GPIO22 | Input        | 전류센서 디지털 출력         |
| Buzzer       |        GPIO15 | Output / PWM | 상태 및 경고음 출력         |
| Host UART TX |         GPIO0 | Output       | Raspberry Pi 통신     |
| Host UART RX |         GPIO1 | Input        | Raspberry Pi 통신     |
| MCU UART TX  |         GPIO4 | Output       | Motor Controller 통신 |
| MCU UART RX  |         GPIO5 | Input        | Motor Controller 통신 |

---

## Voltage / Current Monitoring

배터리 상태 측정에는 전압 및 전류 특성에 따라 서로 다른 센서를 사용하였다.

**INA226**은 배터리 전압 및 전원 상태 측정에 사용하며, I²C를 통해 PCU와 통신한다.

**WCS1800**은 시스템 전류 측정에 사용하며 아날로그 출력을 Pico 2 ADC에서 직접 측정한다.

측정된 값을 이용하여 배터리의 잔여 용량과 SOC를 계산하고 ROS 2의 전원 상태 정보로 전달한다.

---

# 6. BNO055 IMU

BNO055는 로봇의 자세 및 각속도 측정을 담당한다.

센서 데이터는 Raspberry Pi 5의 전용 UART 채널인 `/dev/ttyAMA2`를 통해 전달된다.

IMU에 독립적인 UART 채널을 할당하여 모터 및 전원 제어기의 통신과 분리하였으며, ROS 2에서는 독립된 IMU 데이터로 처리한다.

---

# 7. Dual LiDAR Interface

CUBIC C1은 전·후방에 각각 하나의 RPLIDAR C1을 배치하여 로봇 주변의 거리 정보를 측정한다.

| 위치    | Device                   | ROS 2 Topic   |
| ----- | ------------------------ | ------------- |
| Front | `/dev/cubic_lidar_front` | `/scan_front` |
| Rear  | `/dev/cubic_lidar_rear`  | `/scan_rear`  |

두 LiDAR의 스캔 데이터는 상위 시스템에서 하나의 `/scan` 데이터로 병합하여 SLAM 및 Navigation에 사용한다.

센서의 물리적인 위치와 방향은 URDF/Xacro의 TF를 통해 정의하며, 후방 LiDAR는 전방 LiDAR와 반대 방향으로 설치되어 있다.

---

# 8. Ground & Signal Reference

UART, PWM, DIR과 같이 직접 전압 레벨을 이용하는 신호는 송수신 장치가 동일한 기준 전위를 가져야 하므로 제어 계통의 GND를 공통 기준으로 사용한다.

반면 모터 FG와 같이 모터 구동부에서 발생하여 노이즈의 영향을 받기 쉬운 신호는 PC817 포토커플러를 이용하여 제어부와 분리하였다.

즉 CUBIC의 신호 계통은 다음 원칙을 따른다.

* 일반 제어 및 UART 신호 → **Common GND**
* 모터 노이즈 영향이 큰 FG 신호 → **Optical Isolation**
* 모터 전력 계통 → **별도의 보호 및 전원 경로 적용**

---

# 9. Power Distribution

CUBIC C1의 주 전원은 7S Li-ion 배터리를 사용한다.

모터 구동부와 제어부는 기능에 따라 전원 경로를 분리하며, 두 모터에는 각각 독립적인 DC-DC 전원 계통과 보호 퓨즈를 적용하였다.

이를 통해 한쪽 모터에서 발생하는 급격한 부하 변화가 다른 구동부 및 제어 시스템에 미치는 영향을 줄였다.

전원 계통에는 메인 스위치와 비상 정지 스위치를 배치하여 소프트웨어의 상태와 관계없이 모터 전원을 직접 차단할 수 있도록 구성하였다.

---

# 10. Design Summary

CUBIC C1의 핀 및 전장 구성은 단순한 장치 연결보다 **안전성, 통신 독립성, 노이즈 대응 및 유지보수성**을 중심으로 설계하였다.

주요 설계 원칙은 다음과 같다.

| 설계 요소  | 적용 방법                      |
| ------ | -------------------------- |
| 통신 독립성 | MCU, PCU, IMU에 개별 UART 할당  |
| 유지보수성  | LiDAR 장치명 고정               |
| 부팅 안전성 | PWM Pull-down 적용           |
| 모터 보호  | MBR10100 적용      |
| 신호 안정성 | FG 신호 PC817 절연             |
| 노이즈 대응 | ADC + Hysteresis FG 검출     |
| 전원 보호  | 모터별 전원 경로 및 퓨즈 분리          |
| 비상 안전  | Hardware Emergency Stop 적용 |

이와 같은 구성을 통해 CUBIC C1은 각 모듈을 독립적으로 점검·교체할 수 있으며, 모터 구동부에서 발생하는 전기적 노이즈와 이상 상태가 상위 제어 시스템에 미치는 영향을 최소화하도록 설계하였다.
