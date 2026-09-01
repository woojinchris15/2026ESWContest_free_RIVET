# CUBIC systemd Services

`systemd` 디렉터리는 CUBIC C1의 Raspberry Pi에서 사용하는 **부팅 및 시스템 서비스 구성**을 포함한다.

센서, micro-ROS Agent, EKF, SLAM, Localization, Navigation, Foxglove 및 UPS 관리 기능을 개별 서비스로 분리하여,
부팅 시 필요한 구성 요소를 자동 실행하고 문제가 발생한 기능만 독립적으로 재시작하거나 점검할 수 있도록 구성하였다.

---

## 서비스 구성

| Service | 역할 |
|---|---|
| [`micro-ros-mcu.service`](./system/micro-ros-mcu.service) | Motor Controller용 micro-ROS Agent |
| [`micro-ros-pcu.service`](./system/micro-ros-pcu.service) | Power Controller용 micro-ROS Agent |
| [`bno055.service`](./system/bno055.service) | BNO055 IMU Node |
| [`lidar.service`](./system/lidar.service) | Front / Rear LiDAR |
| [`cubic-ekf.service`](./system/cubic-ekf.service) | Wheel Odometry + IMU 기반 EKF |
| [`cubic-slam.service`](./system/cubic-slam.service) | SLAM Toolbox 기반 Mapping |
| [`cubic-localization.service`](./system/cubic-localization.service) | 저장된 Map 기반 Localization |
| [`cubic-navigation.service`](./system/cubic-navigation.service) | Nav2 Navigation |
| [`cubic-mode-manager.service`](./system/cubic-mode-manager.service) | Mapping / Navigation Mode 관리 |
| [`foxglove.service`](./system/foxglove.service) | Foxglove Bridge |
| [`x1208-shutdown.service`](./system/x1208-shutdown.service) | X1208 UPS 상태 감시 및 Safe Shutdown |

---

## 운용 구조

```text
Hardware / Communication
├─ micro-ros-mcu.service
├─ micro-ros-pcu.service
├─ bno055.service
└─ lidar.service

State Estimation
└─ cubic-ekf.service

Operation Mode
├─ cubic-slam.service
│   └─ Mapping
│
└─ cubic-localization.service
    └─ cubic-navigation.service
        └─ Navigation

System Utility
├─ cubic-mode-manager.service
├─ foxglove.service
└─ x1208-shutdown.service
    └─ scripts/x1208_shutdown.py
```

Mapping과 Navigation 계층은 운용 목적에 따라 선택적으로 실행한다.

> ROS 2 전체 구조와 Mode 전환 방식은
> [`ROS 2 Integration`](../docs/02_Architecture/03_Software/ROS%202%20Integration.md)을 참고한다.

---

## Repository 구조

```text
systemd/
├─ README.md
├─ scripts/
│  └─ x1208_shutdown.py
└─ system/
   ├─ bno055.service
   ├─ cubic-ekf.service
   ├─ cubic-localization.service
   ├─ cubic-mode-manager.service
   ├─ cubic-navigation.service
   ├─ cubic-slam.service
   ├─ foxglove.service
   ├─ lidar.service
   ├─ micro-ros-mcu.service
   ├─ micro-ros-pcu.service
   └─ x1208-shutdown.service
```

실제 Raspberry Pi에서는 service 파일을 다음 경로에 배치한다.

```text
/etc/systemd/system/
```

X1208 Shutdown Script는 현재 CUBIC C1에서 다음 위치에 배포하여 사용한다.

```text
/home/cubic/x1208_shutdown.py
```

따라서 저장소의 `scripts/x1208_shutdown.py`는 실제 운용 Script의 소스 보관본이며,
`x1208-shutdown.service`의 `ExecStart` 경로는 현재 CUBIC C1 환경을 기준으로 작성되어 있다.

---

## 서비스 관리

서비스 파일을 추가하거나 수정한 경우:

```bash
sudo systemctl daemon-reload
```

부팅 시 자동 실행:

```bash
sudo systemctl enable <service-name>
```

시작 / 정지 / 재시작 / 상태 확인:

```bash
sudo systemctl start <service-name>
sudo systemctl stop <service-name>
sudo systemctl restart <service-name>
sudo systemctl status <service-name>
```

로그 확인:

```bash
journalctl -u <service-name> -b
```

실시간 로그:

```bash
journalctl -u <service-name> -f
```

---

## X1208 UPS Shutdown

Raspberry Pi 5와 SSD에는 X1208 UPS HAT을 적용하여
메인 전원 차단 시 SBC가 즉시 비정상 종료되지 않도록 구성하였다.

`x1208-shutdown.service`는 부팅 시 다음 Script를 상시 실행한다.

```text
/home/cubic/x1208_shutdown.py
```

동작 구조:

```text
Main Power
    │
    ▼
X1208 UPS
    │
    ▼
x1208_shutdown.py
    │
    ├─ External Power ON  → Normal Operation
    └─ External Power OFF → UPS Backup / Safe Shutdown
```

메인 전원 차단 시험을 5회 반복한 결과 **5/5 정상 종료**를 확인하였다.

상세 검증 결과는
[`System_Integration_Test.md`](../docs/04_Test/System_Integration_Test.md)를 참고한다.

---

## 경로 설정 주의

서비스 파일 내부의 사용자 및 Workspace 경로는 CUBIC C1의 실제 운용 환경을 기준으로 한다.

주요 경로 예:

```text
/home/cubic/cubic_ws/
/home/cubic/microros_ws/
/home/cubic/x1208_shutdown.py
```

다른 환경에서 사용할 경우 다음 항목을 수정해야 할 수 있다.

- `User=`
- `ExecStart=`
- ROS 2 Setup Script 경로
- Workspace `install/setup.bash`
- Serial Device 경로
- 실행 Script 경로

---

## Mapping / Navigation

### Mapping

```text
cubic-slam.service
```

SLAM Toolbox를 실행하여 새로운 환경의 Map을 생성한다.

### Navigation

```text
cubic-localization.service
cubic-navigation.service
```

저장된 Map을 이용하여 Localization 및 Nav2를 실행한다.

Mode 전환은 다음 서비스에서 관리한다.

```text
cubic-mode-manager.service
```

---

## 관련 문서

- ROS 2 구조 및 Topic  
  → [`ROS 2 Integration`](../docs/02_Architecture/03_Software/ROS%202%20Integration.md)

- ROS 2 Workspace  
  → [`../cubic_ws/README.md`](../cubic_ws/README.md)

- MCU / PCU Firmware  
  → [`../firmware/README.md`](../firmware/README.md)

- UART / 센서 / 전장 구성  
  → [`Electrical Architecture`](../docs/02_Architecture/02_Electrical/README.md)

- 문제 해결 및 시스템 검증  
  → [`04_Test`](../docs/04_Test/)
