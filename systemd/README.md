# CUBIC systemd Services

`systemd` 디렉터리는 CUBIC C1의 Raspberry Pi에서 사용하는 **부팅 및 ROS 2 서비스 구성 파일**을 포함한다.

센서, micro-ROS Agent, EKF, SLAM, Localization, Navigation 및 Foxglove 등의 주요 프로세스를 개별 서비스로 분리하여 관리한다.

이를 통해 시스템 부팅 시 필요한 구성 요소를 자동으로 실행하고, 문제가 발생한 서비스만 독립적으로 재시작하거나 상태를 확인할 수 있다.

---

## 📦 서비스 구성

| Service                      | 역할                                |
| ---------------------------- | --------------------------------- |
| [`micro-ros-mcu.service`](./system/micro-ros-mcu.service)      | Motor Controller용 micro-ROS Agent |
| [`micro-ros-pcu.service`](./system/micro-ros-pcu.service)      | Power Controller용 micro-ROS Agent |
| [`bno055.service`](./system/bno055.service)             | BNO055 IMU Node 실행                |
| [`lidar.service`](./system/lidar.service)              | Front / Rear LiDAR 실행             |
| [`cubic-ekf.service`](./system/cubic-ekf.service)          | Wheel Odometry + IMU 기반 EKF       |
| [`cubic-slam.service`](./system/cubic-slam.service)         | SLAM Toolbox 기반 Mapping           |
| [`cubic-localization.service`](./system/cubic-localization.service) | 저장된 Map 기반 Localization           |
| [`cubic-navigation.service`](./system/cubic-navigation.service)   | Nav2 Navigation                   |
| [`cubic-mode-manager.service`](./system/cubic-mode-manager.service) | Mapping / Navigation Mode 관리      |
| [`foxglove.service`](./system/foxglove.service)           | Foxglove Bridge                   |
| [`x1208-shutdown.service`](./system/x1208-shutdown.service)     | X1208 UPS Shutdown 처리             |

---

## ⚙️ 운용 구조

CUBIC의 서비스는 기능별로 분리되어 있으며, 기본 하드웨어 계층과 자율주행 계층을 독립적으로 관리한다.

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
│    └─ Mapping
│
└─ cubic-localization.service
     └─ cubic-navigation.service
          └─ Navigation

Utility
├─ cubic-mode-manager.service
├─ foxglove.service
└─ x1208-shutdown.service
```

Mapping과 Navigation에 필요한 구성 요소는 운용 목적에 따라 선택적으로 실행한다.

> ROS 2 전체 소프트웨어 구조 및 Mode 전환 방식은
> [`ROS 2 Integration`](../docs/02_Architecture/03_Software/ROS%202%20Integration.md)을 참고한다.

---

## 📁 파일 위치

저장소에는 서비스 원본 파일이 다음 위치에 정리되어 있다.

```text
systemd/
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

실제 Raspberry Pi에서 사용하려면 각 서비스 파일을 시스템의 systemd 서비스 경로에 배치해야 한다.

일반적인 위치:

```text
/etc/systemd/system/
```

---

## 🔧 서비스 등록

서비스 파일을 등록하거나 수정한 뒤에는 systemd 설정을 다시 읽는다.

```bash
sudo systemctl daemon-reload
```

부팅 시 자동 실행하도록 설정할 경우:

```bash
sudo systemctl enable <service-name>
```

예:

```bash
sudo systemctl enable lidar.service
```

---

## ▶️ 서비스 제어

### 시작

```bash
sudo systemctl start <service-name>
```

### 정지

```bash
sudo systemctl stop <service-name>
```

### 재시작

```bash
sudo systemctl restart <service-name>
```

### 상태 확인

```bash
sudo systemctl status <service-name>
```

예:

```bash
sudo systemctl status cubic-navigation.service
```

---

## 📋 로그 확인

서비스 실행 중 발생한 로그는 `journalctl`을 통해 확인할 수 있다.

```bash
journalctl -u <service-name>
```

실시간 로그:

```bash
journalctl -u <service-name> -f
```

예:

```bash
journalctl -u micro-ros-mcu.service -f
```

부팅 이후 로그만 확인하려면:

```bash
journalctl -u <service-name> -b
```

---

## ⚠️ 경로 설정 주의

서비스 파일 내부의 ROS 2 Workspace 및 사용자 경로는 CUBIC 개발 환경을 기준으로 작성되어 있다.

예를 들어 다음과 같은 경로가 포함될 수 있다.

```text
/home/cubic/cubic_ws/
/home/cubic/microros_ws/
```

다른 사용자 계정이나 다른 Workspace 경로에서 사용할 경우 서비스 파일의 다음 항목을 실제 환경에 맞게 수정해야 한다.

* `User=`
* `ExecStart=`
* ROS 2 Setup Script 경로
* Workspace `install/setup.bash`
* Serial Device 경로
* 실행 Script 경로

> 본 저장소의 systemd 파일은 CUBIC C1의 실제 운용 환경을 기준으로 작성되어 있으므로, 다른 시스템에서 그대로 사용할 경우 경로 수정이 필요할 수 있다.

---

## 🔌 micro-ROS Agent

Motor Controller와 Power Controller는 각각 별도의 UART 연결을 사용하므로 micro-ROS Agent 또한 독립적으로 실행한다.

| Controller       | Service                 |
| ---------------- | ----------------------- |
| Motor Controller | `micro-ros-mcu.service` |
| Power Controller | `micro-ros-pcu.service` |

하위 제어기 Firmware 및 micro-ROS 구성은 다음 문서를 참고한다.

[`../firmware/README.md`](../firmware/README.md)

---

## 🗺️ Mapping / Navigation

CUBIC은 Mapping과 Navigation 기능을 분리하여 운용한다.

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

Mode 전환은 다음 서비스를 통해 관리한다.

```text
cubic-mode-manager.service
```

---

## 📚 관련 문서

* ROS 2 전체 구조 및 Topic
  → [`ROS 2 Integration`](../docs/02_Architecture/03_Software/ROS%202%20Integration.md)

* ROS 2 Workspace 구성
  → [`../cubic_ws/README.md`](../cubic_ws/README.md)

* MCU / PCU Firmware
  → [`../firmware/README.md`](../firmware/README.md)

* UART, 센서 및 제어기 핀맵 / 결선 정보
  → [`Electrical Architecture`](../docs/02_Architecture/02_Electrical/README.md)

* 개발 중 발생한 서비스 및 통신 문제
  → [`문제해결 및 트러블슈팅`](../docs/04_Test/문제해결%20및%20트러블슈팅.md)
