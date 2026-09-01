# ROS 2 통신 및 노드 구성

CUBIC의 상위 소프트웨어는 Raspberry Pi 5에서 **ROS 2 Jazzy**를 기반으로 동작한다.

각 센서와 하위제어기를 독립적인 노드 및 토픽으로 구성하고, 필요한 데이터를 ROS 2 네트워크를 통해 교환하는 분산형 구조를 사용하였다.

## 1. 주요 ROS 2 Topic

| Topic              | 역할                |
| ------------------ | ----------------- |
| `/cmd_vel`         | 로봇의 선속도 및 각속도 명령  |
| `/odom`            | 휠 이동량 기반 Odometry |
| `/bno055/imu`      | IMU 자세 및 각속도      |
| `/scan_front`      | 전방 LiDAR 데이터      |
| `/scan_rear`       | 후방 LiDAR 데이터      |
| `/scan`            | 전·후방 LiDAR 통합 데이터 |
| `/power/battery`   | 배터리 상태 정보         |
| `/power/fault`     | 전원부 이상 상태         |
| `/tf`              | 로봇 및 센서 좌표계 변환    |
| `/system/alert_ok` | 시스템 상태 알림         |
| `/odometry/filtered` | Wheel Odometry와 IMU를 EKF로 융합한 Odometry |

---

## 2. 주요 Node 구성

| Node / 기능                  | 역할                            |
| -------------------------- | ----------------------------- |
| Motor Controller micro-ROS | `/cmd_vel` 수신 및 `/odom` 제공    |
| Power Controller micro-ROS | 배터리 및 전원 상태 제공                |
| BNO055 Node                | IMU 데이터 제공                    |
| RPLIDAR Nodes              | 전·후방 LaserScan 생성             |
| LiDAR Merger               | 두 LiDAR 데이터를 하나의 `/scan`으로 통합 |
| Robot State Publisher      | URDF 기반 TF 생성                 |
| SLAM Toolbox               | Mapping 수행                    |
| AMCL                       | 저장된 지도 기반 위치 추정               |
| Nav2                       | 경로 계획 및 주행 명령 생성              |
| Foxglove Bridge            | 외부 모니터링 및 제어 인터페이스 제공         |
| EKF (`robot_localization`) | Wheel Odometry와 IMU 회전 정보를 융합하여 `odom → base_link` 생성 |

---

## 3. Sensor Data Flow

```text
Front LiDAR ── /scan_front ─┐
                            ├── LiDAR Merger ── /scan
Rear LiDAR ─── /scan_rear ──┘

BNO055 ───────── /bno055/imu

Motor Controller ── /odom

Power Controller ── /power/battery
                 └─ /power/fault
```

각 장치가 생성한 데이터는 ROS 2 토픽을 통해 상위 기능에 전달된다.

특히 두 개의 LiDAR 데이터는 각각 독립적으로 수집된 뒤 LiDAR Merger에서 하나의 `/scan`으로 통합하여 SLAM 및 Navigation에서 동일한 센서 입력으로 사용할 수 있도록 구성하였다.

---

## 4. Navigation Data Flow

```text
Motor FG
   │
   ▼
/odom ───────────────┐
                     │
                     ▼
                   EKF
                     ▲
                     │
BNO055 ─ /bno055/imu ┘
                     │
                     ├─ /odometry/filtered
                     └─ odom → base_link
                              │
                              ▼
/scan ─────────── Localization / SLAM
                              │
                              ▼
                            Nav2
                              │
                          /cmd_vel
                              │
                              ▼
                     Motor Controller
```

Motor Controller에서 생성한 `/odom`은 FG 기반 Wheel Odometry로, 로봇의 기본적인 이동량과 회전량을 제공한다.

BNO055의 `yaw` 및 `angular velocity`는 EKF에 추가로 융합되어 Wheel Odometry의 **회전 방향 오차와 누적 heading drift를 보정**한다. IMU는 Wheel Odometry를 대체하는 것이 아니라 회전 추정을 보완하는 센서로 사용된다.

EKF는 `/odometry/filtered`와 `odom → base_link` TF를 생성하며, 이를 기반으로 상위 Localization 및 Navigation 계층에서 로봇의 자세를 사용한다.

Mapping Mode에서는 통합 LiDAR 데이터 `/scan`과 Odometry 정보를 SLAM Toolbox에서 사용하여 지도를 생성한다.

Navigation Mode에서는 저장된 지도와 센서 데이터를 기반으로 AMCL이 `map → odom` 위치 관계를 추정하고, Nav2가 현재 Pose와 주변 장애물 정보를 이용하여 목적지까지의 경로 및 `/cmd_vel`을 생성한다.


---

## 5. ROS 2 기반 모듈화

센서와 제어 기능을 각각 독립적인 Node와 Topic으로 분리함으로써 특정 센서나 제어 모듈을 변경하더라도 전체 소프트웨어 구조를 크게 수정하지 않고 교체할 수 있도록 구성하였다.

이는 CUBIC의 하드웨어 모듈화 구조와 대응되며, 새로운 센서 또는 상부 모듈을 추가할 때 기존 시스템과의 연동을 용이하게 한다.
