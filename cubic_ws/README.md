# CUBIC ROS 2 Workspace

`cubic_ws`는 CUBIC C1의 **ROS 2 기반 상위 제어 소프트웨어**를 포함하는 워크스페이스이다.

센서 Bring-up, Robot Model, LiDAR 처리, 상태 추정, SLAM, Localization, Navigation 및 운용 모드 전환 기능을 담당한다.

하위 모터 및 전원 제어는 별도의 RP2350 기반 펌웨어에서 수행하며, Raspberry Pi의 ROS 2 시스템과 micro-ROS를 통해 연결된다.

---

## 📦 Packages

| Package                                   | 역할                               |
| ----------------------------------------- | -------------------------------- |
| [`cubic_bringup`](./cubic_bringup/)       | 센서, URDF, LiDAR 처리 및 기본 Bring-up |
| [`cubic_navigation`](./cubic_navigation/) | EKF, SLAM, Localization 및 Nav2   |

---

## ⚠️ 필수 의존 패키지

CUBIC의 LiDAR 및 Scan Merge 기능을 사용하려면 다음 외부 ROS 2 패키지가 별도로 설치되어 있어야 한다.

| Package             | 역할                             |
| ------------------- | ------------------------------ |
| `sllidar_ros2`      | RPLIDAR C1 전·후방 LiDAR 구동       |
| `dual_laser_merger` | 두 LiDAR Scan을 하나의 `/scan`으로 병합 |

두 패키지가 설치되어 있지 않으면 LiDAR 관련 Launch 및 Scan Merge 기능이 정상적으로 동작하지 않는다.

ROS 2 환경에서 패키지가 정상적으로 인식되는지는 다음과 같이 확인할 수 있다.

```bash
ros2 pkg prefix sllidar_ros2
ros2 pkg prefix dual_laser_merger
```

패키지 경로가 출력되면 정상적으로 인식된 상태이다.

> `sllidar_ros2`와 `dual_laser_merger`는 CUBIC 저장소에 포함되지 않은 외부 패키지이므로 별도로 준비해야 한다.

---

## 📁 `cubic_bringup`

CUBIC의 센서와 Robot Model을 ROS 2 환경에 연결하는 기본 Bring-up 패키지이다.

```text
cubic_bringup
├─ config
│  ├─ bno055_params.yaml
│  ├─ scan_front_filter.yaml
│  └─ scan_rear_filter.yaml
├─ launch
│  ├─ lidar.launch.py
│  ├─ robot.launch.py
│  └─ scan_merge.launch.py
├─ scripts
│  ├─ mode_manager.py
│  └─ scan_mask_filter.py
└─ urdf
   └─ cubic_c1.urdf.xacro
```

### 주요 파일

| 파일                       | 역할                           |
| ------------------------ | ---------------------------- |
| `bno055_params.yaml`     | BNO055 IMU 설정                |
| `scan_front_filter.yaml` | 전방 LiDAR Scan 필터 설정          |
| `scan_rear_filter.yaml`  | 후방 LiDAR Scan 필터 설정          |
| `lidar.launch.py`        | 전·후방 LiDAR 실행                |
| `robot.launch.py`        | Robot Model 및 기본 Bring-up    |
| `scan_merge.launch.py`   | Dual LiDAR Scan 병합           |
| `mode_manager.py`        | Mapping / Navigation Mode 전환 |
| `scan_mask_filter.py`    | 로봇 자체 구조물에 해당하는 Scan 제거      |
| `cubic_c1.urdf.xacro`    | CUBIC C1 Robot Model 및 TF 정의 |

---

## 📁 `cubic_navigation`

CUBIC의 상태 추정, 지도 작성 및 자율주행 기능을 담당한다.

```text
cubic_navigation
├─ config
│  ├─ ekf.yaml
│  ├─ nav2_params.yaml
│  └─ slam.yaml
├─ launch
│  └─ navigation_full.launch.py
└─ maps
   ├─ cubic_map.pgm
   └─ cubic_map.yaml
```

### 주요 파일

| 파일                          | 역할                             |
| --------------------------- | ------------------------------ |
| `ekf.yaml`                  | Wheel Odometry 및 IMU 기반 EKF 설정 |
| `slam.yaml`                 | SLAM Toolbox 설정                |
| `nav2_params.yaml`          | Nav2 및 Localization 설정         |
| `navigation_full.launch.py` | 저장된 Map 기반 Navigation 실행       |
| `cubic_map.pgm`             | Occupancy Grid Map             |
| `cubic_map.yaml`            | Map 해상도 및 Origin 등의 메타데이터      |

---

## 🔨 Build

워크스페이스 루트에서 빌드한다.

```bash
cd ~/cubic_ws
colcon build
source install/setup.bash
```

특정 패키지만 빌드할 경우:

```bash
colcon build --packages-select cubic_bringup
```

```bash
colcon build --packages-select cubic_navigation
```

> `config` 파일만 수정한 경우에도 설치 방식에 따라 `install/`에 반영된 파일을 사용하는 경우가 있으므로, 실제 실행 경로를 확인하는 것을 권장한다.

---

## 🚀 주요 실행

### Robot Bring-up

```bash
ros2 launch cubic_bringup robot.launch.py
```

### LiDAR

```bash
ros2 launch cubic_bringup lidar.launch.py
```

### Scan Merge

```bash
ros2 launch cubic_bringup scan_merge.launch.py
```

### Navigation

```bash
ros2 launch cubic_navigation navigation_full.launch.py
```

실제 CUBIC 운용 환경에서는 주요 ROS 2 구성 요소를 systemd 서비스로 실행할 수 있다.

서비스 구성은 [`../systemd/README.md`](../systemd/README.md)를 참고한다.

---

## 🔧 설정 파일 위치

| 변경 항목               | 파일                                            |
| ------------------- | --------------------------------------------- |
| Robot Geometry / TF | `cubic_bringup/urdf/cubic_c1.urdf.xacro`      |
| BNO055              | `cubic_bringup/config/bno055_params.yaml`     |
| Front LiDAR Filter  | `cubic_bringup/config/scan_front_filter.yaml` |
| Rear LiDAR Filter   | `cubic_bringup/config/scan_rear_filter.yaml`  |
| EKF                 | `cubic_navigation/config/ekf.yaml`            |
| SLAM                | `cubic_navigation/config/slam.yaml`           |
| Nav2                | `cubic_navigation/config/nav2_params.yaml`    |
| Saved Map           | `cubic_navigation/maps/`                      |

---

## 📚 관련 문서

ROS 2 노드, Topic, TF 및 전체 소프트웨어 구조에 대한 자세한 설명은 다음 문서를 참고한다.

* [`ROS 2 Integration`](../docs/02_Architect/03_Software/ROS%202%20Integration.md)
* [`하위제어 상세설명`](../docs/02_Architect/03_Software/하위제어%20상세설명.md)
* [`문제해결 및 트러블슈팅`](../docs/04_Test/문제해결%20및%20트러블슈팅.md)

하위 실시간 제어 펌웨어는 [`../firmware/`](../firmware/)에 구성되어 있다.
