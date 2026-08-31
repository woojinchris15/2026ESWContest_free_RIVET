#!/usr/bin/env python3

import subprocess
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformListener, TransformException


SLAM_SERVICE = "cubic-slam.service"
LOCALIZATION_SERVICE = "cubic-localization.service"
NAVIGATION_SERVICE = "cubic-navigation.service"

MAP_FILE_BASE = (
    "/home/cubic/cubic_ws/src/"
    "cubic_navigation/maps/cubic_map"
)


class CubicModeManager(Node):

    def __init__(self):
        super().__init__("cubic_mode_manager")

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # Publishers
        self.mode_pub = self.create_publisher(
            String,
            "/cubic/mode",
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        # Services
        self.mapping_srv = self.create_service(
            Trigger,
            "/cubic/mode/mapping",
            self.mapping_callback
        )

        self.navigation_srv = self.create_service(
            Trigger,
            "/cubic/mode/navigation",
            self.navigation_callback
        )

        self.switching_state = None

        self.timer = self.create_timer(
            1.0,
            self.publish_mode
        )

        self.get_logger().info(
            "CUBIC Mode Manager ready"
        )

    # =========================================================
    # Shell utilities
    # =========================================================

    def run_command(self, args, timeout=15):

        try:
            result = subprocess.run(
                args,
                capture_output=True,
                text=True,
                timeout=timeout
            )

            out = result.stdout.strip()
            err = result.stderr.strip()

            if result.returncode != 0:
                return False, err if err else out

            return True, out

        except subprocess.TimeoutExpired:
            return False, "command timeout"

        except Exception as e:
            return False, str(e)

    def systemctl(self, action, service, timeout=20):

        return self.run_command(
            [
                "sudo",
                "-n",
                "/usr/bin/systemctl",
                action,
                service
            ],
            timeout=timeout
        )

    def service_active(self, service):

        result = subprocess.run(
            [
                "/usr/bin/systemctl",
                "is-active",
                service
            ],
            capture_output=True,
            text=True
        )

        return result.stdout.strip() == "active"

    def wait_service_active(
        self,
        service,
        timeout=15.0
    ):

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:

            if self.service_active(service):
                return True

            time.sleep(0.25)

        return False

    # =========================================================
    # Lifecycle
    # =========================================================

    def lifecycle_state(self, node):

        success, out = self.run_command(
            [
                "ros2",
                "lifecycle",
                "get",
                node
            ],
            timeout=4
        )

        if not success:
            return None

        text = out.lower()

        if "active [3]" in text:
            return "active"

        if "inactive [2]" in text:
            return "inactive"

        if "unconfigured [1]" in text:
            return "unconfigured"

        return text

    def wait_lifecycle_active(
        self,
        node,
        timeout=20.0
    ):

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:

            if self.lifecycle_state(node) == "active":
                return True

            time.sleep(0.25)

        return False

    # =========================================================
    # Safety
    # =========================================================

    def stop_robot(self):

        self.get_logger().info(
            "Sending zero cmd_vel"
        )

        msg = Twist()

        for _ in range(8):
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.05)

    # =========================================================
    # TF
    # =========================================================

    def get_map_base_pose(self):

        try:
            return self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                Time(),
                timeout=Duration(seconds=3.0)
            )

        except TransformException as e:

            self.get_logger().error(
                f"map -> base_link unavailable: {e}"
            )

            return None

    def wait_map_base_tf(self, timeout=15.0):

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:

            try:
                self.tf_buffer.lookup_transform(
                    "map",
                    "base_link",
                    Time(),
                    timeout=Duration(seconds=0.5)
                )

                return True

            except TransformException:
                pass

            time.sleep(0.25)

        return False

    # =========================================================
    # Map
    # =========================================================

    def save_map(self):

        self.get_logger().info(
            "Saving current map..."
        )

        success, out = self.run_command(
            [
                "ros2",
                "run",
                "nav2_map_server",
                "map_saver_cli",
                "-f",
                MAP_FILE_BASE
            ],
            timeout=20
        )

        if not success:
            self.get_logger().error(
                f"Map save failed: {out}"
            )
            return False

        self.get_logger().info(
            "Map saved"
        )

        return True

    # =========================================================
    # Initial pose
    # =========================================================

    def publish_initial_pose_cli(self, tf):

        x = tf.transform.translation.x
        y = tf.transform.translation.y

        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w

        msg = (
            "{header: {frame_id: map}, "
            "pose: {"
            "pose: {"
            f"position: {{x: {x}, y: {y}, z: 0.0}}, "
            f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}"
            "}, "
            "covariance: ["
            "0.25,0,0,0,0,0,"
            "0,0.25,0,0,0,0,"
            "0,0,0,0,0,0,"
            "0,0,0,0,0,0,"
            "0,0,0,0,0,0,"
            "0,0,0,0,0,0.0685389"
            "]}}"
        )

        self.get_logger().info(
            f"Setting AMCL pose: x={x:.3f}, y={y:.3f}"
        )

        # 수동으로 성공한 것과 동일한 방식으로 외부 CLI publish
        for attempt in range(3):

            success, out = self.run_command(
                [
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    "/initialpose",
                    "geometry_msgs/msg/PoseWithCovarianceStamped",
                    msg
                ],
                timeout=8
            )

            if success:
                self.get_logger().info(
                    f"Initial pose published ({attempt + 1}/3)"
                )

            else:
                self.get_logger().warning(
                    f"Initial pose publish failed: {out}"
                )

            time.sleep(0.7)

        return True

    # =========================================================
    # Mapping mode
    # =========================================================

    def mapping_callback(self, request, response):

        if self.switching_state is not None:
            response.success = False
            response.message = (
                "Another mode transition is running"
            )
            return response

        self.switching_state = (
            "SWITCHING_TO_MAPPING"
        )

        try:
            self.get_logger().info(
                "=== SWITCHING TO MAPPING ==="
            )

            self.stop_robot()

            # Navigation 먼저 종료
            self.systemctl(
                "stop",
                NAVIGATION_SERVICE
            )

            # localization도 종료
            self.systemctl(
                "stop",
                LOCALIZATION_SERVICE
            )

            time.sleep(1.0)

            # SLAM 시작
            success, out = self.systemctl(
                "start",
                SLAM_SERVICE
            )

            if not success:
                response.success = False
                response.message = (
                    f"Failed to start SLAM: {out}"
                )
                return response

            if not self.wait_service_active(
                SLAM_SERVICE,
                timeout=15
            ):
                response.success = False
                response.message = (
                    "SLAM service did not become active"
                )
                return response

            if not self.wait_lifecycle_active(
                "/slam_toolbox",
                timeout=15
            ):
                response.success = False
                response.message = (
                    "slam_toolbox did not reach ACTIVE"
                )
                return response

            self.get_logger().info(
                "MAPPING mode ready"
            )

            response.success = True
            response.message = (
                "CUBIC switched to MAPPING"
            )

            return response

        finally:
            self.switching_state = None

    # =========================================================
    # Navigation mode
    # =========================================================

    def navigation_callback(
        self,
        request,
        response
    ):

        if self.switching_state is not None:
            response.success = False
            response.message = (
                "Another mode transition is running"
            )
            return response

        self.switching_state = (
            "SWITCHING_TO_NAVIGATION"
        )

        try:

            self.get_logger().info(
                "=== SWITCHING TO NAVIGATION ==="
            )

            # -------------------------------------------------
            # 1. SLAM이 살아 있을 때 현재 map pose 확보
            # -------------------------------------------------

            current_pose = self.get_map_base_pose()

            if current_pose is None:
                response.success = False
                response.message = (
                    "Could not capture map -> base_link pose"
                )
                return response

            self.get_logger().info(
                "Current SLAM pose captured"
            )

            # -------------------------------------------------
            # 2. Robot stop
            # -------------------------------------------------

            self.stop_robot()

            # -------------------------------------------------
            # 3. Map 저장
            # -------------------------------------------------

            if not self.save_map():
                response.success = False
                response.message = (
                    "Map save failed"
                )
                return response

            # -------------------------------------------------
            # 4. SLAM 종료
            # -------------------------------------------------

            self.get_logger().info(
                "Stopping SLAM..."
            )

            self.systemctl(
                "stop",
                SLAM_SERVICE
            )

            time.sleep(1.0)

            # -------------------------------------------------
            # 5. Localization 시작
            # -------------------------------------------------

            self.get_logger().info(
                "Starting localization..."
            )

            success, out = self.systemctl(
                "start",
                LOCALIZATION_SERVICE
            )

            if not success:
                response.success = False
                response.message = (
                    f"Failed to start localization: {out}"
                )
                return response

            if not self.wait_service_active(
                LOCALIZATION_SERVICE,
                timeout=15
            ):
                response.success = False
                response.message = (
                    "Localization service failed"
                )
                return response

            # -------------------------------------------------
            # 6. AMCL + map_server active 확인
            # -------------------------------------------------

            if not self.wait_lifecycle_active(
                "/map_server",
                timeout=15
            ):
                response.success = False
                response.message = (
                    "map_server not ACTIVE"
                )
                return response

            if not self.wait_lifecycle_active(
                "/amcl",
                timeout=15
            ):
                response.success = False
                response.message = (
                    "AMCL not ACTIVE"
                )
                return response

            self.get_logger().info(
                "Localization active"
            )

            # -------------------------------------------------
            # 7. AMCL initial pose 설정
            # -------------------------------------------------

            time.sleep(1.0)

            self.publish_initial_pose_cli(
                current_pose
            )

            # -------------------------------------------------
            # 8. map -> base_link TF 확인
            # -------------------------------------------------

            self.get_logger().info(
                "Waiting for map -> base_link..."
            )

            if not self.wait_map_base_tf(
                timeout=15
            ):

                response.success = False
                response.message = (
                    "AMCL initial pose failed: "
                    "map -> base_link not available"
                )

                return response

            self.get_logger().info(
                "AMCL localization confirmed"
            )

            # -------------------------------------------------
            # 9. Navigation 시작
            # -------------------------------------------------

            self.get_logger().info(
                "Starting Nav2 navigation..."
            )

            success, out = self.systemctl(
                "start",
                NAVIGATION_SERVICE
            )

            if not success:
                response.success = False
                response.message = (
                    f"Navigation start failed: {out}"
                )
                return response

            if not self.wait_service_active(
                NAVIGATION_SERVICE,
                timeout=15
            ):
                response.success = False
                response.message = (
                    "Navigation service failed"
                )
                return response

            # -------------------------------------------------
            # 10. 핵심 Nav2 lifecycle 확인
            # -------------------------------------------------

            required_nodes = [
                "/controller_server",
                "/planner_server",
                "/smoother_server",
                "/behavior_server",
                "/velocity_smoother",
                "/collision_monitor",
                "/bt_navigator",
            ]

            for node in required_nodes:

                self.get_logger().info(
                    f"Waiting for {node}"
                )

                if not self.wait_lifecycle_active(
                    node,
                    timeout=20
                ):

                    response.success = False
                    response.message = (
                        f"{node} did not reach ACTIVE"
                    )

                    return response

            self.get_logger().info(
                "NAVIGATION mode ready"
            )

            response.success = True
            response.message = (
                "CUBIC switched to NAVIGATION"
            )

            return response

        finally:
            self.switching_state = None

    # =========================================================
    # Status
    # =========================================================

    def publish_mode(self):

        msg = String()

        if self.switching_state:

            msg.data = self.switching_state

        elif self.service_active(
            NAVIGATION_SERVICE
        ):

            msg.data = "NAVIGATION"

        elif self.service_active(
            SLAM_SERVICE
        ):

            msg.data = "MAPPING"

        elif self.service_active(
            LOCALIZATION_SERVICE
        ):

            msg.data = "LOCALIZATION"

        else:

            msg.data = "IDLE"

        self.mode_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = CubicModeManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
