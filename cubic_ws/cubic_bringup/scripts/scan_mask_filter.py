#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class ScanMaskFilter(Node):

    def __init__(self):
        super().__init__('scan_mask_filter')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_filtered')

        # 마스킹할 중심 각도와 폭
        self.declare_parameter('mask_center_deg', 180.0)
        self.declare_parameter('mask_width_deg', 90.0)

        input_topic = self.get_parameter(
            'input_topic'
        ).get_parameter_value().string_value

        output_topic = self.get_parameter(
            'output_topic'
        ).get_parameter_value().string_value

        mask_center_deg = self.get_parameter(
            'mask_center_deg'
        ).get_parameter_value().double_value

        mask_width_deg = self.get_parameter(
            'mask_width_deg'
        ).get_parameter_value().double_value

        self.mask_center = math.radians(mask_center_deg)
        self.mask_half_width = math.radians(mask_width_deg / 2.0)

        self.publisher = self.create_publisher(
            LaserScan,
            output_topic,
            qos_profile_sensor_data
        )

        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'{input_topic} -> {output_topic} | '
            f'mask center={mask_center_deg:.1f} deg, '
            f'width={mask_width_deg:.1f} deg'
        )

    def scan_callback(self, msg):

        filtered = LaserScan()

        filtered.header = msg.header

        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment

        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time

        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max

        filtered.ranges = list(msg.ranges)
        filtered.intensities = list(msg.intensities)

        for i in range(len(filtered.ranges)):

            angle = msg.angle_min + i * msg.angle_increment

            # mask 중심과 현재 ray 사이의 최단 각도
            diff = normalize_angle(
                angle - self.mask_center
            )

            if abs(diff) <= self.mask_half_width:
                filtered.ranges[i] = float('inf')

                if i < len(filtered.intensities):
                    filtered.intensities[i] = 0.0

        self.publisher.publish(filtered)


def main(args=None):
    rclpy.init(args=args)

    node = ScanMaskFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
