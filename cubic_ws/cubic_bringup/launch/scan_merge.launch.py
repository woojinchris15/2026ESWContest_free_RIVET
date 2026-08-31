from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # Front LiDAR mask filter
    front_filter = Node(
        package='cubic_bringup',
        executable='scan_mask_filter.py',
        name='scan_front_filter',
        output='screen',
        parameters=[{
            'input_topic': '/scan_front',
            'output_topic': '/scan_front_filtered',
            'mask_center_deg': 0.0,
            'mask_width_deg': 90.0,
        }]
    )

    # Rear LiDAR mask filter
    rear_filter = Node(
        package='cubic_bringup',
        executable='scan_mask_filter.py',
        name='scan_rear_filter',
        output='screen',
        parameters=[{
            'input_topic': '/scan_rear',
            'output_topic': '/scan_rear_filtered',
            'mask_center_deg': 0.0,
            'mask_width_deg': 90.0,
        }]
    )

    # Dual LiDAR merger
    merger_container = ComposableNodeContainer(
        name='cubic_laser_merger_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',

        composable_node_descriptions=[
            ComposableNode(
                package='dual_laser_merger',
                plugin='merger_node::MergerNode',
                name='dual_laser_merger',

                parameters=[
                    {
                        'laser_1_topic': '/scan_front_filtered'
                    },
                    {
                        'laser_2_topic': '/scan_rear_filtered'
                    },
                    {
                        'merged_scan_topic': '/scan'
                    },

                    # 두 LiDAR 모두 URDF/TF 기준으로 base_link에 변환
                    {
                        'target_frame': 'base_link'
                    },

                    # 위치/각도는 URDF TF에서 이미 반영되므로
                    # merger 내부 추가 offset은 0
                    {
                        'laser_1_x_offset': 0.0
                    },
                    {
                        'laser_1_y_offset': 0.0
                    },
                    {
                        'laser_1_yaw_offset': 0.0
                    },

                    {
                        'laser_2_x_offset': 0.0
                    },
                    {
                        'laser_2_y_offset': 0.0
                    },
                    {
                        'laser_2_yaw_offset': 0.0
                    },

                    # 두 scan timestamp 허용 오차
                    {
                        'tolerance': 0.02
                    },

                    # 최종 /scan 각도 범위
                    {
                        'angle_min': -3.141592654
                    },
                    {
                        'angle_max': 3.141592654
                    },
                    {
                        'angle_increment': 0.008726646
                    },

                    # 약 10 Hz
                    {
                        'scan_time': 0.1
                    },

                    {
                        'range_min': 0.05
                    },
                    {
                        'range_max': 40.0
                    },

                    # LiDAR 높이가 base_link 기준 약 +0.244m이므로
                    # 넉넉하게 통과
                    {
                        'min_height': -0.5
                    },
                    {
                        'max_height': 0.5
                    },

                    {
                        'use_inf': True
                    },
                ],
            ),
        ],
    )

    return LaunchDescription([
        front_filter,
        rear_filter,
        merger_container,
    ])
