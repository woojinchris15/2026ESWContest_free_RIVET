from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    front_lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='front_lidar',
        output='screen',
        parameters=[{
            'serial_port': '/dev/cubic_lidar_front',
            'serial_baudrate': 460800,
            'frame_id': 'lidar_front_link',
            'inverted': False,
            'angle_compensate': True,
        }],
        remappings=[
            ('scan', '/scan_front'),
        ]
    )

    rear_lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='rear_lidar',
        output='screen',
        parameters=[{
            'serial_port': '/dev/cubic_lidar_rear',
            'serial_baudrate': 460800,
            'frame_id': 'lidar_rear_link',
            'inverted': False,
            'angle_compensate': True,
        }],
        remappings=[
            ('scan', '/scan_rear'),
        ]
    )

    return LaunchDescription([
        front_lidar,
        rear_lidar,
    ])
