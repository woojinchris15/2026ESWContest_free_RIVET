from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    nav2_bringup = FindPackageShare("nav2_bringup")

    params_file = "/home/cubic/cubic_ws/src/cubic_navigation/config/nav2_params.yaml"
    map_file = "/home/cubic/cubic_ws/src/cubic_navigation/maps/cubic_map.yaml"

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                nav2_bringup,
                "launch",
                "localization_launch.py"
            ])
        ),
        launch_arguments={
            "map": map_file,
            "params_file": params_file,
            "use_sim_time": "False",
            "autostart": "True",
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                nav2_bringup,
                "launch",
                "navigation_launch.py"
            ])
        ),
        launch_arguments={
            "params_file": params_file,
            "use_sim_time": "False",
            "autostart": "True",
        }.items(),
    )

    return LaunchDescription([
        localization,
        navigation,
    ])
