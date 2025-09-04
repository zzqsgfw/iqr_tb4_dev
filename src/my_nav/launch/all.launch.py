import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取各功能包路径
    iqr_tb4_bringup_dir = get_package_share_directory("iqr_tb4_bringup")
    turtlebot4_navigation_dir = get_package_share_directory("turtlebot4_navigation")
    turtlebot4_viz_dir = get_package_share_directory("turtlebot4_viz")

    # 1
    iqr_tb4_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([iqr_tb4_bringup_dir, "launch", "bringup.launch.py"])]
        ),
    )
 
    # 2
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [turtlebot4_navigation_dir, "launch", "localization.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "map": "src/my_nav/map/empty_classroom.yaml",
            "params_file": "src/my_nav/config/localization.yaml",
        }.items(),
    )

    # 3
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [turtlebot4_navigation_dir, "launch", "nav2.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": "src/my_nav/config/nav2.yaml",
        }.items(),
    )

    # 4
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [turtlebot4_viz_dir, "launch", "view_robot.launch.py"]
                )
            ]
        ),
        # launch_arguments={
        #     'rviz_config': PathJoinSubstitution([  # 可选：指定自定义 Rviz 配置
        #         turtlebot4_viz_dir,
        #         'rviz',
        #         'navigation.rviz'
        #     ])
        # }.items()
    )

    return LaunchDescription(
        [
            iqr_tb4_bringup_launch,
            localization_launch,
            nav2_launch,
            rviz_launch,
        ]
    )
