import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # 获取各功能包路径
    iqr_tb4_bringup_dir = get_package_share_directory("iqr_tb4_bringup")
    arm_ctrl_dir = get_package_share_directory("arm_ctrl")
    turtlebot4_viz_dir = get_package_share_directory("turtlebot4_viz")


    # 1
    iqr_tb4_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([iqr_tb4_bringup_dir, "launch", "bringup.launch.py"])]
        ),
    )
 
    # 2
    test_demo_launch = Node(
                package="arm_ctrl",
                executable="test_demo",
                name="ArmController",
            )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(arm_ctrl_dir, 'rviz', 'default.rviz')],
        output='screen'
    )
    return LaunchDescription(
        [
            iqr_tb4_bringup_launch,
            test_demo_launch,
            rviz_node,
        ]
    )
