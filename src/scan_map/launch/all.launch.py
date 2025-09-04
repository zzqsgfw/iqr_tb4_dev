import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 定位到功能包的地址
    pkg_share = FindPackageShare(package="scan_map").find("scan_map")
    # 是否使用仿真时间，我们not用gazebo，这里设置成false
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    # 地图的分辨率
    resolution = LaunchConfiguration("resolution", default="0.05")
    # 地图的发布周期
    publish_period_sec = LaunchConfiguration("publish_period_sec", default="1.0")
    # 配置文件夹路径
    configuration_directory = LaunchConfiguration(
        "configuration_directory", default=os.path.join(pkg_share, "config")
    )
    # 配置文件
    configuration_basename = LaunchConfiguration(
        "configuration_basename", default="scan.lua"
    )
    rviz_config_dir = os.path.join(pkg_share, "config") + "/default.rviz"
    print(f"rviz config in {rviz_config_dir}")

    urdf_model_path = os.path.join(pkg_share, "urdf") + "/tb4_128.urdf"
    # tony@iqr-turtlebot4-128:~/zzq_ws$ xacro /opt/ros/humble/share/turtlebot4_description/urdf/lite/turtlebot4.urdf.xacro > ./tb4_128.urdf

    ############

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[urdf_model_path],
    )

    rplidar_node = Node(
        name="rplidar_composition",
        package="rplidar_ros",
        executable="rplidar_composition",
        output="screen",
        parameters=[
            {
                "serial_port": "/dev/RPLIDAR",
                "serial_baudrate": 256000,
                "frame_id": "rplidar_link",
                "inverted": False,
                "angle_compensate": True,
                "auto_standby": True,
            }
        ],
    )

    # imu_node = Node(
    #     package="imu_sensor_broadcaster",
    #     executable="imu_sensor_broadcaster",
    #     arguments=[
    #         {
    #             "serial_port": "/dev/external_imu",
    #             "serial_baudrate": 115200,
    #             "frame_id": "imu_link",
    #         }
    #     ],
    # )

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory",
            configuration_directory,
            "-configuration_basename",
            configuration_basename,
        ],
    )

    cartographer_occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-resolution",
            resolution,
            "-publish_period_sec",
            publish_period_sec,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_dir],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # ===============================================定义启动文件========================================================
    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rplidar_node)
    #  ld.add_action(imu_node)
    #  ld.add_action(cartographer_node)
    #  ld.add_action(cartographer_occupancy_grid_node)
    ld.add_action(rviz_node)

    return ld
