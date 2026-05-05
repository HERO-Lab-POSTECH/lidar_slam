"""
FAST-LIO Mapping Launch File

Livox MID-360 LiDAR-Inertial Odometry for 3D SLAM mapping.

================================================================================
LAUNCH ARGUMENTS
================================================================================
  use_sim_time    : Use simulation time                     (default: 'false')
  config_path     : FAST-LIO config file path               (default: <pkg>/config/slam/mid360.yaml)
  use_rviz        : Launch RViz                             (default: 'false')
  rviz_config_path: RViz config file path                   (default: <pkg>/rviz/fastlio.rviz)
  output_map_path : Path to save PCD map on shutdown        (default: '' = use default path)

================================================================================
TF TREE (provided by boat_description URDF)
================================================================================
  map
  └── odom
      └── base_link
          ├── livox_frame (LiDAR)
          ├── imu_link
          └── sonar_link

================================================================================
TOPICS
================================================================================
  Input:
    - /livox/lidar (livox_ros_driver2/CustomMsg)
    - /livox/imu (sensor_msgs/Imu)
  Output:
    - /fast_lio/odometry (nav_msgs/Odometry)
    - /fast_lio/cloud_registered (sensor_msgs/PointCloud2)
    - /fast_lio/path (nav_msgs/Path)

================================================================================
EXAMPLES
================================================================================
  # Basic mapping (default)
  ros2 launch fast_lio mapping.launch.py

  # Mapping without RViz
  ros2 launch fast_lio mapping.launch.py use_rviz:=false

  # Mapping with custom map save path
  ros2 launch fast_lio mapping.launch.py output_map_path:=/path/to/save/map.pcd

  # Bag playback with simulation time
  ros2 launch fast_lio mapping.launch.py use_sim_time:=true
"""

import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    boat_desc_path = get_package_share_directory('boat_description')
    default_config_path = os.path.join(package_path, 'config', 'slam', 'mid360.yaml')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_path = LaunchConfiguration('rviz_config_path')
    output_map_path = LaunchConfiguration('output_map_path')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Full path to FAST-LIO config yaml file'
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_config_path', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_output_map_path_cmd = DeclareLaunchArgument(
        'output_map_path', default_value='',
        description='Path to save PCD map on shutdown. Empty uses default.'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[config_path,
                    {'use_sim_time': use_sim_time,
                     'pcd_save.save_path': output_map_path}],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz)
    )

    # Robot state publisher (boat_description URDF)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(boat_desc_path, 'launch', 'robot_state_publisher.launch.py')
        )
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_output_map_path_cmd)

    ld.add_action(robot_state_publisher)
    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)

    return ld
