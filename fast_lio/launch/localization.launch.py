"""
FAST-LIO Localization Launch File

Livox MID-360 LiDAR-Inertial Odometry for localization on a pre-built PCD map.

================================================================================
LAUNCH ARGUMENTS
================================================================================
  map_path        : Path to the PCD map file (required)   (default: '')
  config_file     : Localization config file path         (default: <pkg>/config/localization/localization.yaml)
  use_sim_time    : Use simulation time                   (default: 'false')
  rviz            : Launch RViz                           (default: 'true')
  lio_config_path : FAST-LIO config directory             (default: <pkg>/config/slam)
  lio_config_file : FAST-LIO config file name             (default: 'mid360.yaml')
  foxglove        : Launch Foxglove bridge (ws://localhost:8765) (default: 'true')

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
    - /fast_lio/localization/odometry (nav_msgs/Odometry) - map-aligned pose
    - /fast_lio/cloud_registered (sensor_msgs/PointCloud2)

================================================================================
EXAMPLES
================================================================================
  # Basic localization with pre-built map
  ros2 launch fast_lio localization.launch.py map_path:=/path/to/map.pcd

  # Localization without RViz
  ros2 launch fast_lio localization.launch.py map_path:=/path/to/map.pcd rviz:=false

  # Bag playback with simulation time
  ros2 launch fast_lio localization.launch.py map_path:=/path/to/map.pcd use_sim_time:=true

  # Disable Foxglove bridge
  ros2 launch fast_lio localization.launch.py map_path:=/path/to/map.pcd foxglove:=false

================================================================================
WORKFLOW
================================================================================
  1. First, create a map using mapping.launch.py
  2. Save the PCD map (automatically saved on shutdown or manually)
  3. Use this launch file with the saved map for localization
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Package paths
    package_path = get_package_share_directory('fast_lio')
    boat_desc_path = get_package_share_directory('boat_description')

    default_loc_config = os.path.join(package_path, 'config', 'localization', 'localization.yaml')
    default_lio_config_path = os.path.join(package_path, 'config', 'slam')
    default_rviz_config = os.path.join(package_path, 'rviz', 'localization.rviz')

    # Launch arguments
    map_path = LaunchConfiguration('map_path')
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    lio_config_path = LaunchConfiguration('lio_config_path')
    lio_config_file = LaunchConfiguration('lio_config_file')
    foxglove = LaunchConfiguration('foxglove')

    declare_map_path_cmd = DeclareLaunchArgument(
        'map_path',
        default_value='',
        description='Path to the PCD map file (required)'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=default_loc_config,
        description='Full path to localization config file'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RVIZ'
    )
    declare_lio_config_path_cmd = DeclareLaunchArgument(
        'lio_config_path',
        default_value=default_lio_config_path,
        description='FAST-LIO config directory'
    )
    declare_lio_config_file_cmd = DeclareLaunchArgument(
        'lio_config_file',
        default_value='mid360.yaml',
        description='FAST-LIO config file'
    )
    declare_foxglove_cmd = DeclareLaunchArgument(
        'foxglove',
        default_value='true',
        description='Launch Foxglove bridge (connect via ws://localhost:8765)'
    )

    # FAST-LIO node (with map saving disabled for localization mode)
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        parameters=[
            PathJoinSubstitution([lio_config_path, lio_config_file]),
            {
                'use_sim_time': use_sim_time,
                'pcd_save.pcd_save_en': False,  # Disable map saving in localization mode
            }
        ],
        output='screen'
    )

    # Localization node
    localization_node = Node(
        package='fast_lio',
        executable='localization_node',
        name='localization_node',
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'map_path': map_path,
            }
        ],
        output='screen'
    )

    # Optional RVIZ
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config] if os.path.exists(default_rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz)
    )

    # Robot state publisher (boat_description URDF)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(boat_desc_path, 'launch', 'robot_state_publisher.launch.py')
        )
    )

    # Foxglove bridge (conditional)
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(foxglove)
    )

    ld = LaunchDescription()
    # Declare arguments
    ld.add_action(declare_map_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_lio_config_path_cmd)
    ld.add_action(declare_lio_config_file_cmd)
    ld.add_action(declare_foxglove_cmd)
    # Nodes
    ld.add_action(robot_state_publisher)
    ld.add_action(fast_lio_node)
    ld.add_action(localization_node)
    ld.add_action(rviz_node)
    ld.add_action(foxglove_bridge_node)

    return ld
