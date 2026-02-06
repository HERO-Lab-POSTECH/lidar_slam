"""
FAST-LIO Mapping Launch File

Livox MID-360 LiDAR-Inertial Odometry for 3D SLAM mapping.

================================================================================
LAUNCH ARGUMENTS
================================================================================
  use_sim_time    : Use simulation time                     (default: 'false')
  config_path     : FAST-LIO config directory               (default: <pkg>/config/slam)
  config_file     : Config file name                        (default: 'mid360.yaml')
  rviz            : Launch RViz                             (default: 'true')
  rviz_cfg        : RViz config file path                   (default: <pkg>/rviz/fastlio.rviz)
  save_map_path   : Path to save PCD map on shutdown        (default: '' = use default path)
  foxglove        : Launch Foxglove bridge (ws://localhost:8765) (default: 'true')
  qos_reliability : QoS reliability for sensor subscribers  (default: 'reliable')
                    Options: reliable, best_effort
                    Note: Use best_effort when playing back old bag files recorded
                          with BEST_EFFORT QoS

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
  ros2 launch fast_lio mapping.launch.py rviz:=false

  # Mapping with custom map save path
  ros2 launch fast_lio mapping.launch.py save_map_path:=/path/to/save/map.pcd

  # Bag playback with simulation time
  ros2 launch fast_lio mapping.launch.py use_sim_time:=true

  # Disable Foxglove bridge
  ros2 launch fast_lio mapping.launch.py foxglove:=false

  # BEST_EFFORT QoS (for old bag files)
  ros2 launch fast_lio mapping.launch.py use_sim_time:=true qos_reliability:=best_effort
"""

import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    boat_desc_path = get_package_share_directory('boat_description')
    default_config_path = os.path.join(package_path, 'config', 'slam')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    save_map_path = LaunchConfiguration('save_map_path')
    foxglove = LaunchConfiguration('foxglove')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_save_map_path_cmd = DeclareLaunchArgument(
        'save_map_path', default_value='',
        description='Path to save PCD map on shutdown. Empty uses default.'
    )
    declare_foxglove_cmd = DeclareLaunchArgument(
        'foxglove', default_value='true',
        description='Launch Foxglove bridge (connect via ws://localhost:8765)'
    )
    declare_qos_reliability_cmd = DeclareLaunchArgument(
        'qos_reliability', default_value='reliable',
        description='QoS reliability for sensor subscribers: reliable or best_effort'
    )

    qos_reliability = LaunchConfiguration('qos_reliability')

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time,
                     'pcd_save.save_path': save_map_path,
                     'qos_reliability': qos_reliability}],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    # Robot state publisher (boat_description URDF)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(boat_desc_path, 'launch', 'robot_state_publisher.launch.py')
        )
    )

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(foxglove)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_save_map_path_cmd)
    ld.add_action(declare_foxglove_cmd)
    ld.add_action(declare_qos_reliability_cmd)

    ld.add_action(robot_state_publisher)
    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)
    ld.add_action(foxglove_bridge_node)

    return ld
