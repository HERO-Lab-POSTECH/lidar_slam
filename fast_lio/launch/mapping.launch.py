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
  output_map_path : Path to save PCD map on shutdown        (default: '' = auto-timestamp under $PKRC_MAP_DIR/fast_lio/<YYYYMMDD_HHMMSS>/)

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
    - /localization/fast_lio/odometry (nav_msgs/Odometry)
    - /localization/fast_lio/points_body (sensor_msgs/PointCloud2)
    - /fast_lio/path (nav_msgs/Path)

================================================================================
EXAMPLES
================================================================================
  # Basic mapping — map auto-saved to $PKRC_MAP_DIR/fast_lio/<timestamp>/map.pcd
  ros2 launch fast_lio mapping.launch.py

  # Mapping without RViz
  ros2 launch fast_lio mapping.launch.py use_rviz:=false

  # Mapping with custom map save path
  ros2 launch fast_lio mapping.launch.py output_map_path:=/path/to/save/map.pcd

  # Bag playback with simulation time
  ros2 launch fast_lio mapping.launch.py use_sim_time:=true
"""

import os
import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _resolve_map_path(user_path: str, pkg: str, filename: str) -> str:
    """Workspace-standard map save path resolution (spec §2.9).

    Empty user_path → auto-timestamp dir under $PKRC_MAP_DIR/<pkg>/
    (or ~/data/maps/<pkg>/), and update relative `latest` symlink.
    Non-empty user_path → use as-is, ensure parent dir exists.
    """
    from datetime import datetime
    from pathlib import Path

    if user_path:
        Path(user_path).parent.mkdir(parents=True, exist_ok=True)
        return user_path

    base = Path(os.environ.get('PKRC_MAP_DIR', os.path.expanduser('~/data/maps')))
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    target_dir = base / pkg / ts
    target_dir.mkdir(parents=True, exist_ok=True)
    latest = base / pkg / 'latest'
    if latest.is_symlink() or latest.exists():
        latest.unlink()
    latest.symlink_to(ts)
    return str(target_dir / filename)


def _setup_nodes(context):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    config_path = LaunchConfiguration('config_path').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'
    rviz_config_path = LaunchConfiguration('rviz_config_path').perform(context)
    output_map_path_arg = LaunchConfiguration('output_map_path').perform(context)

    resolved_save_path = _resolve_map_path(output_map_path_arg, 'fast_lio', 'map.pcd')

    package_path = get_package_share_directory('fast_lio')
    boat_desc_path = get_package_share_directory('boat_description')

    nodes = []

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[config_path,
                    {'use_sim_time': use_sim_time == 'true',
                     'pcd_save.save_path': resolved_save_path}],
        output='screen'
    )
    nodes.append(fast_lio_node)

    if use_rviz:
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
        ))

    # Robot state publisher (boat_description URDF)
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(boat_desc_path, 'launch', 'robot_state_publisher.launch.py')
        )
    ))

    return nodes


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config', 'slam', 'mid360.yaml')
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'fastlio.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('config_path', default_value=default_config_path,
            description='Full path to FAST-LIO config yaml file'),
        DeclareLaunchArgument('use_rviz', default_value='false',
            description='Use RViz to monitor results'),
        DeclareLaunchArgument('rviz_config_path', default_value=default_rviz_config_path,
            description='RViz config file path'),
        DeclareLaunchArgument('output_map_path', default_value='',
            description='Path to save PCD map on shutdown. Empty = auto-timestamp.'),
        OpaqueFunction(function=_setup_nodes),
    ])
