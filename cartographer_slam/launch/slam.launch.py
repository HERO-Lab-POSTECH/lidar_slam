"""
Livox Mid-360 Cartographer 2D SLAM unified launch file

Supports both Livox CustomMsg and PointCloud2 input types.

Arguments:
  input_type    : 'livox' or 'pointcloud2' (default: 'livox')
  localization  : 'true' or 'false' (default: 'false')
  use_sim_time  : 'true' or 'false' (default: 'false')
  use_rviz      : 'true' or 'false' (default: 'true')
  resolution    : Occupancy grid resolution (default: '0.05')
  load_state_filename : Path to .pbstream file for localization/continued mapping
  save_state_filename : Path to save .pbstream file on shutdown
  load_frozen_state   : 'true' for localization, 'false' to continue mapping

Configuration selection:
  livox mapping           -> livox_2d.lua
  livox localization      -> localization.lua
  pointcloud2 mapping     -> pointcloud2_2d.lua
  pointcloud2 localization -> pointcloud2_localization.lua

TF Tree:
  camera_init (map_frame)
  └── odom (odom_frame)
      └── body (tracking_frame)
          ├── imu_link
          └── livox_frame

Topics:
  Input (livox mode):
    - /sensor/lidar/livox_mid360/points (livox_driver/CustomMsg)
    - /sensor/ins/livox_mid360/imu (Imu)
  Input (pointcloud2 mode):
    - /sensor/lidar/livox_mid360/points (sensor_msgs/PointCloud2)
    - /sensor/ins/livox_mid360/imu (Imu)
  Output:
    - /cartographer_2d/map (OccupancyGrid)
    - /cartographer_2d/submaps, /cartographer_2d/trajectory_nodes
    - /cartographer_2d/tracked_pose, /cartographer_2d/odom

Examples:
  # Livox CustomMsg SLAM (default)
  ros2 launch cartographer_slam slam.launch.py

  # PointCloud2 SLAM
  ros2 launch cartographer_slam slam.launch.py input_type:=pointcloud2

  # Livox Localization
  ros2 launch cartographer_slam slam.launch.py localization:=true \\
    load_state_filename:=/path/to/map.pbstream

  # PointCloud2 Localization
  ros2 launch cartographer_slam slam.launch.py input_type:=pointcloud2 \\
    localization:=true load_state_filename:=/path/to/map.pbstream
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def launch_setup(context):
    """Generate launch nodes based on runtime configuration."""
    # Get launch configuration values
    input_type = LaunchConfiguration('input_type').perform(context).lower()
    localization = LaunchConfiguration('localization').perform(context).lower() == 'true'
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    resolution = LaunchConfiguration('resolution').perform(context)
    load_state_filename = LaunchConfiguration('load_state_filename').perform(context)
    save_state_filename = LaunchConfiguration('save_state_filename').perform(context)
    load_frozen_state = LaunchConfiguration('load_frozen_state').perform(context)

    # Auto-append .pbstream extension if missing
    if load_state_filename and not load_state_filename.endswith('.pbstream'):
        load_state_filename += '.pbstream'
    if save_state_filename and not save_state_filename.endswith('.pbstream'):
        save_state_filename += '.pbstream'

    # Package paths
    pkg_share = FindPackageShare('cartographer_slam').find('cartographer_slam')
    urdf_file = os.path.join(pkg_share, 'urdf', 'livox_mid360.urdf')
    configuration_directory = os.path.join(pkg_share, 'config')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'livox_mid360.rviz')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Set environment variables for Lua config
    os.environ['CARTOGRAPHER_INPUT_TYPE'] = input_type
    os.environ['CARTOGRAPHER_LOCALIZATION'] = 'true' if localization else 'false'

    # Select topic remapping based on input_type
    use_livox = (input_type == 'livox')
    topic_remapping = ('livox_points' if use_livox else 'points2',
                       '/sensor/lidar/livox_mid360/points')

    nodes = []

    # Log configuration
    nodes.append(LogInfo(
        msg=f"[slam] input={input_type}, localization={localization}"
    ))

    # Robot state publisher
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': use_sim_time == 'true'}
        ],
        output='screen'
    ))

    # Cartographer node
    cartographer_args = [
        '-configuration_directory', configuration_directory,
        '-configuration_basename', 'slam_2d.lua',
    ]
    if load_state_filename:
        cartographer_args.extend(['-load_state_filename', load_state_filename])
        cartographer_args.extend(['-load_frozen_state', 'true' if localization else load_frozen_state])
    if save_state_filename:
        cartographer_args.extend(['-save_state_filename', save_state_filename])
        cartographer_args.extend(['-map_resolution', resolution])

    nodes.append(Node(
        package='cartographer_slam',
        executable='cartographer_node',
        name='cartographer_node',
        parameters=[{'use_sim_time': use_sim_time == 'true'}],
        arguments=cartographer_args,
        remappings=[
            topic_remapping,
            ('imu', '/sensor/ins/livox_mid360/imu'),
        ],
        output='screen'
    ))

    # Occupancy grid node
    nodes.append(Node(
        package='cartographer_slam',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        parameters=[
            {'use_sim_time': use_sim_time == 'true'},
            {'resolution': float(resolution)},
            {'publish_period_sec': 1.0}
        ],
        output='screen'
    ))

    # RViz
    if use_rviz:
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time == 'true'}],
            output='screen'
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_type', default_value='livox',
                              description='Input type: livox (CustomMsg) or pointcloud2'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Use pure localization mode (requires load_state_filename)'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation time'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Launch RViz'),
        DeclareLaunchArgument('resolution', default_value='0.05',
                              description='Occupancy grid resolution'),
        DeclareLaunchArgument('load_state_filename', default_value='',
                              description='Path to .pbstream file to load'),
        DeclareLaunchArgument('save_state_filename', default_value='',
                              description='Path to save .pbstream file on shutdown'),
        DeclareLaunchArgument('load_frozen_state', default_value='true',
                              description='Load state as frozen (true for localization, false to continue mapping)'),
        OpaqueFunction(function=launch_setup),
    ])
