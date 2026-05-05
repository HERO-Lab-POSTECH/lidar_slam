"""
Livox Mid-360 Cartographer 2D SLAM unified launch file

Supports both Livox CustomMsg and PointCloud2 input types.

================================================================================
LAUNCH ARGUMENTS
================================================================================
  input_type       : 'livox' or 'pointcloud2'             (default: 'livox')
  use_localization : Enable pure localization mode        (default: 'false')
  use_sim_time     : Use simulation time                  (default: 'false')
  use_rviz         : Launch RViz                          (default: 'false')
  resolution       : Occupancy grid resolution            (default: '0.05')
  map_path         : Path to .pbstream file to load       (default: '')
  output_map_path  : Path to save .pbstream on shutdown   (default: '')
  load_frozen_state: Load map as frozen (true=localization, false=continue mapping)
                                                          (default: 'true')

================================================================================
CONFIGURATION SELECTION
================================================================================
  livox mapping            -> livox_2d.lua
  livox localization       -> localization.lua
  pointcloud2 mapping      -> pointcloud2_2d.lua
  pointcloud2 localization -> pointcloud2_localization.lua

================================================================================
TF TREE
================================================================================
  map (map_frame)
  └── odom (odom_frame)
      └── base_link (tracking_frame)
          ├── livox_frame
          ├── imu_link
          └── sonar_link

================================================================================
TOPICS
================================================================================
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

================================================================================
EXAMPLES
================================================================================
  # Livox CustomMsg SLAM (default)
  ros2 launch cartographer_slam slam.launch.py

  # PointCloud2 SLAM
  ros2 launch cartographer_slam slam.launch.py input_type:=pointcloud2

  # Livox Localization
  ros2 launch cartographer_slam slam.launch.py use_localization:=true \\
    map_path:=/path/to/map.pbstream

  # PointCloud2 Localization
  ros2 launch cartographer_slam slam.launch.py input_type:=pointcloud2 \\
    use_localization:=true map_path:=/path/to/map.pbstream

  # Continue mapping from existing map (load + save to different path)
  # NOTE: output_map_path is REQUIRED to save results.
  #       Without it, mapping runs but nothing is saved on shutdown.
  ros2 launch cartographer_slam slam.launch.py \\
    map_path:=/path/to/map_v2.pbstream \\
    load_frozen_state:=false \\
    output_map_path:=/path/to/map_v3.pbstream \\
    use_sim_time:=true
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
    use_localization = LaunchConfiguration('use_localization').perform(context).lower() == 'true'
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    resolution = LaunchConfiguration('resolution').perform(context)
    map_path = LaunchConfiguration('map_path').perform(context)
    output_map_path = LaunchConfiguration('output_map_path').perform(context)
    load_frozen_state = LaunchConfiguration('load_frozen_state').perform(context)

    # Auto-append .pbstream extension if missing
    if map_path and not map_path.endswith('.pbstream'):
        map_path += '.pbstream'
    if output_map_path and not output_map_path.endswith('.pbstream'):
        output_map_path += '.pbstream'

    # Auto-create directory for output_map_path
    if output_map_path:
        save_dir = os.path.dirname(output_map_path)
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)

    # Package paths
    pkg_share = FindPackageShare('cartographer_slam').find('cartographer_slam')
    boat_desc_share = FindPackageShare('boat_description').find('boat_description')
    urdf_file = os.path.join(boat_desc_share, 'urdf', 'boat.urdf')
    configuration_directory = os.path.join(pkg_share, 'config')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'livox_mid360.rviz')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Set environment variables for Lua config
    os.environ['CARTOGRAPHER_INPUT_TYPE'] = input_type
    os.environ['CARTOGRAPHER_LOCALIZATION'] = 'true' if use_localization else 'false'

    # Select topic remapping based on input_type
    use_livox = (input_type == 'livox')
    topic_remapping = ('livox_points' if use_livox else 'points2',
                       '/sensor/lidar/livox_mid360/points')

    nodes = []

    # Log configuration
    nodes.append(LogInfo(
        msg=f"[slam] input={input_type}, use_localization={use_localization}"
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
    if map_path:
        cartographer_args.extend(['-load_state_filename', map_path])
        cartographer_args.extend(['-load_frozen_state', 'true' if use_localization else load_frozen_state])
    if output_map_path:
        cartographer_args.extend(['-save_state_filename', output_map_path])
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
    if use_localization:
        # Localization mode: use pbstream_map_publisher for clean static map
        nodes.append(Node(
            package='cartographer_slam',
            executable='cartographer_pbstream_map_publisher',
            name='cartographer_map_publisher',
            arguments=[
                '-pbstream_filename', map_path,
                '-resolution', resolution,
            ],
            parameters=[{'use_sim_time': use_sim_time == 'true'}],
            remappings=[('map', '/cartographer_2d/map')],
            output='screen'
        ))
    else:
        # Mapping mode: use real-time occupancy grid node
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

    # Trajectory filter (localization mode only) - filters out frozen trajectory
    if use_localization:
        nodes.append(Node(
            package='cartographer_slam',
            executable='trajectory_filter_node.py',
            name='trajectory_filter_node',
            parameters=[
                {'use_sim_time': use_sim_time == 'true'},
                {'frozen_trajectory_ids': [0]},  # Filter trajectory 0 (frozen map)
            ],
            remappings=[
                ('trajectory_node_list', '/cartographer_2d/trajectory_nodes'),
                ('trajectory_node_list_filtered', '/cartographer_2d/trajectory_nodes_filtered'),
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
        DeclareLaunchArgument('use_localization', default_value='false',
                              description='Use pure localization mode (requires map_path)'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation time'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Launch RViz'),
        DeclareLaunchArgument('resolution', default_value='0.05',
                              description='Occupancy grid resolution'),
        DeclareLaunchArgument('map_path', default_value='',
                              description='Path to .pbstream file to load'),
        DeclareLaunchArgument('output_map_path', default_value='',
                              description='Path to save .pbstream file on shutdown'),
        DeclareLaunchArgument('load_frozen_state', default_value='true',
                              description='Load state as frozen (true for localization, false to continue mapping)'),
        OpaqueFunction(function=launch_setup),
    ])
