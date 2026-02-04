import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Package path (unified package)
    package_path = get_package_share_directory('fast_lio')

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
        default_value='true',
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

    ld = LaunchDescription()
    # Declare arguments
    ld.add_action(declare_map_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_lio_config_path_cmd)
    ld.add_action(declare_lio_config_file_cmd)
    # Nodes
    ld.add_action(fast_lio_node)
    ld.add_action(localization_node)
    ld.add_action(rviz_node)

    return ld
