import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    fra2mo_dir = FindPackageShare('ros2_fra2mo')
    nav2_bringup_launch_file_dir = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )
    rviz_config_file = os.path.join(get_package_share_directory('ros2_fra2mo'), 'rviz_conf', 'navigation.rviz')

    # Argomenti esistenti
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # NUOVI ARGOMENTI PER DISATTIVARE MAP SERVER E AMCL nel bringup_launch.py
    use_map_server = LaunchConfiguration('use_map_server', default='false')
    use_amcl = LaunchConfiguration('use_amcl', default='false')


    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([fra2mo_dir, 'maps', 'map.yaml']),
        description='Full path to map yaml file to load',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([fra2mo_dir, 'config', 'navigation.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'
    )
    
    # Dichiarazioni per i nuovi flag
    declare_use_map_server_cmd = DeclareLaunchArgument(
        'use_map_server', default_value='false', description='Whether to start map server'
    )
    
    declare_use_amcl_cmd = DeclareLaunchArgument(
        'use_amcl', default_value='false', description='Whether to start AMCL'
    )

    # --- INCLUSIONE NAV2 BRINGUP (con disattivazione) ---
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_launch_file_dir]),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'use_map_server': use_map_server, # <--- DISATTIVATO
            'use_amcl': use_amcl,             # <--- DISATTIVATO
        }.items(),
    )
    
    # Nodo RViz2 (rimane invariato)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription(
        [
            declare_map_yaml_cmd,
            declare_params_file_cmd,
            declare_use_sim_time_cmd,
            declare_use_map_server_cmd, # Aggiunti alla lista di ritorno
            declare_use_amcl_cmd,       # Aggiunti alla lista di ritorno
            
            nav2_bringup_launch,
            rviz_node,
        ]
    )
