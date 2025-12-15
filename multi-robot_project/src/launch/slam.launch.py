import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # --- DEFINIZIONE NOMI PACCHETTI ---
    pkg_fra2mo = 'ros2_fra2mo'
    pkg_project = 'multi-robot_project'

    # --- PERCORSI DEI FILE ---
    # 1. Il file .yaml con i parametri (odom_frame, ecc.)
    # Deve trovarsi in: src/ros2_fra2mo/config/slam.yaml
    slam_config_path = PathJoinSubstitution(
        [FindPackageShare(pkg_fra2mo), 'config', 'slam.yaml']
    )

    # 2. Il file di configurazione di RViz
    # Deve trovarsi in: src/multi-robot_project/rviz/config.rviz
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare(pkg_project), 'rviz', 'slam.rviz']
    )

    # --- CONFIGURAZIONE ARGOMENTI ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # Argomento per Sim Time
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true', 
        description='Use simulation/Gazebo clock'
    )

    # Argomento per Parametri SLAM
    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_config_path,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
    )

    # Argomento per RViz
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=rviz_config_path,
        description='Full path to the RViz config file to use'
    )

    # --- NODI ---
    
    # Nodo SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file, 
            {'use_sim_time': use_sim_time}
        ],
    )
    
    # Nodo RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params,
        declare_rviz_config,
        slam_node,
        rviz_node
    ])
