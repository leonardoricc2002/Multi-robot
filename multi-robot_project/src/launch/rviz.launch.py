import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # 1. TROVIAMO IL PERCORSO DEL FILE DI CONFIGURAZIONE
    pkg_multi_robot = get_package_share_directory('multi-robot_project')
    
    # Assicurati che il file 'config.rviz' esista dentro la cartella 'launch' o 'rviz'
    # Se lo hai salvato altrove, modifica questo percorso
    rviz_config_path = os.path.join(pkg_multi_robot, 'rviz', 'config.rviz')

    # 2. GESTIONE ARGOMENTI
    # Questo argomento viene passato dal tuo 'warehouse.launch.py'
    # Se lanci questo file da solo, userà il default (true)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # 3. NODO RVIZ
    # Qui lanciamo SOLO la visualizzazione.
    # Non servono remapping o bridge perché i dati arrivano già pronti dai topic ROS
    # generati nell'altro file.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}] # <--- CRUCIALE PER LA SINCRONIZZAZIONE
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_node
    ])
