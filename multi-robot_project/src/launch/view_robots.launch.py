import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    
    # PERCORSI
    pkg_multi_robot = get_package_share_directory('multi-robot_project')
    pkg_fra2mo = get_package_share_directory('ros2_fra2mo')
    
    # XACRO
    iiwa_xacro = os.path.join(pkg_multi_robot, 'urdf', 'iiwa_standalone.urdf.xacro')
    fra2mo_xacro = os.path.join(pkg_fra2mo, 'urdf', 'fra2mo.urdf.xacro')
    rviz_config = os.path.join(pkg_multi_robot, 'rviz', 'config.rviz')

    # GENERAZIONE URDF
    robot_desc_iiwa = Command(['xacro ', iiwa_xacro])
    robot_desc_fra2mo = Command(['xacro ', fra2mo_xacro])

    # --- 1. POSIZIONAMENTO (TF STATICI) ---
    # IIWA (x=2.5) -> Ancorato a iiwa_root_link
    tf_iiwa = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_iiwa',
        arguments = ['2.5', '0', '0', '0', '0', '0', 'world', 'iiwa_root_link'],
        output='screen'
    )

    # FRA2MO (x=-2.5) -> Ancorato a base_footprint (CORRETTO!)
    tf_fra2mo = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_fra2mo',
        arguments = ['-2.5', '0', '0.15', '0', '0', '0', 'world', 'base_footprint'],
        output='screen'
    )

    # --- 2. ROBOT STATE PUBLISHERS ---
    rsp_iiwa = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='iiwa',
        output='screen',
        parameters=[{'robot_description': robot_desc_iiwa}],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
    )
    
    rsp_fra2mo = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='fra2mo',
        output='screen',
        parameters=[{'robot_description': robot_desc_fra2mo}],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
    )

    # --- 3. JOINT STATE PUBLISHERS (LA CORREZIONE È QUI) ---

    # NODO JOINT STATE PUBLISHER GUI (IIWA)
    jsp_iiwa = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace='iiwa',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{
            'robot_description': robot_desc_iiwa,
            
            # --- VALORI COPIATI DA GAZEBO ---
            'zeros': {
                'joint_a1': 0.5,
                'joint_a2': -0.7854,
                'joint_a3': 0.0,
                'joint_a4': 1.3962,
                'joint_a5': 0.0,
                'joint_a6': 0.6109,
                'joint_a7': 0.0
            }
        }]
    )

    jsp_fra2mo = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace='fra2mo',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'robot_description': robot_desc_fra2mo}] # <--- AGGIUNTO
    )

    # RVIZ
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        tf_iiwa,
        tf_fra2mo,
        rsp_iiwa,
        rsp_fra2mo,
        jsp_iiwa,
        jsp_fra2mo,
        rviz
    ])
