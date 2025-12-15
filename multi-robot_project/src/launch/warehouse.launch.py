import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # --- 1. DEFINIZIONE PERCORSI ---
    pkg_multi_robot = get_package_share_directory('multi-robot_project')
    pkg_fra2mo = get_package_share_directory('ros2_fra2mo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_iiwa_bringup = get_package_share_directory('iiwa_bringup') 
    pkg_iiwa_desc = get_package_share_directory('iiwa_description')

    # File path
    fra2mo_xacro_file = os.path.join(pkg_fra2mo, "urdf", "fra2mo.urdf.xacro")
    iiwa_xacro_file = os.path.join(pkg_iiwa_desc, "urdf", "iiwa.urdf.xacro") 
    world_file = os.path.join(pkg_multi_robot, 'world', 'warehouse.world')
    slam_config_path = os.path.join(pkg_fra2mo, 'config', 'slam.yaml')

    # Oggetti di scena
    green_box_path = os.path.join(pkg_multi_robot, 'urdf','green_box.sdf')
    red_cylinder_path = os.path.join(pkg_multi_robot, 'urdf', 'red_cylinder.sdf')
    blue_sphere_path = os.path.join(pkg_multi_robot, 'urdf', 'blue_sphere.sdf')

    # --- 2. CONFIGURAZIONE AMBIENTE ---
    render_engine_env = SetEnvironmentVariable(name='IGN_RENDER_ENGINE', value='ogre')
    resource_path_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
               ':', os.path.join(pkg_fra2mo, 'models'),
               ':', os.path.join(pkg_multi_robot, 'model'),
               ':', os.path.join(pkg_iiwa_desc, '..'),
               ':', os.path.join(pkg_fra2mo, '..')]
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # --- 3. GAZEBO SIMULATION ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [world_file, ' -r']}.items(),
    )

    # ========================================================================
    #                                 ROBOT 1: IIWA (NUOVO PRIMARY)
    # ========================================================================
    # 1. Spawn/Controller IIWA (via include)
    iiwa_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_iiwa_bringup, 'launch', 'iiwa.launch.py')
        ),
        launch_arguments={
            'use_sim': 'true', 'robot_name': 'iiwa', 'x_pose': '1.0', 'y_pose': '0.0', 'z_pose': '0.0',
            # 'roll_pose': '0.0',       # CORRETTO: deve essere una stringa
             #'pitch_pose': '1.0',      # CORRETTO: deve essere una stringa
             #'yaw_pose': '0',     # CORRETTO: deve essere una stringa (ho messo 1.5708 per maggiore precisione)
             'use_sim_time': use_sim_time, 'start_controllers': 'true',
        }.items()
    )

    # 2. State Publisher DEDICATO (IIWA -> /robot_description)
    iiwa_desc_xml = Command(['xacro ', iiwa_xacro_file, ' robot_name:=iiwa', ' sim:=true'])
    iiwa_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='iiwa_rsp_primary', # Nome cambiato
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(iiwa_desc_xml, value_type=str),
            'use_sim_time': True
        }],
        remappings=[
            ('/robot_description', '/robot_description'), # <--- IIWA USA IL TOPIC STANDARD
            ('/joint_states', '/iiwa_joint_states')      # <--- Joint States dedicati
        ]
    )

    # 3. TF Statico IIWA (Posizionamento corretto in RViz)
    static_tf_iiwa = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_iiwa',
        arguments=['1.0', '0.0', '0', '0', '0', '0', 'world', 'iiwa_base'],
        parameters=[{'use_sim_time': True}]
    )
    # Nel tuo warehouse.launch.py, usa questo nodo:

    # ========================================================================
    #                                 ROBOT 2: FRA2MO (NUOVO SECONDARY)
    # ========================================================================
    robot_desc_xml = Command(['xacro ', fra2mo_xacro_file])

    # RSP Fra2mo (DEVE pubblicare su /robot_description_fra2mo)
    fra2mo_rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='fra2mo_rsp', output='screen',
        parameters=[{'robot_description': ParameterValue(robot_desc_xml, value_type=str), 'use_sim_time': True}],
        remappings=[
            ('/robot_description', '/robot_description_fra2mo'), # <--- FRA2MO USA IL TOPIC DEDICATO
        ]
    )
    
    # Spawn a -2.5
    fra2mo_spawn = Node(
        package='ros_gz_sim', executable='create',
        output='screen',
        arguments=['-string', robot_desc_xml, '-name', 'fra2mo', '-x', '-2.7', '-y', '1.5', '-z', '0.01']
    )

    # Bridge
    fra2mo_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/model/fra2mo/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/model/iiwa/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model'
        ],
        remappings=[
            ('/model/fra2mo/joint_state', '/joint_states'),
            ('/model/iiwa/joint_state', '/iiwa_joint_states'), 
        ],
        output='screen'
    )

    fra2mo_tf_pub = Node(
        package='ros2_fra2mo', executable='dynamic_tf_publisher',
        name='odom_tf', parameters=[{'use_sim_time': True}],
        remappings=[('odom', '/model/fra2mo/odometry')]
    )

    # ========================================================================
    #                                 SLAM & TF FIX
    # ========================================================================
    static_tf_map = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_world_map',
        arguments=['-2.7', '1.5', '0', '0', '0', '0', 'world', 'map'],
        parameters=[{'use_sim_time': True}]
    )

    slam_node = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        output='screen',
        parameters=[
            slam_config_path, 
            {'use_sim_time': True, 'odom_frame': 'fra2mo/odom', 'base_frame': 'base_footprint', 'map_frame': 'map', 'scan_topic': '/lidar', 'mode': 'mapping'}
        ],
    )

    # Oggetti
    spawn_objs = [
        Node(package='ros_gz_sim', executable='create', arguments=['-file', green_box_path, '-name', 'green_box', '-x', '-2.75', '-y', '1.56', '-z', '0.45']),
        Node(package='ros_gz_sim', executable='create', arguments=['-file', red_cylinder_path, '-name', 'red_cylinder', '-x', '-2.65', '-y', '1.44', '-z', '0.45']),
        Node(package='ros_gz_sim', executable='create', arguments=['-file', blue_sphere_path, '-name', 'blue_sphere', '-x', '-2.7', '-y', '1.5', '-z', '0.45'])
    ]

    # --- RETURN FINALE ---
    return LaunchDescription([
        render_engine_env, resource_path_env, gz_sim,
        
        iiwa_launch,       # Spawn in Gazebo & Controller
        iiwa_rsp,          # <--- IIWA RSP (topic standard)
        static_tf_iiwa,    # <--- TF statico (Posizionamento)
        
        fra2mo_rsp,        # <--- Fra2mo RSP (topic dedicato)
        fra2mo_spawn,
        fra2mo_bridge,
        fra2mo_tf_pub,
        
        static_tf_map,
        slam_node,
        *spawn_objs
    ])
