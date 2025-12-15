# Copyright 2022 ICube Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration,
    PathJoinSubstitution, OrSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # --------------------- DECLARE ARGUMENTS ---------------------
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'runtime_config_package',
            default_value='iiwa_description',
            description='Package with the controller config.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'controllers_file',
            default_value='iiwa_controllers.yaml',
            description='YAML file with controllers config.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='iiwa_description',
            description='Package with URDF/Xacro files.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='iiwa.config.xacro',
            description='URDF/Xacro description file.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Joint prefix for multi-robot usage.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='/',
            description='Robot namespace.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Run robot in simulation'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Use fake hardware backend.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_planning',
            default_value='false',
            description='Start with MoveIt2 planning.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_servoing',
            default_value='false',
            description='Start with MoveIt2 servoing.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_controller',
            default_value='iiwa_arm_controller',
            description='Default robot controller.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Launch RViz2.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.170.10.2',
            description='FRI robot IP.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_port',
            default_value='30200',
            description='FRI robot port.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'initial_positions_file',
            default_value='initial_positions.yaml',
            description='Initial joints positions file.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'command_interface',
            default_value='position',
            description='Controller interface'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'base_frame_file',
            default_value='base_frame.yaml',
            description='Base frame config file.'
        )
    )
    
    # ------------------ INITIALIZE ARGUMENTS ------------------
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    controllers_file = LaunchConfiguration('controllers_file')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    prefix = LaunchConfiguration('prefix')
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_planning = LaunchConfiguration('use_planning')
    use_servoing = LaunchConfiguration('use_servoing')
    robot_controller = LaunchConfiguration('robot_controller')
    start_rviz = LaunchConfiguration('start_rviz')
    robot_ip = LaunchConfiguration('robot_ip')
    robot_port = LaunchConfiguration('robot_port')
    initial_positions_file = LaunchConfiguration('initial_positions_file')
    command_interface = LaunchConfiguration('command_interface')
    base_frame_file = LaunchConfiguration('base_frame_file')
    namespace = LaunchConfiguration('namespace')
    #x_pose = LaunchConfiguration('x_pose')
    #y_pose = LaunchConfiguration('y_pose')
    #z_pose = LaunchConfiguration('z_pose')
    #roll_pose = LaunchConfiguration('roll_pose')
    #pitch_pose = LaunchConfiguration('pitch_pose')
    #yaw_pose = LaunchConfiguration('yaw_pose')

    # ------------------ ROBOT DESCRIPTION ------------------
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare(description_package), 'config', description_file]),
        ' ',
        'prefix:=', prefix, ' ',
        'use_sim:=', use_sim, ' ',
        'use_fake_hardware:=', use_fake_hardware, ' ',
        'robot_ip:=', robot_ip, ' ',
        'robot_port:=', robot_port, ' ',
        'initial_positions_file:=', initial_positions_file, ' ',
        'command_interface:=', command_interface, ' ',
        'base_frame_file:=', base_frame_file, ' ',
        'description_package:=', description_package, ' ',
        'runtime_config_package:=', runtime_config_package, ' ',
        'controllers_file:=', controllers_file, ' ',
        'namespace:=', namespace
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # ------------------ PLANNING/SERVOING ------------------
    iiwa_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('iiwa_bringup'), 'launch', 'iiwa_planning.launch.py'])
        ]),
        launch_arguments={
            'description_package': description_package,
            'description_file': description_file,
            'prefix': prefix,
            'start_rviz': start_rviz,
            'base_frame_file': base_frame_file,
            'namespace': namespace,
            'use_sim': use_sim,
        }.items(),
        condition=IfCondition(use_planning),
    )

    iiwa_servoing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('iiwa_bringup'), 'launch', 'iiwa_servoing.launch.py'])
        ]),
        launch_arguments={
            'description_package': description_package,
            'description_file': description_file,
            'prefix': prefix,
            'base_frame_file': base_frame_file,
            'namespace': namespace,
        }.items(),
        condition=IfCondition(use_servoing),
    )

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'config', controllers_file]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), 'rviz', 'iiwa.rviz']
    )

    # ------------------ NODES ------------------
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
        namespace=namespace,
        condition=UnlessCondition(use_sim),
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[robot_description],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[robot_description],
        condition=UnlessCondition(OrSubstitution(use_planning, use_sim)),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
	executable='create',
	output='screen',
	arguments=[
	    '-topic', 'robot_description',
	    '-name', 'iiwa',
	    '-allow_renaming', 'true',
           # '-x', x_pose,
	    #'-y', y_pose,
	    #'-z', z_pose,
           # '-R', roll_pose,   # AGGIUNTO TRATTINO E MAIUSCOLA
            #'-P', pitch_pose,  # AGGIUNTO TRATTINO E MAIUSCOLA
            #'-Y', yaw_pose,    # AGGIUNTO TRATTINO E MAIUSCOLA
	],
	  condition=IfCondition(use_sim),
     )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', [namespace, 'controller_manager'],
            '--controller-manager-timeout', '10'
        ]
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            robot_controller,
            '--controller-manager', [namespace, 'controller_manager'],
            '--controller-manager-timeout', '10'
        ]
    )

    external_torque_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ets_state_broadcaster', '--controller-manager', [namespace, 'controller_manager']],
        condition=UnlessCondition(use_sim),
    )

    # ------------------ ARUCO NODE ------------------
    aruco_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_detect',
        output='screen',
        remappings=[
            ('/image', '/camera/color/image_raw'),
            ('/camera_info', '/camera/color/camera_info'),
        ],
        parameters=[{
            'camera_topic': '/camera/color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'marker_size': 0.5,
            'marker_id': 201,
            'camera_frame': 'camera_link',
            'marker_frame': 'aruco_marker_static_instance/base'
        }],
        condition=IfCondition(use_sim)
    )

    # ------------------ CAMERA BRIDGE ------------------
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen',
        condition=IfCondition(use_sim),
    )

    # ------------------ STATIC TF FOR ARUCO ------------------
    aruco_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '-1.2', '-0.9', '0.55',
            '-0.4', '1.5708', '0',
            'iiwa_base',
            'aruco_marker_static_instance/base'
        ],
        condition=IfCondition(use_sim),
    )

    # ------------------ ARUCO SET POSE SERVICE BRIDGE ------------------
    aruco_set_pose_service_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='aruco_set_pose_service_bridge',
        arguments=[
            '/world/workshop_world/set_pose@ros_gz_interfaces/srv/SetEntityPose'
        ],
        output='screen',
        condition=IfCondition(use_sim),
    )

    # ------------------ EVENT HANDLERS ------------------
    delay_joint_state_broadcaster_spawner_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        ),
        condition=IfCondition(use_sim),
    )

    delay_joint_state_broadcaster_spawner_after_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[joint_state_broadcaster_spawner],
        ),
        condition=UnlessCondition(use_sim),
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(start_rviz),
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

#    delay_aruco_after_controller_spawner = RegisterEventHandler(
 #       event_handler=OnProcessExit(
  #          target_action=robot_controller_spawner,
   #         on_exit=[aruco_node],
    #    ),
     #   condition=IfCondition(use_sim),
 #   )

    # ------------------ RETURN LAUNCH DESCRIPTION ------------------
    nodes = [
        control_node,
        iiwa_planning_launch,
        iiwa_servoing_launch,
        #aruco_tf_publisher,
        spawn_entity,
        #aruco_set_pose_service_bridge,
        robot_state_pub_node,
        camera_bridge,
        delay_joint_state_broadcaster_spawner_after_control_node,
        delay_joint_state_broadcaster_spawner_after_spawn_entity,
        #delay_rviz_after_joint_state_broadcaster_spawner,
        external_torque_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        #delay_aruco_after_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)

