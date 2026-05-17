"""Fixed phase-4 Nav2 Ackermann navigation launch entry point."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def launch_bool(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    return value in ('1', 'true', 'yes', 'on')


def validate_stage4_required_inputs(context, *args, **kwargs):
    if launch_bool(context, 'start_chassis'):
        value = LaunchConfiguration('counts_per_meter').perform(context).strip()
        try:
            counts_per_meter = float(value)
        except ValueError as exc:
            raise RuntimeError('stage4 counts_per_meter must be a measured positive number') from exc

        if counts_per_meter <= 0.0:
            raise RuntimeError('stage4 counts_per_meter must be > 0 when start_chassis is true')

    if launch_bool(context, 'start_nav2'):
        map_file = LaunchConfiguration('map').perform(context).strip()
        if not map_file:
            raise RuntimeError('stage4 map must be set to a saved Stage-3 map when start_nav2 is true')

        if not Path(map_file).expanduser().is_file():
            raise RuntimeError(f'stage4 map file does not exist: {map_file}')

    return []


def pointcloud_to_laserscan_node():
    return Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/point_cloud_raw'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': 0.1,
            'max_height': 1.5,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00436,
            'scan_time': 0.05,
            'range_min': 0.2,
            'range_max': 20.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
    )


def nav2_started_when(use_composition):
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration('start_nav2'), "'.lower() == 'true' and '",
        use_composition, "'.lower() == 'true'",
    ]))


def nav2_not_composed_when(use_composition):
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration('start_nav2'), "'.lower() == 'true' and '",
        use_composition, "'.lower() != 'true'",
    ]))


def generate_launch_description():
    autoracer_nav_dir = Path(get_package_share_directory('autoracer_robot_nav2'))
    stage1_launch = Path(get_package_share_directory('autoracer_bringup')) / 'launch' / 'stage1_chassis.launch.py'
    robot_description_launch = Path(get_package_share_directory('autoracer_robot_urdf')) / 'launch' / 'robot_description.launch.py'

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    nav2_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    navigation_lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]
    configured_nav2_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    nav2_container = Node(
        condition=nav2_started_when(use_composition),
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[params_file, {'autostart': autostart}],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'),
            'launch',
            'localization_launch.py',
        ])),
        condition=IfCondition(LaunchConfiguration('start_nav2')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'container_name': 'nav2_container',
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'),
            'launch',
            'navigation_launch.py',
        ])),
        condition=nav2_started_when(use_composition),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'container_name': 'nav2_container',
        }.items(),
    )

    navigation_nodes = GroupAction(
        condition=nav2_not_composed_when(use_composition),
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=nav2_remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=nav2_remappings,
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=nav2_remappings,
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=nav2_remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=nav2_remappings,
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=nav2_remappings,
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=nav2_remappings + [
                    ('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', '/nav2_cmd_vel'),
                ],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': navigation_lifecycle_nodes,
                }],
            ),
        ],
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        condition=IfCondition(LaunchConfiguration('start_nav2')),
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
    )

    collision_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        condition=IfCondition(LaunchConfiguration('start_nav2')),
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['collision_monitor'],
        }],
    )

    adapter = Node(
        package='autoracer_robot_nav2',
        executable='twist_to_ackermann',
        name='twist_to_ackermann',
        output='screen',
        parameters=[{
            'input_topic': '/safe_nav2_cmd_vel',
            'output_topic': '/ackermann_cmd',
            'diagnostics_topic': '/twist_to_ackermann/diagnostics',
            'wheelbase_m': ParameterValue(LaunchConfiguration('wheelbase_m'), value_type=float),
            'max_steering_angle_rad': ParameterValue(LaunchConfiguration('max_steering_angle_rad'), value_type=float),
            'max_auto_speed_mps': ParameterValue(LaunchConfiguration('max_auto_speed_mps'), value_type=float),
            'max_reverse_speed_mps': ParameterValue(LaunchConfiguration('max_reverse_speed_mps'), value_type=float),
            'min_turn_speed_mps': ParameterValue(LaunchConfiguration('min_turn_speed_mps'), value_type=float),
            'angular_deadband_radps': ParameterValue(LaunchConfiguration('angular_deadband_radps'), value_type=float),
            'allow_reverse': ParameterValue(LaunchConfiguration('allow_reverse'), value_type=bool),
            'input_timeout_sec': ParameterValue(LaunchConfiguration('input_timeout_sec'), value_type=float),
            'brake_on_stop': ParameterValue(LaunchConfiguration('brake_on_stop'), value_type=bool),
            'enable_on_command': ParameterValue(LaunchConfiguration('enable_on_command'), value_type=bool),
            'publish_timeout_stop': ParameterValue(LaunchConfiguration('publish_timeout_stop'), value_type=bool),
            'diagnostics_rate_hz': ParameterValue(LaunchConfiguration('diagnostics_rate_hz'), value_type=float),
        }],
    )

    rviz = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2_stage4_nav',
        arguments=['-d', os.path.join(str(autoracer_nav_dir), 'rviz', 'nav2.rviz')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='', description='Saved Stage-3 map yaml file; required when start_nav2 is true'),
        DeclareLaunchArgument('params_file', default_value=str(autoracer_nav_dir / 'param' / 'stage4_nav2_params.yaml'), description='Stage-4 Nav2 Ackermann params'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('usart_port_name', default_value='/dev/ttyACM0', description='Serial port for STM32 UART4 bridge'),
        DeclareLaunchArgument('serial_baud_rate', default_value='115200', description='Serial baud rate'),
        DeclareLaunchArgument('counts_per_meter', default_value='', description='Measured Hall counts per meter; required when start_chassis is true'),
        DeclareLaunchArgument('use_ekf', default_value='true', description='Start EKF for canonical /odom when /imu/data is available'),
        DeclareLaunchArgument('start_imu', default_value='true', description='Start IMU raw driver and Madgwick /imu/data filter'),
        DeclareLaunchArgument('start_robot_description', default_value='true', description='Start URDF robot_state_publisher for base_link to sensor TF'),
        DeclareLaunchArgument('start_chassis', default_value='true', description='Start phase-1 Ackermann chassis chain'),
        DeclareLaunchArgument('start_lidar', default_value='true', description='Start LiDAR driver for /point_cloud_raw'),
        DeclareLaunchArgument('start_nav2', default_value='true', description='Start Nav2 and Collision Monitor; set false only for local dry launch parsing'),
        DeclareLaunchArgument('wheelbase_m', default_value='0.60'),
        DeclareLaunchArgument('max_steering_angle_rad', default_value='0.262'),
        DeclareLaunchArgument('max_auto_speed_mps', default_value='1.00'),
        DeclareLaunchArgument('max_reverse_speed_mps', default_value='0.60'),
        DeclareLaunchArgument('min_turn_speed_mps', default_value='0.05'),
        DeclareLaunchArgument('angular_deadband_radps', default_value='0.02'),
        DeclareLaunchArgument('allow_reverse', default_value='false'),
        DeclareLaunchArgument('input_timeout_sec', default_value='0.50'),
        DeclareLaunchArgument('brake_on_stop', default_value='true'),
        DeclareLaunchArgument('enable_on_command', default_value='true'),
        DeclareLaunchArgument('publish_timeout_stop', default_value='true'),
        DeclareLaunchArgument('diagnostics_rate_hz', default_value='10.0'),
        OpaqueFunction(function=validate_stage4_required_inputs),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('hipnuc_imu'),
                'launch',
                'imu_spec_msg.launch.py',
            ])),
            condition=IfCondition(LaunchConfiguration('start_imu')),
        ),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_imu')),
            parameters=[PathJoinSubstitution([
                FindPackageShare('turn_on_autoracer_robot'),
                'config',
                'imu.yaml',
            ])],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(robot_description_launch)),
            condition=IfCondition(LaunchConfiguration('start_robot_description')),
        ),
        GroupAction([
            SetRemap(src='cmd_vel', dst='/stage4_legacy_cmd_vel_disabled'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(stage1_launch)),
                condition=IfCondition(LaunchConfiguration('start_chassis')),
                launch_arguments={
                    'usart_port_name': LaunchConfiguration('usart_port_name'),
                    'serial_baud_rate': LaunchConfiguration('serial_baud_rate'),
                    'counts_per_meter': LaunchConfiguration('counts_per_meter'),
                    'use_ekf': LaunchConfiguration('use_ekf'),
                }.items(),
            ),
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('lslidar_driver'),
                'launch',
                'lslidar_cx_launch.py',
            ])),
            condition=IfCondition(LaunchConfiguration('start_lidar')),
        ),
        pointcloud_to_laserscan_node(),
        GroupAction([
            SetRemap(src='cmd_vel', dst='/nav2_cmd_vel'),
            nav2_container,
            localization_launch,
            navigation_launch,
        ]),
        navigation_nodes,
        collision_monitor,
        collision_lifecycle_manager,
        adapter,
        rviz,
    ])
