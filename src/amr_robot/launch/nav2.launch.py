import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    amr_pkg = get_package_share_directory('amr_robot')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    nav2_params = os.path.join(amr_pkg, 'config', 'nav2_params.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(amr_pkg, 'maps', 'my_map.yaml'),
        description='Full path to the saved map yaml file'
    )
    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        map_arg,

        # LiDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('ldlidar_stl_ros2'),
                '/launch/ld19.launch.py'
            ])
        ),

        # Throttle scans to 10 Hz
        Node(
            package='topic_tools',
            executable='throttle',
            name='scan_throttle',
            arguments=['messages', '/scan', '10.0', '/scan_throttled'],
            output='screen'
        ),

        # Static TF (only one)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=[
                '--x', '0', '--y', '0', '--z', '1.1',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'base_laser'
            ]
        ),

        # Arduino Bridge
        Node(
            package='amr_robot',
            executable='arduino_bridge',
            name='arduino_bridge',
            output='screen',
            parameters=[{
                'serial_port': '/dev/arduino_mega',
                'baud_rate': 115200,
                'wheel_base': 0.165,
                'publish_rate': 20.0,
            }]
        ),

        # Map Server (delayed)
        TimerAction(period=3.0, actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'use_sim_time': False, 'yaml_filename': map_file}]
            ),
        ]),

        # AMCL (delayed, uses throttled scan)
        TimerAction(period=4.0, actions=[
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[nav2_params],
                remappings=[('/scan', '/scan_throttled')]
            ),
        ]),

        # Nav2 stack (delayed)
        TimerAction(period=5.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_pkg, '/launch/navigation_launch.py']),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': nav2_params,
                    'use_lifecycle_mgr': 'false',
                    'map_subscribe_transient_local': 'true'
                }.items()
            ),

            # Lifecycle Manager
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'autostart': True,
                    'node_names': [
                        'map_server', 'amcl', 'controller_server',
                        'planner_server', 'behavior_server',
                        'bt_navigator', 'waypoint_follower'
                    ]
                }]
            ),
        ]),
    ])
