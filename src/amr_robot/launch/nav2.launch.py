import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    amr_pkg = get_package_share_directory('amr_robot')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    nav2_params = os.path.join(amr_pkg, 'config', 'nav2_params.yaml')

    # Map file argument (default to your saved map)
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(amr_pkg, 'maps', 'Synaptic_Cosmetics_Office1.yaml'),
        description='Full path to the saved map yaml file'
    )
    map_file = LaunchConfiguration('map')

    # RViz configuration file (without RobotModel)
    rviz_config = os.path.join(amr_pkg, 'config', 'nav2.rviz')

    return LaunchDescription([
        map_arg,

        # ──────────────────────────────────────────────────────
        # 1. LiDAR (using its own launch file)
        # ──────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('ldlidar_stl_ros2'),
                '/launch/ld19.launch.py'
            ])
        ),

        # ──────────────────────────────────────────────────────
        # 2. Throttle scans to 10 Hz (reduce load on AMCL / costmaps)
        # ──────────────────────────────────────────────────────
        Node(
            package='topic_tools',
            executable='throttle',
            name='scan_throttle',
            arguments=['messages', '/scan', '10.0', '/scan_throttled'],
            output='screen'
        ),

        # ──────────────────────────────────────────────────────
        # 3. Static transform: base_link → base_laser (height 1.1 m)
        # ──────────────────────────────────────────────────────
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

        # ──────────────────────────────────────────────────────
        # 4. Arduino Bridge (publishes /odom + TF every tick)
        # ──────────────────────────────────────────────────────
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

        # ──────────────────────────────────────────────────────
        # 5. Map Server (delayed 3 seconds)
        # ──────────────────────────────────────────────────────
        TimerAction(period=3.0, actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'use_sim_time': False, 'yaml_filename': map_file}]
            ),
        ]),

        # ──────────────────────────────────────────────────────
        # 6. AMCL (delayed 4 seconds, uses throttled scan)
        # ──────────────────────────────────────────────────────
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

        # ──────────────────────────────────────────────────────
        # 7. Nav2 stack (planner, controller, bt, etc.)
        #    This includes its own lifecycle manager.
        # ──────────────────────────────────────────────────────
        TimerAction(period=5.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_pkg, '/launch/navigation_launch.py']),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': nav2_params,
                    'use_lifecycle_mgr': 'true',       # let Nav2 manage its own nodes
                    'map_subscribe_transient_local': 'true'
                }.items()
            ),
        ]),

        # ──────────────────────────────────────────────────────
        # 8. Automatic lifecycle activation for map_server & amcl
        #    (Wait 8 seconds for the nodes to be fully up)
        # ──────────────────────────────────────────────────────
        TimerAction(period=8.0, actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                     'source /opt/ros/humble/setup.bash && '
                     'source /home/wahdan/ros2_ws/install/setup.bash && '
                     'ros2 lifecycle set /map_server configure && '
                     'ros2 lifecycle set /map_server activate && '
                     'ros2 lifecycle set /amcl configure && '
                     'ros2 lifecycle set /amcl activate'
                ],
                output='screen'
            )
        ]),

        # ──────────────────────────────────────────────────────
        # 9. RViz2 (delayed, uses config without RobotModel)
        #    Wait 10 seconds for the map and AMCL to be active.
        # ──────────────────────────────────────────────────────
        TimerAction(period=10.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen'
            )
        ]),
    ])
