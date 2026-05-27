import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    slam_config = os.path.join(
        get_package_share_directory('amr_robot'),
        'config', 'slam_config.yaml'
    )

    return LaunchDescription([

        # 1. LD19 LiDAR (includes its own static TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('ldlidar_stl_ros2'),
                '/launch/ld19.launch.py'
            ])
        ),

        # 2. Throttle scans to 10 Hz
        Node(
            package='topic_tools',
            executable='throttle',
            name='scan_throttle',
            arguments=['messages', '/scan', '2.0', '/scan_throttled'],
            output='screen'
        ),

        # 3. Arduino Bridge (publishes /odom with high covariance)
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

        # 4. SLAM Toolbox – uses throttled scan
        TimerAction(period=4.0, actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_config, {'use_sim_time': False}],
                remappings=[('/scan', '/scan_throttled')]   # ← critical
            )
        ]),

        # 5. Teleop Keyboard
        TimerAction(period=5.0, actions=[
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop',
                output='screen',
                prefix='xterm -e',
                remappings=[('/cmd_vel', '/cmd_vel')],
            )
        ]),
    ])
