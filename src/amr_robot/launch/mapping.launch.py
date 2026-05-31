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

    # FIX: use package lookup instead of hardcoded /home/wahdan/... path
    ldlidar_pkg = get_package_share_directory('ldlidar_stl_ros2')

    return LaunchDescription([

        # 1. LiDAR
        # FIX: hardcoded path replaced with package lookup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ldlidar_pkg, '/launch/ld19.launch.py'])
        ),

        # 2. Arduino bridge (publishes /odom + odom->base_link TF)
        Node(
            package='amr_robot',
            executable='arduino_bridge',
            name='arduino_bridge',
            output='screen',
            parameters=[{
                'serial_port':   '/dev/arduino_mega',
                'baud_rate':     115200,
                'wheel_base':    0.165,
                'publish_rate':  20.0,
            }]
        ),

        # 3. SLAM Toolbox
        # FIX: removed the topic_tools throttle node entirely.
        #      It was broken (invalid --rate argument), ran at 1 Hz,
        #      and stacked on top of throttle_scans:5 in slam_config.yaml —
        #      together they were giving SLAM ~1 scan per 30 seconds.
        #      SLAM toolbox has its own internal scan rate control via
        #      throttle_scans and minimum_time_interval_ms in slam_config.yaml.
        #
        # FIX: removed remappings=[('/scan', '/scan_throttled')].
        #      SLAM now receives /scan directly at full 10 Hz from the LD19,
        #      and slam_config.yaml (throttle_scans: 5) controls the rate internally.
        #
        # use_odometry: false — odometry readings are unreliable on this hardware,
        # SLAM runs on scan matching only.
        TimerAction(period=5.0, actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_config, {'use_sim_time': False, 'use_odometry': False}],
            )
        ]),

        # 4. Teleop keyboard (delayed so SLAM is ready first)
        TimerAction(period=7.0, actions=[
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop',
                output='screen',
                prefix='xterm -e',
            )
        ]),

    ])
