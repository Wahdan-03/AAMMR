import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    amr_pkg        = get_package_share_directory('amr_robot')
    nav2_bringup   = get_package_share_directory('nav2_bringup')
    ldlidar_pkg    = get_package_share_directory('ldlidar_stl_ros2')

    nav2_params    = os.path.join(amr_pkg, 'config', 'nav2_params.yaml')
    rviz_config    = os.path.join(amr_pkg, 'config', 'nav2.rviz')

    # ── Launch arguments ────────────────────────────────────────────────────
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(amr_pkg, 'maps', 'Entrance.yaml'),
        description='Full path to the saved map yaml file'
    )
    map_file = LaunchConfiguration('map')

    # ── Node definitions ────────────────────────────────────────────────────

    # 1. LiDAR
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ldlidar_pkg, '/launch/ld19.launch.py'])
    )

    # 2. Scan throttle  (10 Hz is enough for AMCL + costmaps on a Pi)
    scan_throttle = Node(
        package='topic_tools',
        executable='throttle',
        name='scan_throttle',
        arguments=['messages', '/scan', '10.0', '/scan_throttled'],
        output='screen'
    )

    # 3. Static TF: base_link → base_laser
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=[
            '--x',    '0.0',
            '--y',    '0.0',
            '--z',    '1.1',
            '--yaw',  '0.0',
            '--pitch','0.0',
            '--roll', '0.0',
            '--frame-id',       'base_link',
            '--child-frame-id', 'base_laser',
        ]
    )

    # 4. Arduino bridge  (publishes /odom + odom→base_link TF every tick)
    #
    # Deadband parameters:
    #   min_wheel_speed_left  — minimum m/s that actually moves the left motor.
    #   min_wheel_speed_right — minimum m/s that actually moves the right motor.
    #
    # These are exposed here so you can tweak them without touching the node
    # source. Start with the values below (left intentionally a bit lower than
    # right to reflect your hardware) and adjust until both wheels respond
    # reliably without jerking. The right value is whatever you found in manual
    # testing; 0.55 / 0.65 are safe starting points in your 0.5–0.8 range.
    #
    # How to tune:
    #   1. Run the bridge standalone:
    #        ros2 run amr_robot arduino_bridge
    #   2. Publish a slow command and watch:
    #        ros2 topic pub /cmd_vel geometry_msgs/Twist \
    #          "{linear: {x: 0.1}, angular: {z: 0.0}}" --rate 10
    #   3. Both wheels should move immediately. If one stalls, raise its minimum.
    #   4. Then try x: 0.0, angular z: 0.3 — robot should rotate cleanly.
    arduino_bridge = Node(
        package='amr_robot',
        executable='arduino_bridge',
        name='arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port':           '/dev/arduino_mega',
            'baud_rate':             115200,
            'wheel_base':            0.165,
            'publish_rate':          20.0,
            'min_wheel_speed_left':  0.55,   # tune: lowest speed left motor moves
            'min_wheel_speed_right': 0.65,   # tune: lowest speed right motor moves
        }]
    )

    # 5. Localization stack  (map_server + AMCL under nav2_bringup lifecycle manager)
    localization_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [nav2_bringup, '/launch/localization_launch.py']
                ),
                launch_arguments={
                    'map':          map_file,
                    'use_sim_time': 'false',
                    'params_file':  nav2_params,
                }.items()
            )
        ]
    )

    # 6. Navigation stack  (planner, controller, BT navigator, behaviors)
    navigation_launch = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [nav2_bringup, '/launch/navigation_launch.py']
                ),
                launch_arguments={
                    'use_sim_time':                 'false',
                    'params_file':                  nav2_params,
                    'map_subscribe_transient_local': 'true',
                }.items()
            )
        ]
    )

    # 7. RViz2
    rviz = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen'
            )
        ]
    )

    # ── Launch description ───────────────────────────────────────────────────
    return LaunchDescription([
        map_arg,
        lidar_launch,       # t=0   LiDAR starts publishing /scan
        scan_throttle,      # t=0   /scan → /scan_throttled at 10 Hz
        base_to_laser_tf,   # t=0   base_link → base_laser TF
        arduino_bridge,     # t=0   /odom + odom→base_link TF
        localization_launch,# t=3   map_server + AMCL (managed by nav2_bringup)
        navigation_launch,  # t=7   planner + controller + BT (managed by nav2_bringup)
        rviz,               # t=12  RViz2 (map already latched by now)
    ])
