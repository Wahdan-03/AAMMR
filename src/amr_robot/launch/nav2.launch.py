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
        # FIX #4: default changed to my_map (real SLAM map, not the tiny GIMP image)
        default_value=os.path.join(amr_pkg, 'maps', 'my_map.yaml'),
        description='Full path to the saved map yaml file'
    )
    map_file = LaunchConfiguration('map')

    # ── Node definitions ────────────────────────────────────────────────────

    # 1. LiDAR
    # FIX #6: package lookup instead of hardcoded path (same style as nav2_bringup)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ldlidar_pkg, '/launch/ld19.launch.py'])
    )

    # 2. Scan throttle  (10 Hz is enough for AMCL + costmaps on a Pi)
    # FIX #7: no trailing '--rate' argument
    scan_throttle = Node(
        package='topic_tools',
        executable='throttle',
        name='scan_throttle',
        arguments=['messages', '/scan', '10.0', '/scan_throttled'],
        output='screen'
    )

    # 3. Static TF: base_link → base_laser
    # FIX #3: child-frame-id MUST match the frame_id that ld19.launch.py stamps
    #         on /scan messages.  LD19 defaults to 'base_laser'.
    #         If yours differs, run:  ros2 topic echo /scan --once | grep frame_id
    #         and change --child-frame-id below to match.
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
            '--child-frame-id', 'base_laser',   # ← verify this matches /scan.header.frame_id
        ]
    )

    # 4. Arduino bridge  (publishes /odom + odom→base_link TF every tick)
    arduino_bridge = Node(
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
    )

    # 5. Localization stack  (map_server + AMCL under nav2_bringup lifecycle manager)
    # FIX #2: replaced the manually managed map_server + amcl nodes + manual
    #         "ros2 lifecycle set" ExecuteProcess with nav2_bringup's
    #         localization_launch.py, which owns the lifecycle for both nodes
    #         correctly and avoids the double-lifecycle-manager conflict.
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
    # FIX #5: removed non-existent 'use_lifecycle_mgr' argument
    #         navigation_launch.py always starts its own lifecycle manager.
    #         map_subscribe_transient_local is kept — it tells the global
    #         costmap's StaticLayer to use Transient Local QoS when subscribing
    #         to /map, which is correct.
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
    # FIX #1: RViz is now started after the localization lifecycle manager has
    #         had time to activate map_server (~3 s launch + ~3 s to activate =
    #         ~6 s).  We give it 12 s to be safe.
    #         The actual QoS fix is in nav2.rviz (Durability: Transient Local).
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
