"""Team 7 laptop bringup: SLAM + Nav2 + ArUco + exploration + mission + RViz.

The robot (Raspberry Pi) only runs its factory TurtleBot 4 bringup; everything
here runs on the laptop.

    ros2 launch team7_bringup team7.launch.py                        # team7 Rust explorer
    ros2 launch team7_bringup team7.launch.py explorer:=explore_lite  # baseline
    ros2 launch team7_bringup team7.launch.py explorer:=none          # map/detect only (teleop testing)

Reset odometry first so map frame == start pose (scripts/run_demo.sh does this).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ARGS = [
    ('explorer', 'team7', 'team7 | explore_lite | none'),
    ('rviz', 'true', 'Start RViz'),
    ('slam', 'true', 'Start slam_toolbox (online mapping)'),
    ('nav2', 'true', 'Start Nav2'),
    ('mission', 'true', 'Start the mission node (timer, stop, map save)'),
    ('output_dir', '~/team7_run', 'Where map.pgm/.yaml and targets.csv are written'),
    ('camera_topic', '/oakd/rgb/preview/image_raw', 'Image topic for ArUco detection'),
    ('marker_size', '0.10', 'ArUco side length in m (only used for the PnP fallback)'),
    ('time_limit', '590.0', 'Seconds until the mission stops'),
    ('expected_targets', '3', 'Stop once this many marker IDs are confirmed'),
    ('slam_sync', 'true', 'Synchronous slam_toolbox (processes every scan)'),
]


def generate_launch_description():
    bringup = FindPackageShare('team7_bringup')
    tb4_nav = FindPackageShare('turtlebot4_navigation')
    cfg = lambda name: PathJoinSubstitution([bringup, 'config', name])  # noqa: E731
    arg = LaunchConfiguration
    team7_yaml = cfg('team7.yaml')
    out_dir = {'output_dir': arg('output_dir')}

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([tb4_nav, 'launch', 'slam.launch.py'])),
        launch_arguments={'params': cfg('slam.yaml'), 'sync': arg('slam_sync')}.items(),
        condition=IfCondition(arg('slam')),
    )
    # Give slam_toolbox a moment to publish map->odom before Nav2 activates.
    nav2 = TimerAction(period=3.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([tb4_nav, 'launch', 'nav2.launch.py'])),
        launch_arguments={'params_file': cfg('nav2.yaml')}.items(),
        condition=IfCondition(arg('nav2')),
    )])

    aruco = Node(
        package='aruco_opencv', executable='aruco_tracker_autostart', name='aruco_tracker',
        parameters=[cfg('aruco.yaml'), {
            'cam_base_topic': arg('camera_topic'),
            'marker_size': ParameterValue(arg('marker_size'), value_type=float),
        }],
        output='screen',
    )

    marker_mapper = Node(
        package='team7_nodes', executable='marker_mapper', name='marker_mapper',
        parameters=[team7_yaml, out_dir], output='screen',
    )

    mission = Node(
        package='team7_nodes', executable='mission', name='mission',
        parameters=[team7_yaml, out_dir, {
            'time_limit': ParameterValue(arg('time_limit'), value_type=float),
            'expected_targets': ParameterValue(arg('expected_targets'), value_type=int),
        }],
        output='screen', condition=IfCondition(arg('mission')),
    )

    explorer_team7 = Node(
        package='team7_nodes', executable='explorer', name='explorer',
        parameters=[team7_yaml], output='screen',
        condition=LaunchConfigurationEquals('explorer', 'team7'),
    )
    explorer_baseline = Node(
        package='explore_lite', executable='explore', name='explore_node',
        parameters=[cfg('explore_lite.yaml')], output='screen',
        condition=LaunchConfigurationEquals('explorer', 'explore_lite'),
    )
    # Start exploring once Nav2 has had time to come up.
    explorers = TimerAction(period=10.0, actions=[explorer_team7, explorer_baseline])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', PathJoinSubstitution([bringup, 'rviz', 'team7.rviz'])],
        condition=IfCondition(arg('rviz')),
    )

    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in ARGS]
        + [slam, nav2, aruco, marker_mapper, mission, explorers, rviz]
    )
