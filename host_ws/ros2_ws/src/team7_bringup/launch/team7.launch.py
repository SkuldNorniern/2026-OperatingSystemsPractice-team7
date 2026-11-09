import os
from datetime import datetime
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

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
    ('slam_sync', 'false', 'Synchronous SLAM; false processes latest scans without a backlog'),
    ('record', 'true', 'Record sensors, TF, navigation actions and logs to a rosbag'),
    ('use_sim_time', 'false', 'true when running against the Gazebo simulation'),
]

def start_recording(context):
    output = Path(os.path.expanduser(LaunchConfiguration('output_dir').perform(context)))
    output.mkdir(parents=True, exist_ok=True)
    bag = output / ('rosbag_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    share = Path(get_package_share_directory('team7_bringup'))
    topics = (share / 'config' / 'recording_topics.regex').read_text().strip()
    command = ['ros2', 'bag', 'record', '-o', str(bag), '-s', 'mcap',
               '--include-hidden-topics', '--regex', topics,
               '--qos-profile-overrides-path',
               PathJoinSubstitution([FindPackageShare('team7_bringup'), 'config', 'recording_qos.yaml']),
               '--max-bag-size', '1073741824', '--max-cache-size', '67108864',
               '--disable-keyboard-controls']
    if LaunchConfiguration('use_sim_time').perform(context).lower() == 'true':
        command += ['--use-sim-time']
    return [LogInfo(msg=['Recording: ', str(bag)]),
            ExecuteProcess(cmd=command, output='screen', sigterm_timeout='15', sigkill_timeout='15')]

def native_library_environment():
    """Keep ROS libraries, but use system libraries for Ubuntu native nodes."""
    environment = {}
    prefix = ''
    graphics_vars = ('LD_LIBRARY_PATH', 'LIBGL_DRIVERS_PATH', '__EGL_VENDOR_LIBRARY_FILENAMES')
    if any('/nix/store/' in os.environ.get(key, '') for key in graphics_vars):
        environment['LD_LIBRARY_PATH'] = ':'.join(
            path for path in os.environ.get('LD_LIBRARY_PATH', '').split(':')
            if path and not path.startswith('/nix/store/')
        )
        prefix = 'env -u LIBGL_DRIVERS_PATH -u __EGL_VENDOR_LIBRARY_FILENAMES'
    if os.environ.get('WAYLAND_DISPLAY'):
        environment['QT_QPA_PLATFORM'] = 'xcb'
    return environment, prefix

def generate_launch_description():
    bringup = FindPackageShare('team7_bringup')
    tb4_nav = FindPackageShare('turtlebot4_navigation')
    cfg = lambda name: PathJoinSubstitution([bringup, 'config', name])  # noqa: E731
    arg = LaunchConfiguration
    team7_yaml = cfg('team7.yaml')
    out_dir = {'output_dir': arg('output_dir')}

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([tb4_nav, 'launch', 'slam.launch.py'])),
        launch_arguments={'params': cfg('slam.yaml'), 'sync': arg('slam_sync'),
                          'use_sim_time': arg('use_sim_time')}.items(),
        condition=IfCondition(arg('slam')),
    )
    nav2_params = RewrittenYaml(source_file=cfg('nav2.yaml'),
        param_rewrites={'default_nav_to_pose_bt_xml': cfg('navigate_forward.xml')}, convert_types=True)
    # Launch only the servers used for frontier navigation. The upstream Jazzy
    # launch also starts route, waypoint and docking servers and waits for all
    # of them before activating navigation.
    servers = [
        ('nav2_controller', 'controller_server'),
        ('nav2_smoother', 'smoother_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_velocity_smoother', 'velocity_smoother'),
        ('nav2_collision_monitor', 'collision_monitor'),
        ('nav2_bt_navigator', 'bt_navigator'),
    ]
    nav_nodes = []
    for package, executable in servers:
        remaps = [('/global_costmap/scan', '/scan'), ('/local_costmap/scan', '/scan')]
        if executable in ('controller_server', 'behavior_server', 'velocity_smoother'):
            remaps.append(('cmd_vel', 'cmd_vel_nav'))
        nav_nodes.append(Node(package=package, executable=executable, name=executable,
            parameters=[nav2_params], remappings=remaps, output='screen'))
    nav_nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'autostart': True, 'node_names': [name for _, name in servers],
                     'bond_timeout': 10.0}]))
    nav2 = TimerAction(period=3.0, actions=[GroupAction(actions=nav_nodes,
        condition=IfCondition(arg('nav2')))])

    native_env, native_prefix = native_library_environment()
    aruco = Node(
        package='aruco_opencv', executable='aruco_tracker_autostart', name='aruco_tracker',
        parameters=[cfg('aruco.yaml'), {
            'cam_base_topic': arg('camera_topic'),
            'marker_size': ParameterValue(arg('marker_size'), value_type=float),
        }],
        output='screen', additional_env=native_env, prefix=native_prefix,
    )

    marker_mapper = Node(
        package='team7_nodes', executable='marker_mapper', name='marker_mapper',
        parameters=[team7_yaml, out_dir], output='screen',
    )

    velocity_guard = Node(
        package='team7_nodes', executable='velocity_guard', name='velocity_guard',
        parameters=[team7_yaml], output='screen', condition=IfCondition(arg('nav2')),
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
    explorers = TimerAction(period=10.0, actions=[explorer_team7, explorer_baseline])

    # Ubuntu RViz/Ogre uses GLX. Inherited Nix graphics libraries can prevent
    # it from creating a window under XWayland; keep the ROS library paths.
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', PathJoinSubstitution([bringup, 'rviz', 'team7.rviz'])],
        additional_env=native_env, prefix=native_prefix,
        condition=IfCondition(arg('rviz')),
    )

    sim_time = SetParameter(name='use_sim_time',
                            value=ParameterValue(arg('use_sim_time'), value_type=bool))

    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in ARGS]
        + [sim_time, OpaqueFunction(function=start_recording, condition=IfCondition(arg('record'))),
           velocity_guard, slam, nav2, aruco, marker_mapper, mission, explorers, rviz]
    )
