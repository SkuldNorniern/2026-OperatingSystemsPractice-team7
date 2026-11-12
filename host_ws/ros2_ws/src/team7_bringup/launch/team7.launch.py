import os
from datetime import datetime
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

ARGS = [
    ('explorer', 'team7', 'team7 | explore_lite | none'),
    ('nav', 'nav2', 'nav2 (NavFn + RPP) | lazy_nav2 (our Lazy Theta* + Nav2 RPP) | team7 (Lazy Theta* + our MPPI, no Nav2)'),
    ('aruco', 'opencv', 'opencv (aruco_opencv) | team7 (our detector)'),
    ('slam_backend', 'toolbox', 'toolbox (slam_toolbox) | team7 (our SLAM)'),
    ('scan_filter', 'true', 'filter + deskew /scan into /scan_filtered for SLAM'),
    ('rviz', 'true', 'Start RViz'),
    ('slam', 'true', 'Start SLAM'),
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

# Nav2 servers used for frontier navigation. The upstream Jazzy launch also starts
# route, waypoint and docking servers and waits for all of them before activating.
NAV2_SERVERS = [
    ('nav2_controller', 'controller_server'),
    ('nav2_smoother', 'smoother_server'),
    ('nav2_planner', 'planner_server'),
    ('nav2_behaviors', 'behavior_server'),
    ('nav2_velocity_smoother', 'velocity_smoother'),
    ('nav2_collision_monitor', 'collision_monitor'),
    ('nav2_bt_navigator', 'bt_navigator'),
]

def recording(context):
    output = Path(os.path.expanduser(LaunchConfiguration('output_dir').perform(context)))
    output.mkdir(parents=True, exist_ok=True)
    bag = output / ('rosbag_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    share = Path(get_package_share_directory('team7_bringup'))
    topics = (share / 'config' / 'recording_topics.regex').read_text().strip()
    command = ['ros2', 'bag', 'record', '-o', str(bag), '-s', 'mcap',
               '--include-hidden-topics', '--regex', topics,
               '--qos-profile-overrides-path', str(share / 'config' / 'recording_qos.yaml'),
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

TEAM7_ONLY_DISPLAYS = ['MPPI local cost (team7)', 'Frontiers (team7)', 'Lazy Theta* path (team7)',
                       'MPPI trajectory (team7)', 'SLAM graph (team7)', 'ArUco debug (team7)']
STOCK_ONLY_DISPLAYS = ['Nav2 plan', 'Frontiers (explore_lite)', 'ArUco debug']

def rviz_config_for(nav, aruco_be):
    """Base RViz config: stock.rviz for nav2, team7.rviz otherwise. No per-run generation."""
    base = 'stock.rviz' if nav == 'nav2' else 'team7.rviz'
    return os.path.join(get_package_share_directory('team7_bringup'), 'rviz', base)

def setup(context):
    bringup = FindPackageShare('team7_bringup')
    tb4_nav = FindPackageShare('turtlebot4_navigation')
    cfg = lambda name: PathJoinSubstitution([bringup, 'config', name])  # noqa: E731
    arg = LaunchConfiguration
    val = lambda name: context.perform_substitution(arg(name))  # noqa: E731
    on = lambda name: val(name).lower() in ('true', '1', 'yes')  # noqa: E731
    team7_yaml = cfg('team7.yaml')
    out_dir = {'output_dir': arg('output_dir')}
    nav, aruco_be, slam_be, explorer = val('nav'), val('aruco'), val('slam_backend'), val('explorer')
    actions = [LogInfo(msg=f'team7: nav={nav} aruco={aruco_be} slam={slam_be} explorer={explorer}')]
    native_env, native_prefix = native_library_environment()
    if on('record'):
        actions += recording(context)

    use_nav2 = nav in ('nav2', 'lazy_nav2')
    nav_action = '/navigate_to_pose' if nav == 'nav2' else '/team7/navigate_to_pose'
    if explorer == 'explore_lite' and nav != 'nav2':
        actions.append(LogInfo(msg='team7: explore_lite talks to Nav2 directly, our navigator is skipped by it'))

    scan_topic = '/scan_filtered' if on('scan_filter') else '/scan'
    if on('scan_filter'):
        actions.append(Node(package='team7_nodes', executable='scan_filter', name='scan_filter',
                            parameters=[team7_yaml], output='screen'))

    if on('slam') and slam_be == 'team7':
        actions.append(Node(package='team7_nodes', executable='slam', name='slam',
                            parameters=[team7_yaml, {'scan_topic': scan_topic}], output='screen'))
    elif on('slam'):
        slam_params = RewrittenYaml(source_file=cfg('slam.yaml'), root_key='',
                                    param_rewrites={'scan_topic': scan_topic}, convert_types=True)
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([tb4_nav, 'launch', 'slam.launch.py'])),
            launch_arguments={'params': slam_params, 'sync': arg('slam_sync'),
                              'use_sim_time': arg('use_sim_time')}.items()))

    # every velocity source ends in velocity_guard: forward only, create3 limits, stops on stale commands
    if nav in ('nav2', 'lazy_nav2', 'team7'):
        actions.append(Node(package='team7_nodes', executable='velocity_guard', name='velocity_guard',
                            parameters=[team7_yaml], output='screen'))
    if use_nav2:
        nav2_params = RewrittenYaml(source_file=cfg('nav2.yaml'),
            param_rewrites={'default_nav_to_pose_bt_xml': cfg('navigate_forward.xml')}, convert_types=True)
        nav_nodes = []
        for package, executable in NAV2_SERVERS:
            remaps = [('/global_costmap/scan', '/scan'), ('/local_costmap/scan', '/scan')]
            if executable in ('controller_server', 'behavior_server', 'velocity_smoother'):
                remaps.append(('cmd_vel', 'cmd_vel_nav'))
            nav_nodes.append(Node(package=package, executable=executable, name=executable,
                                  parameters=[nav2_params], remappings=remaps, output='screen'))
        nav_nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                              name='lifecycle_manager_navigation', output='screen',
                              parameters=[{'autostart': True, 'node_names': [n for _, n in NAV2_SERVERS],
                                           'bond_timeout': 10.0}]))
        actions.append(TimerAction(period=3.0, actions=nav_nodes))
    if nav in ('lazy_nav2', 'team7'):
        actions.append(Node(package='team7_nodes', executable='navigator', name='navigator',
                            parameters=[team7_yaml, {'mode': 'nav2' if nav == 'lazy_nav2' else 'mppi',
                                                     'cmd_vel_topic': '/cmd_vel_nav'}],
                            output='screen'))
    if nav == 'team7':
        # our MPPI goes through same safety chain as Nav2 controller:
        # cmd_vel_nav -> smoother -> collision_monitor -> cmd_vel_filtered -> velocity_guard -> cmd_vel
        nav2_params = cfg('nav2.yaml')
        sim = {'use_sim_time': ParameterValue(arg('use_sim_time'), value_type=bool)}
        actions += [
            Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
                 parameters=[nav2_params, sim], remappings=[('cmd_vel', 'cmd_vel_nav')], output='screen'),
            Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
                 parameters=[cfg('collision_monitor_team7.yaml'), sim], output='screen'),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_safety',
                 parameters=[sim, {'autostart': True, 'node_names': ['velocity_smoother', 'collision_monitor']}],
                 output='screen'),
        ]

    if aruco_be == 'team7':
        actions.append(Node(package='team7_nodes', executable='aruco_detector', name='aruco_detector',
                            parameters=[team7_yaml, {
                                'image_topic': arg('camera_topic'),
                                'marker_size': ParameterValue(arg('marker_size'), value_type=float)}],
                            output='screen'))
    else:
        actions.append(Node(package='aruco_opencv', executable='aruco_tracker_autostart', name='aruco_tracker',
                            parameters=[cfg('aruco.yaml'), {
                                'cam_base_topic': arg('camera_topic'),
                                'marker_size': ParameterValue(arg('marker_size'), value_type=float)}],
                            output='screen', additional_env=native_env, prefix=native_prefix))

    # slam_toolbox grid sits half a cell (+0.025 m in x and y) off its own map frame, measured in sim
    shift = {'map_origin_shift': -0.025 if slam_be == 'toolbox' else 0.0}
    actions.append(Node(package='team7_nodes', executable='marker_mapper', name='marker_mapper',
                        parameters=[team7_yaml, out_dir, shift], output='screen'))
    if on('mission'):
        actions.append(Node(package='team7_nodes', executable='mission', name='mission',
                            parameters=[team7_yaml, out_dir, shift, {
                                'nav_action': nav_action,
                                'time_limit': ParameterValue(arg('time_limit'), value_type=float),
                                'expected_targets': ParameterValue(arg('expected_targets'), value_type=int)}],
                            output='screen'))

    explorers = []
    if explorer == 'team7':
        explorers.append(Node(package='team7_nodes', executable='explorer', name='explorer',
                              parameters=[team7_yaml, {
                                  'nav_action': nav_action,
                                  'costmap_topic': '/global_costmap/costmap' if use_nav2 else '/map'}],
                              output='screen'))
    elif explorer == 'explore_lite':
        explorers.append(Node(package='explore_lite', executable='explore', name='explore_node',
                              parameters=[cfg('explore_lite.yaml')], output='screen'))
    if explorers:
        actions.append(TimerAction(period=10.0, actions=explorers))

    # Ubuntu RViz/Ogre uses GLX. Inherited Nix graphics libraries can prevent
    # it from creating a window under XWayland; keep the ROS library paths.
    if on('rviz'):
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                            arguments=['-d', rviz_config_for(nav, aruco_be)],
                            additional_env=native_env, prefix=native_prefix))
    return actions

def generate_launch_description():
    sim_time = SetParameter(name='use_sim_time',
                            value=ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool))
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in ARGS]
        + [sim_time, OpaqueFunction(function=setup)]
    )
