from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

ARGS = [
    ('explorer', 'team7', 'team7 | explore_lite | none'),
    ('nav', 'nav2', 'nav2 (NavFn + MPPI) | lazy_nav2 (our Lazy Theta* + Nav2 MPPI) | team7 (Lazy Theta* + our MPPI, no Nav2)'),
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
    ('slam_sync', 'true', 'Synchronous slam_toolbox (processes every scan)'),
    ('use_sim_time', 'false', 'true when running against the Gazebo simulation'),
]

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

    if use_nav2:
        actions.append(TimerAction(period=3.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([tb4_nav, 'launch', 'nav2.launch.py'])),
            launch_arguments={'params_file': cfg('nav2.yaml'),
                              'use_sim_time': arg('use_sim_time')}.items())]))
    if nav in ('lazy_nav2', 'team7'):
        actions.append(Node(package='team7_nodes', executable='navigator', name='navigator',
                            parameters=[team7_yaml, {'mode': 'nav2' if nav == 'lazy_nav2' else 'mppi'}],
                            output='screen'))

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
                            output='screen'))

    actions.append(Node(package='team7_nodes', executable='marker_mapper', name='marker_mapper',
                        parameters=[team7_yaml, out_dir], output='screen'))
    if on('mission'):
        actions.append(Node(package='team7_nodes', executable='mission', name='mission',
                            parameters=[team7_yaml, out_dir, {
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

    if on('rviz'):
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                            arguments=['-d', PathJoinSubstitution([bringup, 'rviz', 'team7.rviz'])]))
    return actions

def generate_launch_description():
    sim_time = SetParameter(name='use_sim_time',
                            value=ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool))
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in ARGS]
        + [sim_time, OpaqueFunction(function=setup)]
    )
