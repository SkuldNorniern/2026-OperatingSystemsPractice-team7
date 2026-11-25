"""Per-run rust_base config for V1's RViz markers and detection speed limit."""
import math
import shutil
import xml.etree.ElementTree as ET

import yaml


def prepare_bringup(source, prefix, *, map_frame, map_topic, base_frame='base_link'):
    package = 'team7_bringup'
    share = prefix / 'share' / package
    shutil.copytree(source, share)
    config = share / 'config/team7.yaml'
    params = yaml.safe_load(config.read_text())
    params['marker_mapper']['ros__parameters']['slow_on_tentative'] = False
    params['explorer']['ros__parameters']['approach_markers'] = False
    guard = params['velocity_guard']['ros__parameters']
    guard.update(max_forward=.18, max_turn=.5)
    config.write_text(yaml.safe_dump(params, sort_keys=False))

    # The Rust navigator accepts the sentinel "xy_only". Give Nav2 equivalent
    # behavior using a dedicated goal checker and the same forward-only tree.
    nav_path = share / 'config/nav2.yaml'
    nav = yaml.safe_load(nav_path.read_text())
    controller = nav['controller_server']['ros__parameters']
    controller['goal_checker_plugins'].append('position_only_checker')
    controller['position_only_checker'] = dict(controller['general_goal_checker'], yaw_goal_tolerance=math.pi)
    smoother = nav['velocity_smoother']['ros__parameters']
    smoother.update(max_velocity=[.18, 0., .5], min_velocity=[0., 0., -.5])
    nav_path.write_text(yaml.safe_dump(nav, sort_keys=False))
    forward = share / 'config/navigate_forward.xml'
    tree = ET.parse(forward)
    nodes = tree.findall('.//FollowPath')
    for node in nodes:
        node.set('goal_checker_id', 'general_goal_checker')
    tree.write(forward, encoding='unicode')
    for node in nodes:
        node.set('goal_checker_id', 'position_only_checker')
    tree.write(share / 'config/navigate_forward_xy.xml', encoding='unicode')

    for path in (share / 'rviz').glob('*.rviz'):
        view = yaml.safe_load(path.read_text())
        manager = view['Visualization Manager']
        manager['Global Options']['Fixed Frame'] = map_frame
        current_view = manager.get('Views', {}).get('Current', {})
        if current_view.get('Class') == 'rviz_default_plugins/TopDownOrtho':
            current_view.update({'Target Frame': base_frame, 'X': 0, 'Y': 0})
        for display in manager['Displays']:
            topic = display.get('Topic', {})
            if topic.get('Value') == '/map':
                topic['Value'] = map_topic
            if topic.get('Value') == '/team7/targets':
                display['Name'] = 'V1 ArUco Targets'
                topic['Value'] = '/team7/v1/targets'
                topic['Durability Policy'] = 'Transient Local'
            if topic.get('Value') in ('/aruco_tracker/debug', '/team7/aruco/debug'):
                display['Enabled'] = False
                display['Value'] = False
        path.write_text(yaml.safe_dump(view, sort_keys=False))

    index = prefix / 'share/ament_index/resource_index/packages' / package
    index.parent.mkdir(parents=True)
    index.touch()
    return share
