"""Per-run rust_base config for V1's RViz markers and detection speed limit."""
import shutil

import yaml


def prepare_bringup(source, prefix, *, map_frame, map_topic):
    package = 'team7_bringup'
    share = prefix / 'share' / package
    shutil.copytree(source, share)
    config = share / 'config/team7.yaml'
    params = yaml.safe_load(config.read_text())
    params['marker_mapper']['ros__parameters']['slow_on_tentative'] = False
    config.write_text(yaml.safe_dump(params, sort_keys=False))

    for path in (share / 'rviz').glob('*.rviz'):
        view = yaml.safe_load(path.read_text())
        manager = view['Visualization Manager']
        manager['Global Options']['Fixed Frame'] = map_frame
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
