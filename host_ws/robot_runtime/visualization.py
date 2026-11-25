"""Configuration for the optional rust_base map viewer."""
import json
import os
from pathlib import Path


MARKER_TOPIC = '/team7/v1/targets'
VIEWER_PROFILE = 'v1-live'


def find_native_viewer(installs):
    prefixes = [prefix for install in installs for prefix in (install / 'team7_viz', install)]
    prefixes.extend(Path(path) for path in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if path)
    for prefix in dict.fromkeys(prefixes):
        binary = prefix / 'lib/team7_viz/team7_viz'
        if binary.is_file() and os.access(binary, os.X_OK):
            return binary
    return None


def prepare_viewer_profile(config_home, *, map_topic, scan_topic, image_topic):
    # Leave the regular rust_base profiles alone. Both V1 marker layers read
    # the mission's MarkerArray, since upstream detections have their own filter.
    profile_dir = config_home / 'team7_viz'
    profile_dir.mkdir(parents=True)
    contents = [f'camera_topic = {json.dumps(image_topic.removesuffix("/compressed"))}']
    for kind, topic in (('map', map_topic), ('scan', scan_topic),
                        ('confirmed', MARKER_TOPIC), ('candidates', MARKER_TOPIC)):
        contents.extend(('', '[[layers]]', f'kind = "{kind}"', f'topic = {json.dumps(topic)}'))
    contents.extend(('', '[view]', 'follow = true', ''))
    (profile_dir / f'{VIEWER_PROFILE}.toml').write_text('\n'.join(contents))
