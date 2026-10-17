"""Shared helpers for the FAST-Calib2 ROS 2 launch files."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch_ros.actions import Node

SHARE = get_package_share_directory('fast_calib')
CAMERA_KEYS = ('camera_width', 'camera_height', 'fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'p1', 'p2')


def load_params(context, overrides):
    """Load the flat YAML config, apply non-empty launch-argument overrides (typed via YAML),
    expand "$(find fast_calib)" to the package source tree, and take the camera intrinsics
    from <image dir>/camera_info.yaml when capture_scene.py saved one next to the image."""
    cfg = context.launch_configurations
    config_file = cfg['config_file']
    with open(config_file) as f:
        params = yaml.safe_load(f) or {}
    for name in overrides:
        if cfg.get(name, ''):
            params[name] = yaml.safe_load(cfg[name])

    # With --symlink-install the config resolves into the source tree (where calib_data/ and output/ live).
    package_root = os.path.dirname(os.path.dirname(os.path.realpath(config_file)))
    for key, value in params.items():
        if isinstance(value, str):
            params[key] = value.replace('$(find fast_calib)', package_root)

    notes = []
    image_path = params.get('image_path', '')
    camera_info = os.path.join(os.path.dirname(image_path), 'camera_info.yaml')
    if image_path and os.path.isfile(camera_info):
        with open(camera_info) as f:
            info = yaml.safe_load(f)
        params.update({k: info[k] for k in CAMERA_KEYS if k in info})
        notes.append(LogInfo(msg=f'Camera intrinsics taken from {camera_info}'))
    return params, notes


def calib_launch(executable, overrides, extra_args=(), node_arguments=None, extras=()):
    """LaunchDescription for one fast_calib executable configured from the flat YAML."""
    def setup(context):
        params, notes = load_params(context, overrides)
        arguments = node_arguments(context) if node_arguments else []
        return notes + [Node(package='fast_calib', executable=executable, name=executable,
                             output='screen', parameters=[params], arguments=arguments)]

    return LaunchDescription([
        DeclareLaunchArgument('config_file',
                              default_value=os.path.join(SHARE, 'config', 'qr_params.yaml')),
        *[DeclareLaunchArgument(n, default_value='') for n in overrides],
        *extra_args,
        OpaqueFunction(function=setup),
        *extras,
    ])


def rviz_node(condition):
    return Node(package='rviz2', executable='rviz2', name='rviz2', output='log', condition=condition,
                arguments=['-d', os.path.join(SHARE, 'rviz_cfg', 'fast_calib.rviz')])
