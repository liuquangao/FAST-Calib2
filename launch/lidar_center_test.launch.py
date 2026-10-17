"""LiDAR-only annulus center extraction test.

Example:
  ros2 launch fast_calib lidar_center_test.launch.py bag:=calib_data/mid/lidar topic:=/livox/lidar mode:=solid
"""

import os
import sys

from launch.actions import DeclareLaunchArgument

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import fast_calib_common as common  # noqa: E402


def generate_launch_description():
    return common.calib_launch(
        'lidar_center_test', ['output_path', 'lidar_forward_axis', 'lidar_up_axis'],
        extra_args=[DeclareLaunchArgument('bag'),
                    DeclareLaunchArgument('topic', default_value='/livox/lidar'),
                    DeclareLaunchArgument('mode', default_value='auto')],
        node_arguments=lambda ctx: [os.path.abspath(ctx.launch_configurations['bag']),
                                    ctx.launch_configurations['topic'],
                                    ctx.launch_configurations['mode']])
