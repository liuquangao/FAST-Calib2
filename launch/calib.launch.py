"""Single-scene LiDAR-camera calibration."""

import os
import sys

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import fast_calib_common as common  # noqa: E402


def generate_launch_description():
    return common.calib_launch(
        'fast_calib',
        ['bag_path', 'image_path', 'lidar_topic', 'lidar_forward_axis', 'lidar_up_axis',
         'output_path', 'use_auto_lidar_roi'],
        extra_args=[DeclareLaunchArgument('rviz', default_value='false')],
        extras=[common.rviz_node(IfCondition(LaunchConfiguration('rviz')))])
