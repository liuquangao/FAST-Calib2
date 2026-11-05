"""Multi-scene joint calibration from the last num_scenes recorded scenes (default 3, 0 = all)."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import fast_calib_common as common  # noqa: E402


def generate_launch_description():
    return common.calib_launch('multi_fast_calib', ['output_path', 'num_scenes'])
