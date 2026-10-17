"""Multi-scene joint calibration from the last three recorded scenes."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import fast_calib_common as common  # noqa: E402


def generate_launch_description():
    return common.calib_launch('multi_fast_calib', ['output_path'])
