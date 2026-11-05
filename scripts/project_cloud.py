#!/usr/bin/env python3
"""Project a recorded scene's LiDAR cloud onto its camera image (offline).

Reads <scene>/lidar (rosbag2), <scene>/image.png and <scene>/camera_info.yaml as
written by capture_scene.py, and an extrinsic in FAST-Calib2 result format
(Rcl/Pcl, e.g. output/multi_calib_result.txt). Points are drawn coloured by depth
(near = blue, far = red) so edges of the target and the scene can be checked
against the image.

Example:
    python3 scripts/project_cloud.py calib_data/mid output/multi_calib_result.txt \\
        output/projection_mid.png
"""

import argparse
import re

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from numpy.lib.recfunctions import structured_to_unstructured
from sensor_msgs_py.point_cloud2 import read_points


def load_extrinsic(path):
    text = open(path).read()
    nums = lambda key: [float(v) for v in re.search(key + r':\s*\[([^\]]*)\]', text).group(1).split(',')]
    return np.array(nums('Rcl')).reshape(3, 3), np.array(nums('Pcl'))


def load_points(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    pts = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, PointCloud2)
        # read_points (not read_points_numpy): Livox clouds mix field types (uint8 tag/line, float64 time)
        pts.append(structured_to_unstructured(read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)))
    p = np.concatenate(pts).astype(np.float64)
    return p[np.linalg.norm(p, axis=1) > 0.3]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('scene', help='scene directory from capture_scene.py')
    ap.add_argument('extrinsic', help='file with Rcl/Pcl (T_cam_lidar)')
    ap.add_argument('output', help='output image (.png/.jpg)')
    ap.add_argument('--depth-range', type=float, nargs=2, default=(1.0, 6.0), metavar=('NEAR', 'FAR'),
                    help='depth [m] mapped to the colour map ends')
    ap.add_argument('--radius', type=int, default=1, help='point radius [px]')
    ap.add_argument('--alpha', type=float, default=0.8, help='overlay opacity')
    args = ap.parse_args()

    R, t = load_extrinsic(args.extrinsic)
    info = yaml.safe_load(open(f'{args.scene}/camera_info.yaml'))
    K = np.array([[info['fx'], 0, info['cx']], [0, info['fy'], info['cy']], [0, 0, 1]])
    D = np.array([info['k1'], info['k2'], info['p1'], info['p2']])
    img = cv2.imread(f'{args.scene}/image.png')
    h, w = img.shape[:2]

    pc = load_points(f'{args.scene}/lidar') @ R.T + t
    pc = pc[pc[:, 2] > 0.2]
    uv = cv2.projectPoints(pc, np.zeros(3), np.zeros(3), K, D)[0].reshape(-1, 2)
    u, v = uv[:, 0].astype(np.int64), uv[:, 1].astype(np.int64)
    ok = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    u, v, z = u[ok], v[ok], pc[ok, 2]

    near, far = args.depth_range
    order = np.argsort(-z)  # far first, so nearer points end up on top
    u, v, z = u[order], v[order], z[order]
    level = np.clip((z - near) / (far - near) * 255, 0, 255).astype(np.uint8)
    colors = cv2.applyColorMap(level.reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)

    overlay = img.copy()
    for x, y, c in zip(u, v, colors):
        cv2.circle(overlay, (int(x), int(y)), args.radius, tuple(int(i) for i in c), -1)
    cv2.imwrite(args.output, cv2.addWeighted(overlay, args.alpha, img, 1 - args.alpha, 0))
    print(f'{args.output}: {len(u)} points projected')


if __name__ == '__main__':
    main()
