#!/usr/bin/env python3
"""Colour a recorded scene's LiDAR cloud with its camera image (offline).

Reads <scene>/lidar (rosbag2), <scene>/image.png and <scene>/camera_info.yaml as
written by capture_scene.py, and an extrinsic in FAST-Calib2 result format
(Rcl/Pcl, e.g. output/multi_calib_result.txt).

With --occlusion, points the camera cannot see (hidden behind a nearer object
because the LiDAR and camera are mounted at different positions) are removed
instead of taking the occluder's colour; --occlusion grey keeps them, painted grey.

Example:
    python3 scripts/colorize_cloud.py calib_data/mid output/multi_calib_result.txt \\
        output/colored_mid_occl.pcd --occlusion
"""

import argparse
import re

import cv2
import numpy as np
import open3d as o3d
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
    ap.add_argument('output', help='output .pcd (LiDAR frame, rgb)')
    ap.add_argument('--occlusion', nargs='?', const='drop', choices=['drop', 'grey'],
                    help='handle points hidden from the camera: drop (default) or paint grey')
    ap.add_argument('--voxel', type=float, default=0.01, help='output voxel size [m], 0 to keep all')
    args = ap.parse_args()

    R, t = load_extrinsic(args.extrinsic)
    info = yaml.safe_load(open(f'{args.scene}/camera_info.yaml'))
    K = np.array([[info['fx'], 0, info['cx']], [0, info['fy'], info['cy']], [0, 0, 1]])
    D = np.array([info['k1'], info['k2'], info['p1'], info['p2']])
    img = cv2.imread(f'{args.scene}/image.png')
    h, w = img.shape[:2]

    p = load_points(f'{args.scene}/lidar')
    pc = p @ R.T + t
    z = pc[:, 2]
    front = z > 0.05
    p, pc, z = p[front], pc[front], z[front]
    uv = cv2.projectPoints(pc, np.zeros(3), np.zeros(3), K, D)[0].reshape(-1, 2)
    u, v = uv[:, 0].astype(np.int64), uv[:, 1].astype(np.int64)
    ok = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    p, u, v, z = p[ok], u[ok], v[ok], z[ok]
    bgr = img[v, u].copy()

    if args.occlusion:
        # coarse z-buffer: a point is hidden if a nearer point lands in the same or a neighbouring 4 px cell
        cell = 4
        cu, cv_ = u // cell, v // cell
        zbuf = np.full((h // cell + 2, w // cell + 2), np.inf)
        np.minimum.at(zbuf, (cv_, cu), z)
        zmin = zbuf[cv_, cu]
        for dv in (-1, 0, 1):
            for du in (-1, 0, 1):
                zmin = np.minimum(zmin, zbuf[np.clip(cv_ + dv, 0, None), np.clip(cu + du, 0, None)])
        hidden = z > zmin * 1.03 + 0.10
        if args.occlusion == 'grey':
            bgr[hidden] = (120, 120, 120)
        else:
            p, bgr = p[~hidden], bgr[~hidden]
        print(f'occluded from camera: {hidden.sum()} points ({100 * hidden.mean():.1f}%), '
              f'{"painted grey" if args.occlusion == "grey" else "removed"}')

    if args.voxel > 0:
        _, idx = np.unique(np.floor(p / args.voxel).astype(np.int64), axis=0, return_index=True)
        p, bgr = p[idx], bgr[idx]
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p))
    cloud.colors = o3d.utility.Vector3dVector(bgr[:, ::-1] / 255.0)
    o3d.io.write_point_cloud(args.output, cloud)
    print(f'{args.output}: {len(p)} points')


if __name__ == '__main__':
    main()
