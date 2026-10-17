#!/usr/bin/env python3
"""Capture one static calibration scene for FAST-Calib2 (ROS 2).

Records the LiDAR topic into a rosbag2 for --duration seconds and saves the last
camera frame plus its camera_info. Keep the rig and the target still while it runs.

Output layout (default under <repo>/calib_data):
    <name>/lidar/            rosbag2 with the LiDAR topic
    <name>/image.png         camera frame (BGR)
    <name>/camera_info.yaml  intrinsics of that frame

Example:
    ros2 run fast_calib capture_scene.py mid --duration 15
"""

import argparse
import os
import time

import cv2
import numpy as np
import rclpy
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2

REPO_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

try:
    from livox_ros_driver2.msg import CustomMsg
except ImportError:  # optional
    CustomMsg = None


class SceneCapture(Node):
    def __init__(self, args, out_dir):
        super().__init__('fast_calib_capture')
        self.args = args
        self.lidar_count = 0
        self.last_image = None
        self.camera_info = None
        self.acc = []

        lidar_type = self.resolve_type(args.lidar_topic)
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(
            rosbag2_py.StorageOptions(uri=os.path.join(out_dir, 'lidar'), storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('cdr', 'cdr'))
        self.writer.create_topic(rosbag2_py.TopicMetadata(
            name=args.lidar_topic, type=lidar_type, serialization_format='cdr'))

        msg_cls = PointCloud2 if lidar_type == 'sensor_msgs/msg/PointCloud2' else CustomMsg
        self.create_subscription(msg_cls, args.lidar_topic, self.on_lidar,
                                 qos_profile_sensor_data, raw=True)
        # raw=True: only the last frame is kept, so deserialise (slow for a 1080p frame in rclpy) once at the end.
        self.create_subscription(Image, args.image_topic, self.on_image, qos_profile_sensor_data, raw=True)
        self.create_subscription(CameraInfo, args.info_topic, self.on_info, qos_profile_sensor_data)
        if args.imu_topic:
            self.create_subscription(Imu, args.imu_topic, self.on_imu, qos_profile_sensor_data)

    def resolve_type(self, topic):
        deadline = time.time() + 5.0
        while time.time() < deadline:
            for name, types in self.get_topic_names_and_types():
                if name == topic and types:
                    if types[0] == 'livox_ros_driver2/msg/CustomMsg' and CustomMsg is None:
                        raise RuntimeError('livox_ros_driver2 python package not sourced')
                    return types[0]
            time.sleep(0.2)
        raise RuntimeError(f'LiDAR topic {topic} is not being published')

    def on_lidar(self, data):
        self.writer.write(self.args.lidar_topic, data, self.get_clock().now().nanoseconds)
        self.lidar_count += 1

    def on_image(self, data):
        self.last_image = data

    def on_info(self, msg):
        self.camera_info = msg

    def on_imu(self, msg):
        a = msg.linear_acceleration
        self.acc.append((a.x, a.y, a.z))


def write_camera_info(path, info):
    k, d = info.k, list(info.d) + [0.0] * 5
    text = (f'camera_width: {info.width}\ncamera_height: {info.height}\n'
            f'fx: {k[0]:.9f}\nfy: {k[4]:.9f}\ncx: {k[2]:.9f}\ncy: {k[5]:.9f}\n'
            f'k1: {d[0]:.9f}\nk2: {d[1]:.9f}\np1: {d[2]:.9f}\np2: {d[3]:.9f}\n'
            f'# distortion_model: {info.distortion_model}, D: {list(info.d)}\n')
    with open(path, 'w') as f:
        f.write(text)
    return text


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('name', help='scene name, e.g. mid / left / right')
    parser.add_argument('--duration', type=float, default=15.0, help='LiDAR recording time [s]')
    parser.add_argument('--out-dir', default=os.path.join(REPO_DIR, 'calib_data'))
    parser.add_argument('--lidar-topic', default='/livox/lidar')
    parser.add_argument('--image-topic', default='/zed/zed_node/rgb/color/rect/image')
    parser.add_argument('--info-topic', default='', help='default: <image-topic>/camera_info')
    parser.add_argument('--imu-topic', default='/livox/imu',
                        help='LiDAR IMU used to report which LiDAR axis points up ("" to disable)')
    args, ros_args = parser.parse_known_args()
    args.info_topic = args.info_topic or args.image_topic + '/camera_info'

    out_dir = os.path.join(args.out_dir, args.name)
    if os.path.exists(os.path.join(out_dir, 'lidar')):
        parser.error(f'{out_dir}/lidar already exists; pick another name or remove it')
    os.makedirs(out_dir, exist_ok=True)

    rclpy.init(args=ros_args)
    node = SceneCapture(args, out_dir)
    print(f'Recording {args.lidar_topic} for {args.duration:.0f} s -> {out_dir} (keep everything still)')
    end = time.time() + args.duration
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    # make sure we have a frame and intrinsics even if the camera is slow
    deadline = time.time() + 5.0
    while rclpy.ok() and (node.last_image is None or node.camera_info is None) and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    del node.writer  # closes the bag

    print(f'LiDAR messages recorded: {node.lidar_count}')
    ok = node.lidar_count > 0
    if node.last_image is None:
        print(f'ERROR: no image received on {args.image_topic}')
        ok = False
    else:
        img = CvBridge().imgmsg_to_cv2(deserialize_message(node.last_image, Image), "bgr8")
        cv2.imwrite(os.path.join(out_dir, 'image.png'), img)
        print(f'Image saved: {img.shape[1]}x{img.shape[0]} -> {out_dir}/image.png')
    if node.camera_info is None:
        print(f'WARNING: no camera_info on {args.info_topic}')
    else:
        print('Camera intrinsics (for config/qr_params.yaml):')
        print(write_camera_info(os.path.join(out_dir, 'camera_info.yaml'), node.camera_info))
    if node.acc:
        mean = np.mean(np.array(node.acc), axis=0)
        axis = int(np.argmax(np.abs(mean)))
        sign = '+' if mean[axis] > 0 else '-'
        print(f'IMU mean acceleration {np.round(mean, 3).tolist()} -> lidar_up_axis is likely '
              f'"{sign}{"xyz"[axis]}" (assumes the IMU axes match the LiDAR axes, e.g. Livox)')

    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
