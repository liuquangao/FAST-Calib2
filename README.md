# FAST-Calib2 — ROS 2 Port

This fork provides a ROS 2 port of [xuankuzcr/FAST-Calib2](https://github.com/xuankuzcr/FAST-Calib2), with tooling for static LiDAR-camera calibration using a Livox Mid360S and a ZED2i camera. The adaptation lives on the `ros2-humble` branch; the branch name describes the port target, not a guarantee of compatibility with every ROS 2 distribution.

The port adds `ament_cmake` builds, `rclcpp` nodes, Python launch files, rosbag2 input, static scene capture, and offline cloud colouring. The original calibration method, target design, and authorship are credited below.

## LiDAR-Camera Extrinsic Calibration with Reflective Annular Targets

FAST-Calib2 extends [FAST-Calib](https://github.com/hku-mars/FAST-Calib) to LiDAR-camera modules that were previously hard to calibrate due to **low-quality point clouds**. With a custom-designed reflective annular calibration target, it enables robust center extraction on **large-spot solid-state and mechanical LiDARs**, including Mid360, Avia, Ouster, XT32, JT128, Airy, E1R, and Adaps Photonics Spad LiDAR.

**Key highlights include:**

1. A self-designed 3D reflective annular calibration target that avoids center extraction errors caused by hole-edge inflation and bleeding artifacts in previous circular-hole calibration boards.
2. A robust concentric-circle fitting method that uses the fixed inner and outer annulus radii as geometric constraints.
3. Automatic calibration board ROI extraction without manual pass-through tuning.
4. Geometry and radius quality checks for extracted annulus centers.
5. Single-scene and multi-scene LiDAR-camera extrinsic calibration without initial extrinsic parameters.

📬 For further assistance or inquiries, please feel free to contact Chunran Zheng at zhengcr@connect.hku.hk.

<p align="center">
  <img src="./pics/cover.jpg" width="100%">
  <font color=#a0a0a0 size=2>Mid360 calibration example.</font>
</p>

## 1. Prerequisites

- ROS 2 Humble with a sourced development environment, `colcon`, and `rosdep`.
- C++17, PCL >= 1.10, and OpenCV >= 4.0 with ArUco support (the contrib module on older OpenCV versions). The code also supports the OpenCV >= 4.7 ArUco API.
- ROS 2 packages listed in `package.xml`, plus `cv_bridge` for scene capture and `rviz2` for optional visualization.
- Optional: build and source `livox_ros_driver2` **before compiling** this package to enable `livox_ros_driver2/msg/CustomMsg` input. Standard `sensor_msgs/msg/PointCloud2` input does not require it.

Build in a ROS 2 workspace:

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/fast_calib_ws/src
cd ~/fast_calib_ws/src
git clone --branch ros2-humble https://github.com/liuquangao/FAST-Calib2.git
cd ~/fast_calib_ws
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-humble-cv-bridge ros-humble-rviz2 libopencv-contrib-dev
colcon build --symlink-install --packages-select fast_calib
source install/setup.bash
cd src/FAST-Calib2
```

Source an existing Livox workspace before the build if using CustomMsg, then rebuild `fast_calib` if that driver was added later. Use `--symlink-install`: the launch helper resolves default data/output paths relative to the source configuration file, and the capture script resolves its default data directory relative to its source file.

Commands below assume the workspace has been sourced and the current directory is this repository. Local recordings and generated results are excluded from Git; collect your own data before running calibration.

## 2. Calibration Target

FAST-Calib2 uses four reflective annuli and four visual markers on one board. The annuli are used by LiDAR center extraction, while the visual markers are used by the camera pipeline.

Materials:

- Board: PVC
- Reflective annulus stickers: 3M engineering-grade reflective film

<p align="center">
  <img src="./pics/FAST-Calib2-board.png" width="100%">
  <font color=#a0a0a0 size=2>Reflective annular calibration target and annotated dimensions.</font>
</p>

DIY Calibration Target Tips:

1. Fabricate the board based on the schematic. Ensure a minimum thickness of 1 cm to avoid bending.
2. Apply reflective annulus stickers to the designated ring positions on the fabricated board.

## 3. Method Overview

Both LiDAR pipelines first **locate the calibration board automatically**, fit the board plane, and align the plane to `Z=0`. Center extraction is then performed in the aligned board frame.

Solid-state LiDAR pipeline:

1. Extract high-reflectivity annulus points on the fitted board plane.
2. Cluster the extracted annulus points.
3. Fit robust single circles as the default center estimate.
4. Optionally extract annulus boundary points and fit fixed inner/outer radius concentric circles.
5. Select the best result by checking four-center geometry consistency against the known target geometry.

Mechanical LiDAR pipeline:

1. Use LiDAR ring order within each scan to find intensity transition points on the annulus boundary.
2. Try both interpolated boundary points and high-reflectivity-side boundary points.
3. Cluster the extracted boundary points.
4. Fit fixed inner/outer radius concentric circles.
5. Select the best result by checking four-center geometry consistency against the known target geometry.

The final quality checks include center-to-center geometry error and annulus radius consistency.

## 4. Run Examples

### Configure the rig and target

Edit the flat YAML file `config/qr_params.yaml` to match your camera intrinsics, physical target dimensions, and LiDAR mounting:

```yaml
lidar_forward_axis: "+x"
lidar_up_axis: "+z"
```

Each value describes the signed native LiDAR axis pointing in that physical direction. The two axes must be perpendicular, for example `+x` and `-y`. Target dimensions are in metres. The bundled camera parameters are an example for a ZED2i rectified HD1080 stream; replace them for your own camera. Rectified images require matching rectified intrinsics and zero distortion.

The launch helper automatically overrides camera intrinsics from `camera_info.yaml` beside the selected image, when present. Use `config_file:=/absolute/path/to/qr_params.yaml` to select another flat YAML configuration. The helper treats the parent of the directory containing the configuration file as the package root for `$(find fast_calib)` paths; keep custom configs in a `config/` subdirectory or use absolute data/output paths.

### Capture static scenes

Start the camera and LiDAR drivers first. Keep both the rig and the target still throughout each capture. Check the actual topic names with `ros2 topic list -t`, then record one scene:

```bash
ros2 run fast_calib capture_scene.py mid --duration 15 \
  --lidar-topic /livox/lidar \
  --image-topic /zed/zed_node/rgb/color/rect/image \
  --info-topic /zed/zed_node/rgb/color/rect/image/camera_info
```

Pass the actual CameraInfo topic explicitly if your driver uses another name. The script defaults to `<image-topic>/camera_info`. Its optional IMU subscription defaults to `/livox/imu`; use `--imu-topic ""` to disable it. The reported up-axis estimate assumes the IMU and LiDAR axes agree.

The capture creates:

```text
calib_data/mid/
├── lidar/             # SQLite3 rosbag2 directory
├── image.png          # Last camera frame received
└── camera_info.yaml   # Intrinsics, when CameraInfo is available
```

Repeat with scene names `left` and `right`, changing the target placement between captures. An existing scene's `lidar` directory is not overwritten. These are static acquisitions: the script saves the last image and accumulates LiDAR messages rather than synchronizing a moving sequence.

You can also supply an existing rosbag2 directory (or `.db3` file) and a corresponding image. Supported LiDAR message types are `sensor_msgs/msg/PointCloud2` and, when enabled at build time, `livox_ros_driver2/msg/CustomMsg`. Mechanical LiDAR processing requires the `ring` field. Upstream [example data](https://drive.google.com/drive/folders/1VnMCsGj3Gat7dxe6IION0SfS7jYNMw1g?usp=sharing) may contain ROS 1 `.bag` files; those must be converted to rosbag2 before use with this branch.

### Single-scene calibration

```bash
ros2 launch fast_calib calib.launch.py \
  bag_path:="$PWD/calib_data/mid/lidar" \
  image_path:="$PWD/calib_data/mid/image.png" \
  output_path:="$PWD/output" rviz:=true
```

Additional launch overrides are `lidar_topic`, `lidar_forward_axis`, `lidar_up_axis`, and `use_auto_lidar_roi`. RViz is disabled by default.

Outputs include `single_calib_result.txt`, `colored_cloud.pcd`, and `qr_detect.png`. Each successful run appends four LiDAR-camera center pairs to `circle_center_record.txt`. The single-scene result and debug images/cloud are overwritten on subsequent runs; copy them if you want per-scene snapshots.

### Multi-scene calibration

Run single-scene calibration for `mid`, `left`, and `right`, using the same output directory and camera configuration. Then run:

```bash
ros2 launch fast_calib multi_calib.launch.py output_path:="$PWD/output"
```

By default the solver uses the **last three valid scene records** in `output/circle_center_record.txt`, and writes `output/multi_calib_result.txt`. Pass `num_scenes:=N` to use the last N records, or `num_scenes:=0` to use all of them; more scenes spread over distance, height, and tilt make the extrinsic more stable. The solver prints each scene's residual so an inconsistent scene can be spotted and its three-line record removed. Use a fresh output directory for a new rig or calibration session so records do not mix.

Both result files contain FAST-LIVO2-style `Rcl` and `Pcl` values, with the LiDAR-to-camera convention:

```text
p_camera = Rcl * p_lidar + Pcl
```

Translation is in metres. Check the projected cloud/image alignment before using the extrinsics downstream.

Typical multi-scene target placement:

<p align="center">
  <img src="./pics/multi-scene.jpg" width="100%">
  <font color=#a0a0a0 size=2>Placement of the calibration target for multi-scene data collection: (a) facing forward, (b) oriented to the right, (c) oriented to the left.</font>
</p>

## 5. Standalone LiDAR Center Extraction Test

The LiDAR-only tool checks annulus center extraction before full camera-LiDAR calibration:

```bash
ros2 launch fast_calib lidar_center_test.launch.py \
  bag:="$PWD/calib_data/mid/lidar" topic:=/livox/lidar mode:=solid \
  output_path:="$PWD/output"
```

Use `mode:=mech` for mechanical LiDAR data and supply its point-cloud topic. The default `mode:=auto` infers the pipeline from the message fields. `config_file`, `lidar_forward_axis`, and `lidar_up_axis` can also be overridden.

The tool writes `*_centers.txt` and `*_debug_cloud.pcd`. Debug cloud colors are: board points in an intensity color map, annulus points in green, solid-LiDAR boundary points in red, and centers as white spheres.

## 6. Optional Offline Cloud Colouring

With Open3D installed in your ROS Python environment, colour a captured scene using the estimated extrinsics:

```bash
python3 scripts/colorize_cloud.py calib_data/mid output/multi_calib_result.txt \
  output/colored_mid_occl.pcd --occlusion
```

This helper requires `camera_info.yaml`, a SQLite3 scene bag containing only `PointCloud2` messages, and the Python `sensor_msgs_py`, NumPy, OpenCV, PyYAML, and Open3D packages. It does not decode Livox CustomMsg. `--occlusion` drops points hidden from the camera; `--occlusion grey` keeps them in grey. The helper is run from source and is not installed as a `ros2 run` executable.

To check the extrinsics visually, project a captured scene's cloud onto its image, coloured by depth:

```bash
python3 scripts/project_cloud.py calib_data/mid output/multi_calib_result.txt \
  output/projection_mid.png
```

The target edges, annuli, and other scene edges should coincide with the image. It has the same input requirements as `colorize_cloud.py` but does not need Open3D. `--depth-range NEAR FAR` sets the depth mapped to the colour map ends (default 1–6 m). It is also run from source.

The older `scripts/distance_filter_tool.py` still imports ROS 1 `rosbag` and is not part of the ROS 2 workflow, even though it is installed by CMake.

## 7. Tests

The package includes C++ tests for common geometry and configuration helpers:

```bash
cd ~/fast_calib_ws
colcon test --packages-select fast_calib --event-handlers console_direct+
colcon test-result --verbose
```

These tests do not replace validation on your own recorded calibration scenes.

## 8. Attribution and License

Original FAST-Calib2 author: Chunran Zheng and the upstream contributors. This fork builds on [FAST-Calib2](https://github.com/xuankuzcr/FAST-Calib2) and its predecessor [FAST-Calib](https://github.com/hku-mars/FAST-Calib).

The ROS 2 adaptation and documentation are maintained in this fork. This project is distributed under the GNU General Public License version 2; see [LICENSE](LICENSE). Original copyright and author notices are retained.
