# aerial_tn_utility

ROS 2 utility package for the AerialTN drone platform (VOXL-based). Provides H.264 decoding nodes, a PX4 odometry republisher, and launch files for rosbag playback and live streaming.

## Dependencies

- [`aerial_tn`](https://github.com/AerialTN/aerial_tn) — provides the `voxl_h264_decoder` node
- `px4_msgs`
- FFmpeg (`libavcodec`, `libavutil`, `libswscale`)

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select aerial_tn_utility
source install/setup.bash
```

## Quick start

In most cases, simply launch the decoder and play back your rosbag:

```bash
ros2 launch aerial_tn_utility decoder_launch.py
```

```bash
ros2 bag play <path_to_bag>
```

## Nodes

### `ircam_decoder`

Decodes H.264-encoded IR camera streams using FFmpeg. Subscribes to a `sensor_msgs/CompressedImage` topic and publishes a `sensor_msgs/Image` in `mono8` encoding. Waits for an IDR keyframe before starting to decode.

| Parameter | Default | Description |
|---|---|---|
| `input_topic` | `/ircam/h264` | Encoded input topic |
| `output_topic` | `/ircam/decoded` | Decoded output topic |

### `px4_odom_republisher_node`

Converts PX4 `VehicleOdometry` (NED/FRD frame) to standard ROS `nav_msgs/Odometry` (ENU/FLU frame). Also publishes a `nav_msgs/Path` and broadcasts the `odom → base_link` TF transform.

| Subscribed | Published |
|---|---|
| `/fmu/out/vehicle_odometry` | `/px4/odom`, `/px4/path`, TF |

## Launch files

### `decoder_launch.py` — rosbag playback (primary use case)

Decodes all VOXL camera streams from an H.264-encoded rosbag and opens RViz.

```bash
ros2 launch aerial_tn_utility decoder_launch.py
```

Then in a separate terminal, play back your rosbag:

```bash
ros2 bag play <path_to_bag>
```

Decoded topics:

| Raw (encoded) | Decoded |
|---|---|
| `/low_light_down_misp_encoded` | `/low_light_down_misp_decoded` |
| `/tracking_down_misp_encoded` | `/tracking_down_misp_decoded` |
| `/tracking_front_misp_encoded` | `/tracking_front_misp_decoded` |
| `/ircam/h264` | `/ircam/decoded` |

> **Note:** `live_stream` is set to `False` and `convert_to_bgr` to `True` — optimized for offline bag playback with RViz visualization.

### `stereo_ir_decoder_launch.py`

Variant for stereo IR camera setups (`ircam_wide` and `ircam_narrow`). Usage identical to above.

### `record_decoded_launch.py`

Records decoded topics to `/bagfiles/voxlbag_decoded_<timestamp>`. Bags are split at 5 GB. Run alongside `decoder_launch.py` when you need to save decoded output.

```bash
ros2 launch aerial_tn_utility record_decoded_launch.py
```

### `orbslam_mono_launch.py` / `orbslam_mono_inertial_launch.py`

Launch files for running ORB-SLAM3 in monocular and monocular-inertial modes.
