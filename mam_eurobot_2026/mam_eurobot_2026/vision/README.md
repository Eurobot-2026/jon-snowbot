# Vision Nodes
Reference for the ROS 2 nodes in `mam_eurobot_2026/vision`: what each one does, its topics/parameters, and how to launch it.

## aruco_detector
- **Node**: `aruco_detector` – detects ArUco markers on a provided camera topic, estimates each marker pose, and publishes a `vision_msgs/Detection3DArray` with pose/bbox per marker.
- **Inputs**: `--ros-args -p image_topic:=<sensor_msgs/Image>` (required, QoS depth 1). The ArUco dictionary, image size, and allowed IDs/marker lengths come from `vision_settings.yaml` (parameter `config_yaml`, default packaged file).
- **Outputs**: `/aruco/detections` (`vision_msgs/Detection3DArray`).
- **Debug**: `show_debug_image` (default `true`) opens an OpenCV window and draws marker axes.
- **Run for example**:
  ```bash
  ros2 run mam_eurobot_2026 aruco_detector --ros-args \
    -p image_topic:=/front_camera \
    -p config_yaml:=vision_settings.yaml \
    -p show_debug_image:=false
  ```

## color_detector
- **Node**: `color_detector` – thresholds incoming images in HSV, applies erosion/dilation, finds the largest blobs above `min_area`, and publishes their pixel centroids.
- **Inputs**: `--image-topic <sensor_msgs/Image>` (required). QoS BEST_EFFORT by default; add `--reliable` for RELIABLE.
- **Outputs**: `/camera/features/cursor_center` (`geometry_msgs/PointStamped`, pixel x/y). Shows a binary mask window; press `q` to stop.
- **Parameters** (override with `--ros-args`): `h_min/h_max`, `s_min/s_max`, `v_min/v_max` (defaults 40–70 / 170–255 / 170–255), `erode`, `dilate` (default 2), `min_area` (default 200).
- **Run**:
  ```bash
  ros2 run mam_eurobot_2026 color_detector --image-topic /top_camera/image_3 \
    --ros-args -p h_min:=35 -p h_max:=85
  ```

## estimate_cursor_position
- **Node**: `estimate_cursor_position` – projects cursor pixels into world coordinates using the TF from `world_to_topcamera`.
- **Inputs**: `/camera/features/cursor_center` (`geometry_msgs/PointStamped`, pixel coordinates, BEST_EFFORT) and TF `world -> camera_frame` (default `observation_device_optical_frame`).
- **Outputs**: `/world_coordinate/cursor` (`geometry_msgs/PointStamped` in `world` frame).
- **Key parameters**: `cursor_height` (0.08 m), `cursor_world_x` (-1.0 m), `image_width/height` (640x480), `horizontal_fov_deg` (60.0), `input_topic`, `output_topic`, `camera_frame`, `world_frame`, `tf_timeout_sec`.
- **Run**:
  ```bash
  ros2 run mam_eurobot_2026 estimate_cursor_position
  ```

## world_to_topcamera
- **Node**: `world_to_topcamera` – solves the static transform `world -> <camera>_optical_frame` for every camera defined in `vision_settings.yaml` using ArUco markers, publishes a latched pose, and broadcasts `/tf_static`.
- **Config**: `config_yaml` parameter (default `vision_settings.yaml`, either from CWD or packaged). Uses `global` fields (HFOV, image size, ArUco dictionary) and each entry under `cameras` (`image_topic`, `marker_id`, `marker_length_m`, `world_marker_xyzrpy`).
- **Inputs**: subscribes to every `image_topic` listed in the YAML (`sensor_msgs/Image`, QoS depth 5, BEST_EFFORT).
- **Outputs**: `/<camera>/pose_world` (`geometry_msgs/PoseStamped`, RELIABLE + TRANSIENT_LOCAL) and `/tf_static` (`world` → `<camera>_optical_frame`).
- **Exit behavior**: `exit_on_complete` (default `false`) controls shutdown after solving; `require_both_cameras` (default `false`) requires all cameras to be solved before exiting when `exit_on_complete` is true.
- **Run**:
  ```bash
  ros2 run mam_eurobot_2026 world_to_topcamera \
    --ros-args -p config_yaml:=vision_settings.yaml -p exit_on_complete:=true
  ```

## camera_viewer.py
- **Node**: `camera_viewer` – generic viewer for any image topic; logs `CameraInfo` and frame rate.
- **Inputs**: positional `<image_topic>` (`sensor_msgs/Image`, QoS depth 5, BEST_EFFORT/volatile). Optional `--info-topic` (defaults to `<image_topic>/camera_info`).
- **Outputs**: OpenCV window (press `q` to quit). In headless environments, periodically dumps `/tmp/camera_viewer_sample.jpg`.
- **Run (not registered as a console script)**:
  ```bash
  python3 src/mam_eurobot_2026/mam_eurobot_2026/vision/camera_viewer.py /front_camera
  ```

## show_front_camera.py
- Legacy viewer fixed to `/front_camera` and `/front_camera/camera_info` (similar to `camera_viewer`). Use only if you need a quick viewer; `camera_viewer` is preferred.
- **Run**:
  ```bash
  python3 src/mam_eurobot_2026/mam_eurobot_2026/vision/show_front_camera.py
  ```
