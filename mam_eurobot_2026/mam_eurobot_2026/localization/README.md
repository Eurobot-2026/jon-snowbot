# Localization Nodes
Reference for the ROS 2 nodes in `mam_eurobot_2026/localization`: what each one does, its topics/parameters, and how to launch it.

## gt_pose_to_map_tf
- **Node**: `gt_pose_to_map_tf` – listens to a ground-truth `geometry_msgs/Pose` and publishes a TF from `map` to `base_link` at a fixed rate.
- **Inputs**: `pose_topic` (`geometry_msgs/Pose`, default `/model/simple_robot/pose`).
- **Outputs**: TF `<map_frame>` → `<base_frame>` (defaults `map` → `base_link`).
- **Key parameters**: `pose_topic`, `map_frame`, `base_frame`, `publish_rate_hz`.
- **Notes**: Only the latest Pose is used when publishing; earlier poses received within a cycle are overwritten.
- **Run**:
  ```bash
  ros2 run mam_eurobot_2026 gt_pose_to_map_tf
  ```
