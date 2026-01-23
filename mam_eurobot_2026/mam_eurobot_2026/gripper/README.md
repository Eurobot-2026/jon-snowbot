# Gripper Nodes
Reference for the ROS 2 nodes in `mam_eurobot_2026/gripper`: what each one does, its topics/parameters, and how to launch it.

## gripper_keyboard
- **Node**: `gripper_keyboard` – keyboard teleop that toggles the gripper open/close.
- **Inputs**: keypresses `g` (toggle) and `q` (quit).
- **Outputs**: publishes `std_msgs/Float64` to `command_topic` (default `/gripper/cmd_pos`).
- **Key parameters**: `command_topic`, `open_position`, `closed_position`.
- **Run**:
  ```bash
  ros2 run mam_eurobot_2026 gripper_keyboard
  ```
