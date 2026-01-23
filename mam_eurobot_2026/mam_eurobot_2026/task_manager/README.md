# Task Manager Nodes
Reference for the ROS 2 nodes in `mam_eurobot_2026/task_manager`: what each one does, its topics/parameters, and how to launch it.

## task_manager_fsm
- **Node**: `task_manager_fsm` – finite state machine that orchestrates the match flow: move the cursor, select crates, navigate to pregrasp/place poses, control the gripper, and return to the nest.
- **Inputs**:
  - `/detected_crates` (`vision_msgs/Detection3DArray`) – crate IDs + poses.
  - `/world_coordinate/cursor` (`geometry_msgs/PointStamped`, BEST_EFFORT) – cursor position in map/world.
  - `/joint_states` (`sensor_msgs/JointState`) – gripper joint position for grasp verification.
  - `/gripper_has_object` (`std_msgs/Bool`) – optional boolean grasp feedback.
  - TF: `<global_frame>` → `<base_frame>` (defaults `map` → `base_link`) for robot pose.
- **Outputs**:
  - `/task_state` (`std_msgs/String`) – state machine status string (`STATE|crate=...|pantry=...|reason=...`).
  - `<task_goal_topic>` (`geometry_msgs/PoseStamped`, default `/task_goal`) – published navigation goal used by the upstream controller.
  - `cmd_vel` (`geometry_msgs/Twist`) – short alignment/back-up motions during grasp/release steps.
- **Key parameters** (override with `--ros-args -p`):
  - Frames: `global_frame` (`map`), `base_frame` (`base_link`).
  - Navigation: `nav_action_name` (`navigate_to_pose`), `task_goal_topic` (`/task_goal`).
  - Pantry config: `pantry_yaml` (path or YAML string), `pantry_region_xy` ([0.2, 0.2]).
  - Cursor: `cursor_topic`, `cursor_grab_duration`, `cursor_grab_speed`, `cursor_release_duration`, `cursor_release_speed`.
  - Nest: `nest_x`, `nest_y`, `nest_yaw`.
  - Gripper: `gripper_command_topic`, `gripper_open_position`, `gripper_closed_position`, `gripper_joint`, `gripper_closed_pos`, `gripper_closed_tol`.
  - Timing: `state_timer_period`, `align_duration`, `backup_duration`, `gripper_wait`, `place_wait`, `verify_wait`.
  - Timeouts: `timeout_move_cursor`, `timeout_select_crate`, `timeout_nav`, `timeout_align`, `timeout_hook`, `timeout_gripper`, `timeout_verify`, `timeout_return`.
- **State flow** (high level): `INIT → MOVE_CURSOR → NAV_TO_CURSOR → HOOK_GRAB → NAV_CURSOR_TO_TARGET → HOOK_RELEASE → SELECT_CRATE → NAV_TO_PREGRASP → ALIGN_TO_CRATE → GRIPPER_CLOSE → VERIFY_GRASP → NAV_TO_PLACE → PLACE_RELEASE → VERIFY_PLACE → RETURN_TO_NEST → DONE` (with `RECOVERY` for failed grasps).
- **Run**:
  ```bash
  ros2 run mam_eurobot_2026 task_manager_fsm \
    --ros-args -p pantry_yaml:=mam_eurobot_2026/mam_eurobot_2026/task_manager/pantry.yaml
  ```
