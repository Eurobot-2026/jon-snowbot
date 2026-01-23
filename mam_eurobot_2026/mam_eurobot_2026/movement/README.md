# Movement Nodes
Reference for the ROS 2 nodes in `mam_eurobot_2026/movement`: what each one does, its topics/parameters, and how to launch it.

## path_follow_client
- **Node**: `path_follow_client` – listens for `nav_msgs/Path` messages and forwards them to Nav2's FollowPath action.
- **Inputs**:
  - `/planned_path` (`nav_msgs/Path` by default).
- **Outputs**: FollowPath action requests to `/follow_path` by default.
- **Key parameters**: `path_topic`, `follow_action`, `controller_id`, `goal_checker_id`, `progress_checker_id`, `cancel_on_new_path`.
- **Notes**: If `cancel_on_new_path` is true, a new path cancels the current FollowPath goal before sending the next one. `progress_checker_id` is declared but currently not set on the goal.
- **Run**:
  ```bash
  ros2 run mam_eurobot_2026 path_follow_client
  ```
