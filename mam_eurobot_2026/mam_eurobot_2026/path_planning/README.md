# Path Planning Nodes
Reference for the ROS 2 nodes in `mam_eurobot_2026/path_planning`: what each one does, its topics/parameters, and how to launch it.

## staging_path_planner
- **Node**: `staging_path_planner` – reads crate staging poses from `objects.yaml`, selects the nearest target to the robot, and requests a Nav2 plan to it on a fixed rate.
- **Inputs**:
  - TF: `<global_frame>` → `<base_frame>` (defaults `map` → `base_link`) for current robot pose.
  - `objects_yaml` file (required to load staging poses).
- **Outputs**: `/planned_path` (`nav_msgs/Path` by default).
- **Key parameters**: `objects_yaml`, `plan_rate_hz`, `tf_timeout_sec`, `planner_action_name` (`/compute_path_to_pose`), `planner_id` (`GridBased`), `output_path_topic`, `base_frame`, `global_frame`.
- **Run**:
  ```bash
  ros2 run mam_eurobot_2026 staging_path_planner_node \
    --ros-args -p objects_yaml:=mam_eurobot_2026/mam_eurobot_2026/path_planning/objects.yaml
  ```

## task_goal_path_planner
- **Node**: `task_goal_path_planner` – listens to a goal pose on `/task_goal`, transforms it into the global frame if needed, and requests a Nav2 plan to publish as a `nav_msgs/Path`.
- **Inputs**:
  - `/task_goal` (`geometry_msgs/PoseStamped` by default).
  - TF: `<global_frame>` → `<base_frame>` for current robot pose; TF from goal frame → `<global_frame>` if the goal is not already in the global frame.
- **Outputs**: `/planned_path` (`nav_msgs/Path` by default).
- **Key parameters**: `goal_topic`, `output_path_topic`, `planner_action_name`, `planner_id`, `base_frame`, `global_frame`, `plan_rate_hz`, `tf_timeout_sec`.
- **Run**:
  ```bash
  ros2 run mam_eurobot_2026 task_goal_path_planner_node
  ```

## fixed_goal_path_planner
- **Node**: `fixed_goal_path_planner` – publishes a plan to a fixed (x, y, yaw) goal, intended for quick testing.
- **Inputs**:
  - TF: `<global_frame>` → `<base_frame>` for current robot pose.
  - Fixed goal parameters: `goal_x`, `goal_y`, `goal_yaw`.
- **Outputs**: `/planned_path` (`nav_msgs/Path` by default).
- **Key parameters**: `goal_x`, `goal_y`, `goal_yaw`, `output_path_topic`, `planner_action_name`, `planner_id`, `base_frame`, `global_frame`, `plan_rate_hz`, `tf_timeout_sec`, `plan_once`.
- **Run**:
  ```bash
  ros2 run mam_eurobot_2026 fixed_goal_path_planner_node --ros-args -p goal_x:=0.5 -p goal_y:=0.2
  ```
