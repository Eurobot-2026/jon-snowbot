# Path Planning Demo (Nav2)

This folder contains a minimal Nav2 planner demo that computes a path from the
current `map -> base_link` TF pose to the nearest crate staging target and
publishes a `nav_msgs/Path` on `/planned_path` for RViz.


## Run
```
ros2 launch mam_eurobot_2026 bringup_path_demo.launch.py
```

## Configuration
- Crate staging targets:
  - `mam_eurobot_2026/mam_eurobot_2026/path_planning/objects.yaml`
  - Only `crates[*].staging` is used.
- Robot footprint:
  - `mam_eurobot_2026/mam_eurobot_2026/path_planning/robot_model.yaml`
  - The launch file builds a rectangular polygon (with padding) and injects it
    into both global and local costmaps.
- Nav2 planner parameters:
  - `mam_eurobot_2026/mam_eurobot_2026/path_planning/nav2_params.yaml`

## Maps
`field.yaml` references `field.pgm`. If you generate your own map with SLAM,
update `field.yaml` to point to the correct image.

## Optional: regenerate the sample blank map
```
ros2 run mam_eurobot_2026 gen_blank_map.py
```
