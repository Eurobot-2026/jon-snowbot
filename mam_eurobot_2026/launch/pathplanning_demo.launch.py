import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import yaml

MAC = True
current_display = os.environ.get('DISPLAY', ':0')  # get current DISPLAY
print(f"Detected DISPLAY: {current_display}")

def _load_footprint(robot_model_path: str) -> List[List[float]]:
    try:
        with open(robot_model_path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
    except (OSError, yaml.YAMLError) as exc:
        print(f"[bringup_path_demo] Failed to load robot_model.yaml: {exc}")
        data = {}

    length = float(data.get("footprint_length_m", 0.40))
    width = float(data.get("footprint_width_m", 0.30))
    padding = float(data.get("footprint_padding_m", 0.0))

    length += 2.0 * padding
    width += 2.0 * padding

    return [
        [length / 2.0, width / 2.0],
        [length / 2.0, -width / 2.0],
        [-length / 2.0, -width / 2.0],
        [-length / 2.0, width / 2.0],
    ]


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("mam_eurobot_2026")
    path_planning_share = os.path.join(pkg_share, "path_planning")
    model_path = PathJoinSubstitution([pkg_share, 'models'])

    nav2_params = os.path.join(path_planning_share, "nav2_params.yaml")
    map_yaml = os.path.join(path_planning_share, "field.yaml")
    objects_yaml = os.path.join(path_planning_share, "objects.yaml")
    robot_model_yaml = os.path.join(path_planning_share, "robot_model.yaml")
    rviz_config = os.path.join(path_planning_share, "path_demo_with_map.rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    footprint = _load_footprint(robot_model_yaml)
    footprint_str = str(footprint)
    global_costmap_override = {"global_costmap": {"ros__parameters": {"footprint": footprint_str}}}
    local_costmap_override = {"local_costmap": {"ros__parameters": {"footprint": footprint_str}}}

    if MAC:
        env_actions = [
            SetEnvironmentVariable('DISPLAY', ':1'),  # VNC / Xvfb
            SetEnvironmentVariable('HOME', '/home/rosdev'),
            SetEnvironmentVariable('ROS_LOG_DIR', '/home/rosdev/.ros/log'),
            SetEnvironmentVariable('XDG_RUNTIME_DIR', '/tmp/runtime-rosdev'),
            SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
            SetEnvironmentVariable('IGN_RENDER_ENGINE', 'ogre2'),
            # make path of resource of Gazebo 
            SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', pkg_share),
            SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', model_path),
            ExecuteProcess(
                cmd=[
                    '/bin/bash', '-lc',
                    'mkdir -p /home/rosdev/.ros/log '
                    '&& mkdir -p /tmp/runtime-rosdev '
                    '&& chmod 700 /tmp/runtime-rosdev'
                ],
                output='screen'
            ),
        ]
    else:
        env_actions = [
            SetEnvironmentVariable('DISPLAY', current_display), # use current DISPLAY
            SetEnvironmentVariable('HOME', '/home/rosdev'),
            SetEnvironmentVariable('ROS_LOG_DIR', '/home/rosdev/.ros/log'),
            SetEnvironmentVariable('XDG_RUNTIME_DIR', '/tmp/runtime-rosdev'),
            SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
            SetEnvironmentVariable('IGN_RENDER_ENGINE', 'ogre2'),
            # make path of resource of Gazebo 
            SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', pkg_share),
            SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', model_path),
            ExecuteProcess(
                cmd=[
                    '/bin/bash', '-lc',
                    'mkdir -p /home/rosdev/.ros/log '
                    '&& mkdir -p /tmp/runtime-rosdev '
                    '&& chmod 700 /tmp/runtime-rosdev'
                ],
                output='screen'
            ),
        ]

    return LaunchDescription(
        [
            *env_actions,
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{"yaml_filename": map_yaml, "use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[nav2_params, {"use_sim_time": use_sim_time}, global_costmap_override, local_costmap_override],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_path_planning",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": ["map_server", "planner_server"],
                    }
                ],
            ),
            Node(
                package="mam_eurobot_2026",
                executable="staging_path_planner_node.py",
                name="staging_path_planner",
                output="screen",
                parameters=[{"objects_yaml": objects_yaml}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
