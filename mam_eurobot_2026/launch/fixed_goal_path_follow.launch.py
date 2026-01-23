import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("mam_eurobot_2026")
    path_planning_share = os.path.join(pkg_share, "path_planning")

    nav2_params = os.path.join(path_planning_share, "nav2_params.yaml")
    map_yaml = os.path.join(path_planning_share, "field.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_params_arg = LaunchConfiguration("nav2_params")
    map_yaml_arg = LaunchConfiguration("map_yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("nav2_params", default_value=nav2_params),
            DeclareLaunchArgument("map_yaml", default_value=map_yaml),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{"yaml_filename": map_yaml_arg, "use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[nav2_params_arg, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[nav2_params_arg, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_path_follow",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": ["map_server", "planner_server", "controller_server"],
                    }
                ],
            ),
            Node(
                package="mam_eurobot_2026",
                executable="fixed_goal_path_planner_node",
                name="fixed_goal_path_planner",
                output="screen",
            ),
            Node(
                package="mam_eurobot_2026",
                executable="path_follow_client",
                name="path_follow_client",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
