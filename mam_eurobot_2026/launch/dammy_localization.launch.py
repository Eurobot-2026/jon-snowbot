from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("world_name", default_value="eurobot_2026_arena"),
            DeclareLaunchArgument("target_model_name", default_value="simple_robot"),
            DeclareLaunchArgument("out_topic", default_value="/model/simple_robot/pose_gt"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("publish_rate_hz", default_value="2.0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gz_pose_bridge",
                output="screen",
                arguments=[
                    [
                        LaunchConfiguration("out_topic"),
                        "@geometry_msgs/msg/Pose@ignition.msgs.Pose",
                    ],
                ],
            ),
            Node(
                package="mam_eurobot_2026",
                executable="pose_info_filter_gz",
                name="pose_info_filter_gz",
                output="screen",
                parameters=[
                    {"world_name": LaunchConfiguration("world_name")},
                    {"target_model_name": LaunchConfiguration("target_model_name")},
                    {"out_topic": LaunchConfiguration("out_topic")},
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
            Node(
                package="mam_eurobot_2026",
                executable="gt_pose_to_map_tf",
                name="gt_pose_to_map_tf",
                output="screen",
                parameters=[
                    {"pose_topic": LaunchConfiguration("out_topic")},
                    {"map_frame": LaunchConfiguration("map_frame")},
                    {"base_frame": LaunchConfiguration("base_frame")},
                    {"publish_rate_hz": LaunchConfiguration("publish_rate_hz")},
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
