from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("pose_topic", default_value="/model/simple_robot/pose"),
            DeclareLaunchArgument("gz_pose_type", default_value="ignition.msgs.Pose"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("publish_rate_hz", default_value="2.0"),
            # DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gz_pose_bridge",
                output="screen",
                arguments=[
                    [
                        LaunchConfiguration("pose_topic"),
                        "@geometry_msgs/msg/Pose@",
                        LaunchConfiguration("gz_pose_type"),
                    ],
                ],
            ),
            Node(
                package="mam_eurobot_2026",
                executable="gt_pose_to_map_tf",
                name="gt_pose_to_map_tf",
                output="screen",
                parameters=[
                    {"pose_topic": LaunchConfiguration("pose_topic")},
                    {"map_frame": LaunchConfiguration("map_frame")},
                    {"base_frame": LaunchConfiguration("base_frame")},
                    {"publish_rate_hz": LaunchConfiguration("publish_rate_hz")},
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
