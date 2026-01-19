from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("model_name", default_value="simple_robot"),
            DeclareLaunchArgument("odom_topic", default_value="/model/simple_robot/odometry"),
            DeclareLaunchArgument("publish_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gz_odom_bridge",
                output="screen",
                arguments=[
                    [
                        LaunchConfiguration("odom_topic"),
                        "/model/simple_robot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
                    ],
                ],
            ),
            Node(
                package="mam_eurobot_2026",
                executable="true_tf_from_odom.py",
                name="true_tf_from_odom",
                output="screen",
                parameters=[
                    {"map_frame": LaunchConfiguration("map_frame")},
                    {"base_frame": LaunchConfiguration("base_frame")},
                    {"model_name": LaunchConfiguration("model_name")},
                    {"odom_topic": LaunchConfiguration("odom_topic")},
                    {"publish_rate_hz": LaunchConfiguration("publish_rate_hz")},
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
