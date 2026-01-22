from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    color_detector = Node(
        package="mam_eurobot_2026",
        executable="color_detector",
        name="color_detector",
        output="screen",
        arguments=["--image-topic", "/top_camera/image_3"],
    )

    world_to_topcamera = Node(
        package="mam_eurobot_2026",
        executable="world_to_topcamera",
        name="world_to_topcamera",
        output="screen",
    )

    estimate_cursor = Node(
        package="mam_eurobot_2026",
        executable="estimate_cursor_position",
        name="estimate_cursor_position",
        output="screen",
    )

    # aruco_detector_top_camera = Node(
    #     package="mam_eurobot_2026",
    #     executable="aruco_detector.py",
    #     name="aruco_detector_top",
    #     output="screen",
    #     parameters=[{"image_topic": "/top_camera/image_3"}],
    # ) commented out because this is useless for now. It cant detect aruco on crates from top camera

    aruco_detector_front_camera = Node(
        package="mam_eurobot_2026",
        executable="aruco_detector",
        name="aruco_detector_front",
        output="screen",
        parameters=[{"image_topic": "/front_camera"}],
    )

    return LaunchDescription([
        color_detector,
        world_to_topcamera,
        estimate_cursor,
        # aruco_detector_top_camera,
        aruco_detector_front_camera
    ])
