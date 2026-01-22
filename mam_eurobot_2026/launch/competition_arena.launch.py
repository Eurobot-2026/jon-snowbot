# test launch file for Mac (cleaned)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    RegisterEventHandler,
    LogInfo,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os

MAC = False
current_display = os.environ.get('DISPLAY', ':0')
print(f"Detected DISPLAY: {current_display}")


def _spawn_model_cmd(file_uri: str, name: str, x: float, y: float, z: float, Y: float = None):
    cmd = [
        "ros2", "run", "ros_gz_sim", "create",
        "-file", file_uri,
        "-name", name,
        "-x", str(x), "-y", str(y), "-z", str(z),
    ]
    if Y is not None:
        cmd += ["-Y", str(Y)]
    return cmd


def generate_launch_description():

    os.system("pkill -9 -f ign")
    os.system("pkill -9 -f gz")
    # path sharing package
    pkg_share = FindPackageShare('mam_eurobot_2026')
    model_path = PathJoinSubstitution([pkg_share, 'models'])

    # ---------------- Environment ----------------
    if MAC:
        env_actions = [
            SetEnvironmentVariable('DISPLAY', ':1'),
            SetEnvironmentVariable('HOME', '/home/rosdev'),
            SetEnvironmentVariable('ROS_LOG_DIR', '/home/rosdev/.ros/log'),
            SetEnvironmentVariable('XDG_RUNTIME_DIR', '/tmp/runtime-rosdev'),
            SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
            SetEnvironmentVariable('IGN_RENDER_ENGINE', 'ogre2'),
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
            SetEnvironmentVariable('DISPLAY', current_display),
            SetEnvironmentVariable('HOME', '/home/rosdev'),
            SetEnvironmentVariable('ROS_LOG_DIR', '/home/rosdev/.ros/log'),
            SetEnvironmentVariable('XDG_RUNTIME_DIR', '/tmp/runtime-rosdev'),
            SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
            SetEnvironmentVariable('IGN_RENDER_ENGINE', 'ogre2'),
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

    # ---------------- World argument ----------------
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([pkg_share, 'worlds', 'competition_world.sdf']),
        description='SDF world file'
    )
    world_cfg = LaunchConfiguration('world')
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'simple_robot_camera.urdf'])
    robot_description = ParameterValue(Command(['cat ', urdf_path]), value_type=str)

    # ---------------- Start Gazebo ----------------
    ign = ExecuteProcess(
        cmd=["ign", "gazebo", world_cfg],
        output="screen",
    )

    # ---------------- Spawn robot ----------------
    spawn_after_ign = RegisterEventHandler(
        OnProcessStart(
            target_action=ign,
            on_start=[
                ExecuteProcess(
                    cmd=_spawn_model_cmd(
                        file_uri="model://models/simple_robot",
                        name="simple_robot",
                        x=0.80, y=-1.20, z=0.05, Y=3.1415
                    ),
                    output="screen",
                ),
            ],
        )
    )

    # ---------------- Wheel velocity bridges ----------------
    # wheel_bridges = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='wheel_velocity_bridges',
    #     output='screen',
    #     arguments=[
    #         '/omni/front_left_speed@std_msgs/msg/Float64@gz.msgs.Double',
    #         '/omni/front_right_speed@std_msgs/msg/Float64@gz.msgs.Double',
    #         '/omni/rear_left_speed@std_msgs/msg/Float64@gz.msgs.Double',
    #         '/omni/rear_right_speed@std_msgs/msg/Float64@gz.msgs.Double',
    #     ],
    # )
    mecanum_drive_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='mecanum_drive_bridge',
        output='screen',
        arguments=[
            # Command Velocity: ROS 2 -> Gazebo
            '/model/simple_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # Odometry Feedback: Gazebo -> ROS 2
            '/model/simple_robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        # Optional: Remap the ROS 2 topic to a more conventional name if needed
        # remappings=[
        #     ('/model/simple_robot/cmd_vel', '/cmd_vel'),
        #     ('/model/simple_robot/odometry', '/odom'),
        # ],
    )
    
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        output='screen',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
    )

    gripper_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gripper_bridge',
        output='screen',
        arguments=[
            '/gripper/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double',
        ],
    )

    # ---------------- Bridges: camera ----------------
    img_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='image_bridge_front_camera',
        arguments=['/front_camera'],
        remappings=[
            ('image', '/front_camera/image'),
            ('camera_info', '/front_camera/camera_info'),
        ],
        parameters=[{'qos': 'sensor_data'}],
        output='screen',
    )

    # top_img_bridge_1 =  Node( 
    #     package='ros_gz_image',
    #     executable='image_bridge',
    #     name='image_bridge_top_camera',
    #     arguments=['/top_camera/image_1'], 
    #     parameters=[{'qos': 'sensor_data'}],  # BESTEFFORT VOLATILE shallow depth
    #     output='screen',
    # )
    # top_img_bridge_2 =  Node( 
    #     package='ros_gz_image',
    #     executable='image_bridge',
    #     name='image_bridge_top_camera',
    #     arguments=['/top_camera/image_2'], 
    #     parameters=[{'qos': 'sensor_data'}],  # BESTEFFORT VOLATILE shallow depth
    #     output='screen',
    # )
    top_img_bridge_3 =  Node( 
        package='ros_gz_image',
        executable='image_bridge',
        name='image_bridge_top_camera',
        arguments=['/top_camera/image_3'], 
        parameters=[{'qos': 'sensor_data'}],  # BESTEFFORT VOLATILE shallow depth
        output='screen',
    )
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_camera_bridge',
        output='screen',
        arguments=[
            '/world/eurobot_2026_arena/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
        ],
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # ---------------- RViz ----------------
    rviz_env = {
        'DISPLAY': current_display if not MAC else ':1',
        'LIBGL_ALWAYS_SOFTWARE': '1',
        'LD_LIBRARY_PATH': '/opt/ros/humble/opt/rviz_ogre_vendor/lib:' + os.environ.get('LD_LIBRARY_PATH', ''),
        'HOME': '/home/rosdev',
        'ROS_LOG_DIR': '/home/rosdev/.ros/log',
        'XDG_RUNTIME_DIR': '/tmp/runtime-rosdev',
        'RCUTILS_LOGGING_USE_STDOUT': '1',
        'AMENT_PREFIX_PATH': os.environ.get('AMENT_PREFIX_PATH', ''),
    }

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        env=rviz_env,
    )

    return LaunchDescription([
        LogInfo(msg=['pkg_share: ', pkg_share]),
        LogInfo(msg=['model_path: ', model_path]),
        *env_actions,
        world_arg,
        ign,
        spawn_after_ign,
        img_bridge,
        # top_img_bridge_1,
        # top_img_bridge_2,
        top_img_bridge_3,
        camera_bridge,
        mecanum_drive_bridge,
        gripper_bridge,
        cmd_vel_bridge,
        # wheel_bridges,     # <—— added
        robot_state_publisher,
        rviz,
    ])
