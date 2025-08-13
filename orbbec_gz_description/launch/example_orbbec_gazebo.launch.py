# Author: Marq Rasmussen

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_gz(context: LaunchContext):
    gz_world_file = (
        get_package_share_directory("orbbec_gz_description") + "/world/example.sdf"
    )
    # -r is to run the simulation on start
    # -v is the verbose level
    #  0: No output, 1: Error, 2: Error and warning, 3: Error, warning, and info, 4: Error, warning, info, and debug.
    sim_options = "-r -v 3"
    if LaunchConfiguration("headless").perform(context) == "true":
        sim_options += " -s"  # -s is to only run the server (headless mode).
    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", [f"{sim_options} {gz_world_file}"])],
    )
    return [gz_launch_description]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless", default_value="true", description="Launch Gazebo headless?"
        )
    )

    # Initialize Arguments
    launch_rviz = LaunchConfiguration("rviz")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("orbbec_gz_description"), "rviz", "example.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("orbbec_gz_description"),
                    "urdf",
                    "example_335Le_gazebo.urdf.xacro",
                ]
            ),
            " ",
            "camera_name:=gz_camera",
            " ",
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": robot_description_content,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[{"use_sim_time": True}],
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "Gemini335Le_camera",
            "-allow_renaming",
            "true",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
        ],
    )

    # Bridge the camera data to ROS and match the default topics that the real camera would publish
    #
    # Note the gz_topic_name comes from <camera_model>.gazebo.xacro which is set to to 'gz_topic_name="$(arg camera_name)"'
    #
    # Note the rgbd sensor in Gazebo does not return the correct frame_id for the data so
    #   "/gz_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked", does not work correctly!!
    #
    # Note the depth/camera_info is not publishing as expected so no messages are sent from Gz.
    # In this repo we assume that the RGB and Depth sensors are in the same location so that both can share the the same `camera_info`
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "/gz_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/gz_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/gz_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/gz_camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        remappings=[
            (
                "/gz_camera/image",
                "/gz_camera/camera/color/image_raw",
            ),
            (
                "/gz_camera/depth_image",
                "/gz_camera/camera/depth_registered/image_rect",
            ),
            (
                "/gz_camera/camera_info",
                "/gz_camera/camera/color/camera_info",
            ),
            (
                "/gz_camera/depth/camera_info",
                "/gz_camera/camera/depth_registered/camera_info",
            ),
        ],
        output="screen",
    )

    # Because the Point cloud data from Gazebo is wrong, generate it w/ROS given the RGB + Depth images and camera_info
    point_cloud_node = Node(
        package="depth_image_proc",
        executable="point_cloud_xyzrgb_node",
        output="both",
        remappings=[
            ("rgb/image_rect_color", "/gz_camera/camera/color/image_raw"),
            ("rgb/camera_info", "/gz_camera/camera/color/camera_info"),
            (
                "depth_registered/image_rect",
                "/gz_camera/camera/depth_registered/image_rect",
            ),
            ("points", "/gz_camera/camera/depth/color/points"),
        ],
    )

    nodes_to_start = [
        robot_state_publisher_node,
        rviz_node,
        OpaqueFunction(function=launch_gz),
        gz_spawn_entity,
        gazebo_bridge,
        point_cloud_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
