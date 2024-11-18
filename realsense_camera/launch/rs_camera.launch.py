import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def find_workspace_directory(package_name):
    # Locate the workspace directory for a given package.
    install_dir = get_package_share_directory(package_name)
    return os.path.abspath(os.path.join(install_dir, '..', '..', '..', '..'))

def generate_launch_description():
    # Declare launch arguments
    launch_arguments = [
        DeclareLaunchArgument("frame_id", default_value="camera_link", description="Frame ID for the camera."),
        DeclareLaunchArgument("width", default_value="640", description="Image width."),
        DeclareLaunchArgument("height", default_value="480", description="Image height."),
        DeclareLaunchArgument("fps", default_value="30", description="Frames per second."),
    ]

    # Extract launch configurations
    frame_id = LaunchConfiguration("frame_id")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    fps = LaunchConfiguration("fps")

    # Locate workspace directory
    workspace_path = find_workspace_directory("realsense_camera")

    # Define the realsense node
    realsense_node = ComposableNode(
        package="realsense_camera",
        name="realsense_camera",
        namespace="realsense_camera",
        plugin="realsense_camera::RSCameraNodelet",
        parameters=[
            {"frame_id": frame_id},
            {"ws_path": workspace_path},
            {"width": width},
            {"height": height},
            {"fps": fps},
        ],
    )

    # Define the container for the composable node
    node_container = ComposableNodeContainer(
        name="realsense_camera_container",
        namespace="realsense_camera",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[realsense_node],
        output="screen",
        emulate_tty=True,
    )

    # Assemble the launch description
    launch_description = LaunchDescription(launch_arguments)
    launch_description.add_action(node_container)

    return launch_description
