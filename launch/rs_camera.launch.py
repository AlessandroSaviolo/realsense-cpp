import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def find_ws_directory(package_name):
    install_dir = get_package_share_directory(package_name)
    ws_dir = os.path.abspath(os.path.join(install_dir, '..', '..', '..', '..'))
    return ws_dir

def generate_launch_description():
	launch_args = [
		DeclareLaunchArgument(name="frame_id", default_value="camera_link"),
		DeclareLaunchArgument(name="width",    default_value="640"),
		DeclareLaunchArgument(name="height",   default_value="480"),
		DeclareLaunchArgument(name="fps",      default_value="30"),
	]

	frame_id = LaunchConfiguration('frame_id')
	ws_path = find_ws_directory('realsense-cpp')
	width = LaunchConfiguration('width')
	height = LaunchConfiguration('height')
	fps = LaunchConfiguration('fps')

	rs_node = ComposableNode(
		package="realsense-cpp",
		name="realsense_camera",
		namespace='realsense_camera',
		plugin="realsense_camera::RSCameraNodelet",
		parameters=[
			{'frame_id':  frame_id},
			{'ws_path':   ws_path},
			{'width':     width},
			{'height':    height},
			{'fps':       fps},
		],
	)

	ld = LaunchDescription(launch_args)
	c = ComposableNodeContainer(
		name='realsense_camera_container',
		namespace='realsense_camera',
		package='rclcpp_components',
		executable='component_container',
		composable_node_descriptions=[rs_node],
		output='screen',
		emulate_tty=True
	)
	ld.add_action(c)

	return ld
