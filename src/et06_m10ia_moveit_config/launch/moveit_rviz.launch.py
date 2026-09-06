from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

import os

def generate_launch_description():
	package_name = "et06_m10ia_moveit_config"

	moveit_config = (
		MoveItConfigsBuilder(
			"fanuc",
			package_name=package_name,
		)
		.to_moveit_configs()
	)

	rviz_config = os.path.join(
		get_package_share_directory(package_name),
		"config",
		"moveit.rviz",
	)

	rviz_node = Node(
		package="rviz2",
		executable="rviz2",
		name="rviz2",
		output="screen",
		arguments=["-d", rviz_config],
		parameters=[
			moveit_config.robot_description,
			moveit_config.robot_description_semantic,
			moveit_config.robot_description_kinematics,
		],
	)

	return LaunchDescription([rviz_node])

