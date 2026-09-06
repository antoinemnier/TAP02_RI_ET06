import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory(
        "moveit_resources_fanuc_description"
    )

    urdf_file = os.path.join(
        package_share,
        "urdf",
        "fanuc.urdf",
    )

    with open(urdf_file, "r", encoding="utf-8") as urdf:
        robot_description = urdf.read()

    common_parameters = (
        dict(robot_description=robot_description),
    )

    nodes = (
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=list(common_parameters),
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            parameters=list(common_parameters),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
        ),
    )

    return LaunchDescription(nodes)
