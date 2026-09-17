import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Charge la configuration de votre robot depuis votre package
    moveit_config = MoveItConfigsBuilder(
        package_name="et06_m10ia_moveit_config"
    ).to_moveit_configs()
    
    # Prépare le chemin vers ton script Python
    script_path = os.path.join(os.getcwd(), 'src/et06_pick_place/scripts/get_jacobian.py')

    # Lance le script avec les paramètres MoveIt
    jacobian_node = Node(
        executable='python3',
        arguments=[script_path],
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ]
    )
    
    return LaunchDescription([jacobian_node])
