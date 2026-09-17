import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Charge TOUTE la configuration, y compris OMPL
    moveit_config = MoveItConfigsBuilder(
        "fanuc", package_name="et06_m10ia_moveit_config"
    ).planning_pipelines(pipelines=["ompl"]).to_moveit_configs()
    
    script_path = os.path.join(os.getcwd(), 'src/et06_pick_place/scripts/get_jacobian.py')

    jacobian_node = Node(
        executable='python3',
        arguments=[script_path],
        output='screen',
        # On passe absolument tous les paramètres générés par le builder
        parameters=[moveit_config.to_dict()]
    )
    
    return LaunchDescription([jacobian_node])
