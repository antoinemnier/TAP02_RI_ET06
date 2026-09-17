#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# Dans ROS 2 Jazzy, l'interface Python officielle est moveit_py
from moveit.planning import MoveItPy

def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Initialisation (MoveIt doit tourner en fond avec demo.launch.py)
        fanuc = MoveItPy(node_name="jacobian_node")
        robot_state = fanuc.get_robot_model().create_robot_state()
        
        # Positions de la pose PICK
        robot_state.joint_positions = {
            'joint_1': -0.3805, 'joint_2': -0.0087, 'joint_3': -0.4613,
            'joint_4': 0.0, 'joint_5': -1.1182, 'joint_6': 0.3805
        }
        robot_state.update()
        
        # Calcul par le solveur KDL de MoveIt
        jacobian = robot_state.get_jacobian("manipulator", [0.0, 0.0, 0.0])
        print("\n=== JACOBIEN MOVEIT 2 (KDL) ===")
        print(jacobian)
        
    except Exception as e:
        print(f"Erreur d'initialisation MoveItPy : {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
