#!/usr/bin/env python3
from pathlib import Path
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetPlanningScene, GetMotionPlan
from moveit_msgs.action import ExecuteTrajectory

from attach_piece import read_scene
from check_scene import call_service
from execute_pick import check_position, execute


def main():
    folder = Path(__file__).resolve().parents[1]
    data = yaml.safe_load(
        (folder / "config/key_poses.yaml").read_text()
    )
    names = data["joint_names"]
    target = [float(q) for q in data["pre_place"]["joints"]]

    rclpy.init()
    node = Node("et06_transfer_piece")

    try:
        reader = node.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        planner = node.create_client(
            GetMotionPlan, "/plan_kinematic_path"
        )
        executor = ActionClient(
            node, ExecuteTrajectory, "/execute_trajectory"
        )

        # Confirmer que le decollage a deja ete realise.
        check_position(node, names, data["pre_pick"]["joints"], 0.001)

        scene = read_scene(node, reader)

        attached = any(
            obj.object.id == "et06_piece" and obj.link_name == "tool0"
            for obj in scene.robot_state.attached_collision_objects
        )

        if not attached:
            raise RuntimeError("La piece n'est pas attachee a tool0.")

        positions = dict(zip(
            scene.robot_state.joint_state.name,
            scene.robot_state.joint_state.position,
        ))
        start = [positions[name] for name in names]

        request = GetMotionPlan.Request()
        req = request.motion_plan_request

        req.group_name = data["planning_group"]
        req.pipeline_id = "ompl"
        req.planner_id = "RRTConnect"
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 1
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1

        # Etat complet : articulations ET piece attachee.
        req.start_state = scene.robot_state
        req.start_state.is_diff = False

        goal = Constraints()
        goal.name = "pre_place_with_piece"

        for name, position in zip(names, target):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = position
            joint.tolerance_above = 1e-5
            joint.tolerance_below = 1e-5
            joint.weight = 1.0
            goal.joint_constraints.append(joint)

        req.goal_constraints = [goal]

        print("Planification vers PRE_PLACE avec la piece...", flush=True)
        response = call_service(node, planner, request)
        result = response.motion_plan_response
        points = result.trajectory.joint_trajectory.points

        print(f"Code planification : {result.error_code.val}")
        print(f"Temps rapporte : {result.planning_time:.6f} s")
        print(f"Nombre de points : {len(points)}")

        if result.error_code.val != 1 or len(points) < 2:
            print(f"Message : {result.error_code.message}")
            print(f"Source : {result.error_code.source}")
            raise RuntimeError("Planification echouee. Aucune execution.")

        duration = (
            points[-1].time_from_start.sec
            + points[-1].time_from_start.nanosec * 1e-9
        )
        print(f"Duree du transfert : {duration:.6f} s")
        print("SIMULATION UNIQUEMENT. Ne pas modifier la scene.")

        answer = input("Taper OUI pour executer le transfert : ")
        if answer.strip() != "OUI":
            print("Annule : aucun mouvement commande.")
            return

        # Verifier que le robot n'a pas bouge pendant la confirmation.
        check_position(node, names, start, 0.001)

        execute(node, executor, result.trajectory)

        check_position(node, names, target, 0.001)

        final_scene = read_scene(node, reader)
        still_attached = any(
            obj.object.id == "et06_piece" and obj.link_name == "tool0"
            for obj in final_scene.robot_state.attached_collision_objects
        )

        if not still_attached:
            raise RuntimeError("Piece non retrouvee parmi les objets attaches.")

        print("Transfert termine : robot a PRE_PLACE.")
        print("Piece toujours attachee. Depot non effectue.")

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nProgramme interrompu.")
    except Exception as error:
        print(f"\nARRET : {error}")
