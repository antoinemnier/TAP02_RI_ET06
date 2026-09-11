#!/usr/bin/env python3
from pathlib import Path
import yaml
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPlanningScene, GetCartesianPath
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

    rclpy.init()
    node = Node("et06_lift_piece")

    try:
        reader = node.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        cartesian = node.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )
        executor = ActionClient(
            node, ExecuteTrajectory, "/execute_trajectory"
        )

        # Relever les articulations ET les objets attaches.
        scene = read_scene(node, reader)

        attached = any(
            obj.object.id == "et06_piece" and obj.link_name == "tool0"
            for obj in scene.robot_state.attached_collision_objects
        )

        if not attached:
            raise RuntimeError("La piece n'est pas attachee a tool0.")

        # Ce programme est prevu pour commencer a PICK.
        check_position(node, names, data["pick"]["joints"], 0.001)

        state = scene.robot_state
        positions = dict(zip(
            state.joint_state.name,
            state.joint_state.position,
        ))
        start = [positions[name] for name in names]

        request = GetCartesianPath.Request()
        request.header.frame_id = data["frame_id"]
        request.group_name = data["planning_group"]
        request.link_name = data["tool_link"]

        # Etat complet, incluant explicitement la piece attachee.
        request.start_state = state
        request.start_state.is_diff = False

        request.max_step = 0.001
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        target = Pose()
        target.position.x = float(data["pre_pick"]["position"][0])
        target.position.y = float(data["pre_pick"]["position"][1])
        target.position.z = float(data["pre_pick"]["position"][2])

        orientation = data["pre_pick"]["quaternion_xyzw"]
        target.orientation.x = float(orientation[0])
        target.orientation.y = float(orientation[1])
        target.orientation.z = float(orientation[2])
        target.orientation.w = float(orientation[3])

        request.waypoints = [target]

        response = call_service(node, cartesian, request)
        trajectory = response.solution
        points = trajectory.joint_trajectory.points

        print(f"Code resultat : {response.error_code.val}")
        print(f"Fraction calculee : {response.fraction:.8f}")
        print(f"Nombre de points : {len(points)}")

        if (
            response.error_code.val != 1
            or response.fraction < 1.0 - 1e-9
            or len(points) < 2
        ):
            raise RuntimeError("Decollage incomplet : aucune execution.")

        duration = (
            points[-1].time_from_start.sec
            + points[-1].time_from_start.nanosec * 1e-9
        )
        print(f"Duree MoveIt : {duration:.6f} s")
        print("SIMULATION UNIQUEMENT : montee avec la piece attachee.")

        if os.environ.get("ET06_AUTO_CONFIRM") != "1":
            answer = input(
                "Taper OUI pour executer le decollage : "
            )
            if answer.strip().upper() != "OUI":
                raise RuntimeError("Execution annulee par l'utilisateur.")
        else:
            print("Execution autorisee par le programme principal.")

        # Ne pas executer si le robot a bouge depuis le calcul.
        check_position(node, names, start, 0.001)

        execute(node, executor, trajectory)

        # Verifier la configuration finale dans le bon ordre.
        final_positions = dict(zip(
            trajectory.joint_trajectory.joint_names,
            points[-1].positions,
        ))
        endpoint = [final_positions[name] for name in names]
        check_position(node, names, endpoint, 0.001)

        print("Decollage termine.")
        print("La piece reste attachee. Aucun transfert vers PLACE effectue.")

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nProgramme interrompu.")
        raise SystemExit(130)
    except Exception as error:
        print(f"\nARRET : {error}")
        raise SystemExit(1)
