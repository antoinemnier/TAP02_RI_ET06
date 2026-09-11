#!/usr/bin/env python3
from pathlib import Path
import json
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPlanningScene, GetCartesianPath

from attach_piece import read_scene
from check_scene import call_service
from execute_pick import check_position


def main():
    folder = Path(__file__).resolve().parents[1]
    data = yaml.safe_load(
        (folder / "config/key_poses.yaml").read_text()
    )

    rclpy.init()
    node = Node("et06_cartesian_place")

    try:
        reader = node.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        cartesian = node.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )

        check_position(
            node,
            data["joint_names"],
            data["pre_place"]["joints"],
            0.001,
        )

        scene = read_scene(node, reader)

        attached = any(
            obj.object.id == "et06_piece" and obj.link_name == "tool0"
            for obj in scene.robot_state.attached_collision_objects
        )

        if not attached:
            raise RuntimeError("La piece n'est pas attachee.")

        request = GetCartesianPath.Request()
        request.header.frame_id = data["frame_id"]
        request.group_name = data["planning_group"]
        request.link_name = data["tool_link"]

        # Etat complet : articulations et piece attachee.
        request.start_state = scene.robot_state
        request.start_state.is_diff = False

        request.max_step = 0.001
        request.avoid_collisions = True

        # Timing provisoire, remplace ensuite par notre profil bleu.
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        start = data["pre_place"]["position"]
        end = data["place"]["position"]
        orientation = data["place"]["quaternion_xyzw"]

        for fraction in (0.25, 0.50, 0.75, 1.0):
            pose = Pose()
            pose.position.x = start[0] + fraction * (end[0] - start[0])
            pose.position.y = start[1] + fraction * (end[1] - start[1])
            pose.position.z = start[2] + fraction * (end[2] - start[2])

            pose.orientation.x = float(orientation[0])
            pose.orientation.y = float(orientation[1])
            pose.orientation.z = float(orientation[2])
            pose.orientation.w = float(orientation[3])

            request.waypoints.append(pose)

        response = call_service(node, cartesian, request)
        trajectory = response.solution.joint_trajectory
        points = trajectory.points

        print(f"Code resultat : {response.error_code.val}")
        print(f"Fraction calculee : {response.fraction:.8f}")
        print(f"Nombre de points : {len(points)}")

        if (
            response.error_code.val != 1
            or response.fraction < 1.0 - 1e-9
            or len(points) < 2
        ):
            print("Approche incomplete : aucune execution.")
            print("Conserver les journaux MoveIt pour le diagnostic.")
            return

        record = {
            "description": "pre_place -> place, piece attachee",
            "frame_id": data["frame_id"],
            "fraction": response.fraction,
            "joint_names": list(trajectory.joint_names),
            "points": [
                {
                    "time_s": (
                        p.time_from_start.sec
                        + p.time_from_start.nanosec * 1e-9
                    ),
                    "positions": list(p.positions),
                    "velocities": list(p.velocities),
                    "accelerations": list(p.accelerations),
                }
                for p in points
            ],
        }

        output = folder / "results/cartesian"
        output.mkdir(parents=True, exist_ok=True)
        filename = output / "pre_place_to_place.json"
        filename.write_text(
            json.dumps(record, indent=2, allow_nan=False)
        )

        print(f"Chemin sauvegarde : {filename}")
        print("Timing provisoire : profil bleu pas encore applique.")
        print("Aucun mouvement commande. Piece toujours attachee.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

