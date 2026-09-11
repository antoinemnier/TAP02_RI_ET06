#!/usr/bin/env python3
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity


def call_service(node, client, request):
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(f"Service indisponible : {client.srv_name}")

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)

    if not future.done():
        raise RuntimeError(f"Delai depasse : {client.srv_name}")

    result = future.result()
    if result is None:
        raise RuntimeError(f"Aucune reponse : {client.srv_name}")

    return result


def box(frame, name, center, dimensions):
    obj = CollisionObject()
    obj.header.frame_id = frame
    obj.id = name
    obj.operation = CollisionObject.ADD

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(v) for v in dimensions]

    pose = Pose()
    pose.position.x = float(center[0])
    pose.position.y = float(center[1])
    pose.position.z = float(center[2])
    pose.orientation.w = 1.0

    obj.primitives = [primitive]
    obj.primitive_poses = [pose]
    return obj


def main():
    config_path = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "key_poses.yaml"
    )
    data = yaml.safe_load(config_path.read_text())

    states = ["home", "pre_pick", "pick", "pre_place", "place"]

    for name in states:
        q = data[name]["joints"]
        if len(q) != len(data["joint_names"]):
            raise ValueError(f"Nombre d'angles incorrect : {name}")

    rclpy.init()
    node = Node("et06_check_scene")

    try:
        apply_client = node.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        validity_client = node.create_client(
            GetStateValidity, "/check_state_validity"
        )

        frame = data["frame_id"]

        objects = [
            box(frame, "et06_pick_support",
                [0.75, -0.30, 0.71], [0.30, 0.30, 0.10]),
            box(frame, "et06_piece",
                [0.75, -0.30, 0.79], [0.06, 0.06, 0.06]),
            box(frame, "et06_place_surface",
                [0.75, 0.30, 0.71], [0.30, 0.30, 0.10]),
            box(frame, "et06_obstacle",
                [0.45, -0.06, 0.975], [0.04, 0.04, 0.35]),
        ]

        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects = objects

        response = call_service(node, apply_client, request)

        if not response.success:
            raise RuntimeError("Echec de l'application de la scene.")

        print("\nScene appliquee : quatre objets ET06.", flush=True)

        all_valid = True

        for name in states:
            request = GetStateValidity.Request()
            request.group_name = data["planning_group"]
            request.robot_state.is_diff = True
            request.robot_state.joint_state.name = data["joint_names"]
            request.robot_state.joint_state.position = [
                float(v) for v in data[name]["joints"]
            ]

            response = call_service(node, validity_client, request)

            status = "VALIDE" if response.valid else "INVALIDE"
            print(f"{name.upper():12s} : {status}", flush=True)

            if not response.valid:
                all_valid = False
                for contact in response.contacts[:10]:
                    print(
                        f"  Contact : {contact.contact_body_1}"
                        f" / {contact.contact_body_2}",
                        flush=True,
                    )

        if all_valid:
            print("\nLes cinq etats sont valides dans cette scene.")
            print("Les trajets entre ces etats restent a verifier.")
        else:
            print("\nScene a ajuster avant de planifier le cycle.")

        print("Aucun mouvement n'a ete commande.")
        print("La piece reste un objet du monde, non attache.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
