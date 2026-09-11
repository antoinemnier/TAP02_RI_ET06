#!/usr/bin/env python3

from pathlib import Path
import csv
import yaml

import rclpy
from rclpy.node import Node

from moveit_msgs.msg import AttachedCollisionObject
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.srv import ApplyPlanningScene

from attach_workpiece import read_scene
from setup_scene import call_service
from execute_pick_approach import check_position


def main():
    folder = Path(__file__).resolve().parents[1]

    data = yaml.safe_load(
        (folder / "config/key_poses.yaml").read_text()
    )

    profile_file = (
        folder
        / "results"
        / "retimed"
        / "quintic_blue_place_joints.csv"
    )

    with profile_file.open() as file:
        rows = list(csv.DictReader(file))

    if not rows:
        raise RuntimeError("Profil de depot vide.")

    target = [
        float(rows[-1][f"q{joint}"])
        for joint in range(1, 7)
    ]

    rclpy.init()
    node = Node("et06_detach_piece")

    try:
        reader = node.create_client(
            GetPlanningScene,
            "/get_planning_scene",
        )

        writer = node.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )

        check_position(
            node,
            data["joint_names"],
            target,
            0.001,
        )

        scene = read_scene(node, reader)

        matches = [
            item
            for item in scene.robot_state.attached_collision_objects
            if item.object.id == "et06_piece"
        ]

        if len(matches) != 1:
            raise RuntimeError(
                "La piece attachee est absente ou dupliquee."
            )

        attached = matches[0]

        if attached.link_name != "tool0":
            raise RuntimeError(
                "La piece est attachee a un lien inattendu."
            )

        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True

        # Retirer la piece des objets attaches.
        remove_attached = AttachedCollisionObject()
        remove_attached.link_name = attached.link_name
        remove_attached.object.id = attached.object.id
        remove_attached.object.operation = CollisionObject.REMOVE

        request.scene.robot_state.attached_collision_objects = [
            remove_attached
        ]

        # Remettre dans le monde l'objet fourni par MoveIt.
        # Sa pose doit deja etre exprimee dans le repere global.
        world_object = attached.object
        world_object.operation = CollisionObject.ADD
        request.scene.world.collision_objects = [world_object]

        response = call_service(
            node,
            writer,
            request,
        )

        if not response.success:
            raise RuntimeError(
                "La scene a refuse le detachement."
            )

        final_scene = read_scene(node, reader)

        still_attached = any(
            item.object.id == "et06_piece"
            for item in final_scene.robot_state.attached_collision_objects
        )

        world_matches = [
            item
            for item in final_scene.world.collision_objects
            if item.id == "et06_piece"
        ]

        if still_attached:
            raise RuntimeError(
                "La piece est encore attachee."
            )

        if len(world_matches) != 1:
            raise RuntimeError(
                "La piece n'a pas ete remise correctement dans le monde."
            )

        obj = world_matches[0]

        print("Detachement confirme.")
        print("La piece est maintenant un objet du monde.")
        print(
            "Repere de la piece : "
            f"{obj.header.frame_id}"
        )
        print(
            "Position de la piece : "
            f"[{obj.pose.position.x:.6f}, "
            f"{obj.pose.position.y:.6f}, "
            f"{obj.pose.position.z:.6f}]"
        )
        print("Aucun mouvement commande.")

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
