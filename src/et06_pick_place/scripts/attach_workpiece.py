#!/usr/bin/env python3
from pathlib import Path
import csv
import yaml

import rclpy
from rclpy.node import Node

from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from moveit_msgs.srv import GetStateValidity

from setup_scene import call_service
from execute_pick_approach import check_position


def read_scene(node, client):
    request = GetPlanningScene.Request()
    request.components.components = (
        PlanningSceneComponents.ROBOT_STATE
        | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    )
    return call_service(node, client, request).scene


def attach_piece(node):
    reader = node.create_client(GetPlanningScene, "/get_planning_scene")
    writer = node.create_client(ApplyPlanningScene, "/apply_planning_scene")

    scene = read_scene(node, reader)

    attached = [
        obj for obj in scene.robot_state.attached_collision_objects
        if obj.object.id == "et06_piece"
    ]

    if attached:
        if attached[0].link_name != "tool0":
            raise RuntimeError("The workpiece is attached to another link.")
        print("The workpiece is already attached to tool0.")
        return scene

    world_ids = [obj.id for obj in scene.world.collision_objects]

    if "et06_piece" not in world_ids:
        raise RuntimeError("The workpiece is missing from the world.")

    obj = AttachedCollisionObject()
    obj.link_name = "tool0"
    obj.object.id = "et06_piece"
    obj.object.operation = CollisionObject.ADD

    # Prise virtuelle : seul tool0 est autorise ici.
    # No contact with the support or the rest of the arm is allowed.
    obj.touch_links = ["tool0"]

    request = ApplyPlanningScene.Request()
    request.scene.is_diff = True
    request.scene.robot_state.is_diff = True
    request.scene.robot_state.attached_collision_objects = [obj]

    response = call_service(node, writer, request)
    if not response.success:
        raise RuntimeError("The planning scene rejected the attachment.")

    scene = read_scene(node, reader)

    attached_ok = any(
        item.object.id == "et06_piece" and item.link_name == "tool0"
        for item in scene.robot_state.attached_collision_objects
    )
    still_in_world = any(
        item.id == "et06_piece"
        for item in scene.world.collision_objects
    )

    if not attached_ok or still_in_world:
        raise RuntimeError("Unexpected planning-scene state after attachment.")

    print("Attachment confirmed: et06_piece -> tool0.")
    print("The workpiece is no longer an independent world object.")
    return scene


def main():
    folder = Path(__file__).resolve().parents[1]
    data = yaml.safe_load(
        (folder / "config/key_poses.yaml").read_text()
    )

    filename = folder / "results/retimed/quintic_red_joints.csv"
    with filename.open() as file:
        rows = list(csv.DictReader(file))

    if not rows:
        raise RuntimeError("CSV de l'approche vide.")

    target = [float(rows[-1][f"q{j}"]) for j in range(1, 7)]

    rclpy.init()
    node = Node("et06_attach_piece")

    try:
        # Check the robot position before modifying the planning scene.
        check_position(node, data["joint_names"], target, 0.001)

        scene = attach_piece(node)

        client = node.create_client(
            GetStateValidity, "/check_state_validity"
        )

        request = GetStateValidity.Request()
        request.group_name = data["planning_group"]
        request.robot_state = scene.robot_state
        request.robot_state.is_diff = False

        response = call_service(node, client, request)

        print(
            "State with attached workpiece: "
            + ("VALID" if response.valid else "INVALID")
        )

        for contact in response.contacts:
            print(
                f"Contact : {contact.contact_body_1}"
                f" / {contact.contact_body_2}"
            )

        print("No motion was commanded.")
        print("The workpiece remains attached at the end of the program.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
