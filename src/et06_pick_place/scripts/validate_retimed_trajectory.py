#!/usr/bin/env python3
from pathlib import Path
import csv
import yaml

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetStateValidity

from setup_scene import call_service


def main():
    folder = Path(__file__).resolve().parents[1]
    data = yaml.safe_load(
        (folder / "config/key_poses.yaml").read_text()
    )

    rclpy.init()
    node = Node("et06_check_retimed")

    try:
        client = node.create_client(
            GetStateValidity, "/check_state_validity"
        )

        for profile in ("cubic", "quintic"):
            filename = (
                folder / "results/retimed"
                / f"{profile}_red_joints.csv"
            )

            with filename.open() as file:
                rows = list(csv.DictReader(file))

            if not rows:
                raise RuntimeError(f"Fichier vide : {filename}")

            print(f"\n=== {profile} : {len(rows)} points ===", flush=True)
            all_valid = True

            for index, row in enumerate(rows):
                request = GetStateValidity.Request()
                request.group_name = data["planning_group"]
                request.robot_state.is_diff = True
                request.robot_state.joint_state.name = data["joint_names"]
                request.robot_state.joint_state.position = [
                    float(row[f"q{j}"]) for j in range(1, 7)
                ]

                response = call_service(node, client, request)

                if not response.valid:
                    all_valid = False
                    print(
                        f"INVALIDE : point {index}, "
                        f"temps = {float(row['time_s']):.6f} s",
                        flush=True,
                    )

                    for contact in response.contacts:
                        print(
                            f"  Contact : {contact.contact_body_1}"
                            f" / {contact.contact_body_2}",
                            flush=True,
                        )
                    break

                if (index + 1) % 500 == 0:
                    print(
                        f"  {index + 1}/{len(rows)} points verifies",
                        flush=True,
                    )

            if all_valid:
                print(
                    f"VALIDE : {len(rows)}/{len(rows)} points.",
                    flush=True,
                )

        print("\nAucun mouvement commande.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

