#!/usr/bin/env python3
from pathlib import Path
import json
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath

from setup_scene import call_service


def main():
    folder = Path(__file__).resolve().parents[1]
    data = yaml.safe_load(
        (folder / "config/key_poses.yaml").read_text()
    )

    rclpy.init()
    node = Node("et06_cartesian_approach")

    try:
        client = node.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )

        request = GetCartesianPath.Request()
        request.header.frame_id = data["frame_id"]
        request.group_name = data["planning_group"]
        request.link_name = data["tool_link"]

        # Specified start state : configuration pre_pick deja calculee.
        request.start_state.is_diff = True
        request.start_state.joint_state.name = data["joint_names"]
        request.start_state.joint_state.position = [
            float(q) for q in data["pre_pick"]["joints"]
        ]

        # Pas cartesien de 1 mm.
        request.max_step = 0.001
        request.avoid_collisions = True

        # Temporary timing : ce ne sont PAS les profils des CSV.
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        start = data["pre_pick"]["position"]
        end = data["pick"]["position"]
        orientation = data["pick"]["quaternion_xyzw"]

        # Trois points interieurs, puis le point final.
        # Le depart est deja defini par start_state.
        for fraction in [0.25, 0.50, 0.75, 1.0]:
            pose = Pose()

            pose.position.x = start[0] + fraction * (end[0] - start[0])
            pose.position.y = start[1] + fraction * (end[1] - start[1])
            pose.position.z = start[2] + fraction * (end[2] - start[2])

            pose.orientation.x = float(orientation[0])
            pose.orientation.y = float(orientation[1])
            pose.orientation.z = float(orientation[2])
            pose.orientation.w = float(orientation[3])

            request.waypoints.append(pose)

        response = call_service(node, client, request)
        trajectory = response.solution.joint_trajectory

        print(f"Result code : {response.error_code.val}")
        print(f"Computed fraction : {response.fraction:.8f}")
        print(f"Number of points : {len(trajectory.points)}")

        complete = (
            response.error_code.val == 1
            and response.fraction >= 1.0 - 1e-9
            and len(trajectory.points) > 1
        )

        if not complete:
            print("The approach is incomplete or failed.")
            print("No execution was requested.")
            return

        # Controle simple des ecarts entre points articulaires.
        max_step_joint = max(
            abs(b - a)
            for p, q in zip(
                trajectory.points[:-1], trajectory.points[1:]
            )
            for a, b in zip(p.positions, q.positions)
        )

        print(
            "Plus grand ecart articulaire entre deux points : "
            f"{max_step_joint:.8f} rad"
        )

        record = {
            "description": "pre_pick -> pick, timing MoveIt provisoire",
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
                for p in trajectory.points
            ],
        }

        output = folder / "results" / "cartesian"
        output.mkdir(parents=True, exist_ok=True)
        filename = output / "pre_pick_to_pick.json"
        filename.write_text(
            json.dumps(record, indent=2, allow_nan=False)
        )

        print(f"Trajectoire enregistree : {filename}")
        print("Complete path returned with collision checking enabled.")
        print("Cubic/quintic profile not yet applied.")
        print("No execution was requested.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
