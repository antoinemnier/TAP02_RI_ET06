#!/usr/bin/env python3
import os
from pathlib import Path
import csv
import math
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.action import ExecuteTrajectory

from attach_workpiece import read_scene
from execute_pick_approach import check_position, execute


def main():
    folder = Path(__file__).resolve().parents[1]

    data = yaml.safe_load(
        (folder / "config/key_poses.yaml").read_text()
    )

    names = data["joint_names"]

    filename = (
        folder
        / "results"
        / "retimed"
        / "quintic_blue_place_joints.csv"
    )

    if not filename.is_file():
        raise RuntimeError(f"Fichier absent : {filename}")

    with filename.open() as file:
        rows = list(csv.DictReader(file))

    if len(rows) < 2:
        raise RuntimeError("The placement profile is empty or incomplete.")

    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = names

    previous_ns = -1

    for row in rows:
        point = JointTrajectoryPoint()

        point.positions = [
            float(row[f"q{joint}"])
            for joint in range(1, 7)
        ]

        point.velocities = [
            float(row[f"dq{joint}"])
            for joint in range(1, 7)
        ]

        point.accelerations = [
            float(row[f"ddq{joint}"])
            for joint in range(1, 7)
        ]

        time_s = float(row["time_s"])

        values = (
            list(point.positions)
            + list(point.velocities)
            + list(point.accelerations)
            + [time_s]
        )

        if not all(math.isfinite(value) for value in values):
            raise RuntimeError("Valeur non finie dans le profil.")

        time_ns = round(time_s * 1_000_000_000)

        if time_ns < 0 or time_ns <= previous_ns:
            raise RuntimeError("Temps non strictement croissants.")

        point.time_from_start.sec = (
            time_ns // 1_000_000_000
        )
        point.time_from_start.nanosec = (
            time_ns % 1_000_000_000
        )

        previous_ns = time_ns
        trajectory.joint_trajectory.points.append(point)

    start = list(
        trajectory.joint_trajectory.points[0].positions
    )
    endpoint = list(
        trajectory.joint_trajectory.points[-1].positions
    )

    duration = float(rows[-1]["time_s"])

    rclpy.init()
    node = Node("et06_execute_place")

    try:
        reader = node.create_client(
            GetPlanningScene,
            "/get_planning_scene",
        )

        executor = ActionClient(
            node,
            ExecuteTrajectory,
            "/execute_trajectory",
        )

        check_position(
            node,
            names,
            data["pre_place"]["joints"],
            0.001,
        )

        scene = read_scene(node, reader)

        attached = any(
            obj.object.id == "et06_piece"
            and obj.link_name == "tool0"
            for obj in scene.robot_state.attached_collision_objects
        )

        if not attached:
            raise RuntimeError(
                "The workpiece is not attached to tool0."
            )

        check_position(
            node,
            names,
            start,
            0.001,
        )

        print(
            f"Blue quintic profile : {len(rows)} points, "
            f"duration {duration:.6f} s."
        )
        print(
            "SIMULATION ONLY. "
            "The workpiece must remain attached during the descent."
        )

        if os.environ.get("ET06_AUTO_CONFIRM") != "1":
            answer = input(
                "Type YES to execute the placement approach : "
            )
            if answer.strip().upper() != "YES":
                raise RuntimeError("Execution cancelled by the user.")
        else:
            print("Execution authorized by the main program.")

        check_position(
            node,
            names,
            start,
            0.001,
        )

        execute(
            node,
            executor,
            trajectory,
        )

        check_position(
            node,
            names,
            endpoint,
            0.001,
        )

        final_scene = read_scene(node, reader)

        still_attached = any(
            obj.object.id == "et06_piece"
            and obj.link_name == "tool0"
            for obj in final_scene.robot_state.attached_collision_objects
        )

        if not still_attached:
            raise RuntimeError(
                "The workpiece is no longer attached after the descent."
            )

        print("Placement approach completed.")
        print("The workpiece remains attached.")
        print("Detachment has not been performed yet.")

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nProgram interrupted.")
        raise SystemExit(130)
    except Exception as error:
        print(f"\nSTOP : {error}")
        raise SystemExit(1)
