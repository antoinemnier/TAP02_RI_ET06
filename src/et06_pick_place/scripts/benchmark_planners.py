#!/usr/bin/env python3
from pathlib import Path
from datetime import datetime
import json
import time
import yaml

import rclpy
from rclpy.node import Node

from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan

from setup_scene import call_service


def main():
    folder = Path(__file__).resolve().parents[1]
    config_file = folder / "config" / "key_poses.yaml"
    data = yaml.safe_load(config_file.read_text())

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output_dir = folder / "results" / run_id
    output_dir.mkdir(parents=True, exist_ok=False)
    print(f"Results : {output_dir}", flush=True)

    trials = []
    for trial in range(1, 11):
        order = ["RRTConnect", "RRTstar"]
        if trial % 2 == 0:
            order.reverse()

        for planner in order:
            trials.append((trial, planner))

    successes = {"RRTConnect": 0, "RRTstar": 0}

    rclpy.init()
    node = Node("et06_test_plan")

    try:
        client = node.create_client(
            GetMotionPlan, "/plan_kinematic_path"
        )

        for trial, planner in trials:
            request = GetMotionPlan.Request()
            req = request.motion_plan_request

            req.group_name = data["planning_group"]
            req.pipeline_id = "ompl"
            req.planner_id = planner
            req.num_planning_attempts = 1
            req.allowed_planning_time = 5.0
            req.max_velocity_scaling_factor = 0.1
            req.max_acceleration_scaling_factor = 0.1

            # Explicit start state : theoretical HOME.
            # No motion is commanded.
            req.start_state.is_diff = True
            req.start_state.joint_state.name = data["joint_names"]
            req.start_state.joint_state.position = [
                float(v) for v in data["home"]["joints"]
            ]

            goal = Constraints()
            goal.name = "pre_pick_joint_goal"

            for name, position in zip(
                data["joint_names"],
                data["pre_pick"]["joints"],
            ):
                joint = JointConstraint()
                joint.joint_name = name
                joint.position = float(position)
                joint.tolerance_above = 1e-5
                joint.tolerance_below = 1e-5
                joint.weight = 1.0
                goal.joint_constraints.append(joint)

            req.goal_constraints = [goal]

            print(
                f"\n=== Trial {trial}/10 : {planner} ===",
                flush=True,
            )

            start = time.perf_counter()
            response = call_service(node, client, request)
            elapsed = time.perf_counter() - start

            result = response.motion_plan_response
            trajectory = result.trajectory.joint_trajectory
            points = trajectory.points

            record = {
                "trial": trial,
                "planner": planner,
                "pipeline": req.pipeline_id,
                "allowed_planning_time_s": req.allowed_planning_time,
                "velocity_scaling": req.max_velocity_scaling_factor,
                "acceleration_scaling": (
                    req.max_acceleration_scaling_factor
                ),
                "start_joints": list(
                    req.start_state.joint_state.position
                ),
                "goal_joints": list(data["pre_pick"]["joints"]),
                "error_code": result.error_code.val,
                "error_message": result.error_code.message,
                "error_source": result.error_code.source,
                "planning_time_s": result.planning_time,
                "service_wall_time_s": elapsed,
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

            output_file = output_dir / f"{planner}_{trial:02d}.json"
            output_file.write_text(
                json.dumps(record, indent=2, allow_nan=False)
            )

            print(f"Result code : {result.error_code.val}")
            print(f"Message : {result.error_code.message}")
            print(f"Source : {result.error_code.source}")
            print(
                f"Time reported by MoveIt : "
                f"{result.planning_time:.6f} s"
            )
            print(f"Total service-call time : {elapsed:.6f} s")
            print(f"Number of returned points : {len(points)}")
            print(f"File : {output_file.name}")

            if result.error_code.val == 1 and points:
                successes[planner] += 1
                print("Trajectory returned successfully.", flush=True)
            else:
                print("No usable trajectory was returned.", flush=True)

        print("\n=== SUMMARY ===")
        for planner, count in successes.items():
            print(f"{planner} : {count}/10 succes")

        print(f"Results saved in : {output_dir}")
        print("No motion execution was requested.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
