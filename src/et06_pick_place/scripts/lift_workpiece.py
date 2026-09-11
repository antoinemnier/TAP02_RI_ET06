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

from attach_workpiece import read_scene
from setup_scene import call_service
from execute_pick_approach import check_position, execute


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

        # Read both joint positions and attached objects.
        scene = read_scene(node, reader)

        attached = any(
            obj.object.id == "et06_piece" and obj.link_name == "tool0"
            for obj in scene.robot_state.attached_collision_objects
        )

        if not attached:
            raise RuntimeError("The workpiece is not attached to tool0.")

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

        # Complete state explicitly including the attached workpiece.
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

        print(f"Result code : {response.error_code.val}")
        print(f"Computed fraction : {response.fraction:.8f}")
        print(f"Number of points : {len(points)}")

        if (
            response.error_code.val != 1
            or response.fraction < 1.0 - 1e-9
            or len(points) < 2
        ):
            raise RuntimeError("Incomplete lift: no execution was requested.")

        duration = (
            points[-1].time_from_start.sec
            + points[-1].time_from_start.nanosec * 1e-9
        )
        print(f"MoveIt duration : {duration:.6f} s")
        print("SIMULATION ONLY: upward motion with the attached workpiece.")

        if os.environ.get("ET06_AUTO_CONFIRM") != "1":
            answer = input(
                "Type YES to execute the lift : "
            )
            if answer.strip().upper() != "YES":
                raise RuntimeError("Execution cancelled by the user.")
        else:
            print("Execution authorized by the main program.")

        # Do not execute if the robot moved after planning.
        check_position(node, names, start, 0.001)

        execute(node, executor, trajectory)

        # Check la configuration finale dans le bon ordre.
        final_positions = dict(zip(
            trajectory.joint_trajectory.joint_names,
            points[-1].positions,
        ))
        endpoint = [final_positions[name] for name in names]
        check_position(node, names, endpoint, 0.001)

        print("Lift completed.")
        print("The workpiece remains attached. No transfer to PLACE was performed.")

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
