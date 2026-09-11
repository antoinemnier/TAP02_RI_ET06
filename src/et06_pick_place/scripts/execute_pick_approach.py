#!/usr/bin/env python3
from pathlib import Path
import csv
import math
import yaml
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.wait_for_message import wait_for_message
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory, Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.action import ExecuteTrajectory

from setup_scene import call_service


def current_positions(node, names):
    ok, message = wait_for_message(
        JointState,
        node,
        "/joint_states",
        qos_profile=qos_profile_sensor_data,
        time_to_wait=5.0,
    )

    if not ok:
        raise RuntimeError("Aucun etat articulaire recu.")

    positions = dict(zip(message.name, message.position))

    if not all(name in positions for name in names):
        raise RuntimeError("Etat articulaire incomplet.")

    q = [float(positions[name]) for name in names]

    if not all(math.isfinite(value) for value in q):
        raise RuntimeError("Position articulaire non finie.")

    return q


def check_position(node, names, target, tolerance):
    actual = current_positions(node, names)
    error = max(abs(a - b) for a, b in zip(actual, target))
    print(f"Ecart articulaire maximal : {error:.8f} rad", flush=True)

    if error > tolerance:
        raise RuntimeError("Le robot n'est pas a la position attendue.")


def execute(node, client, trajectory):
    if not client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError("Action /execute_trajectory indisponible.")

    goal = ExecuteTrajectory.Goal()
    goal.trajectory = trajectory

    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    handle = future.result()

    if handle is None or not handle.accepted:
        raise RuntimeError("Trajectoire refusee par MoveIt.")

    try:
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, future)
        result = future.result()

        if result is None:
            raise RuntimeError("Aucun resultat d'execution.")

        code = result.result.error_code.val
        print(f"Code execution : {code}", flush=True)

        if result.status != 4 or code != 1:
            raise RuntimeError("Execution non terminee avec succes.")

    except BaseException:
        # Demande d'annulation si le programme est interrompu.
        if rclpy.ok():
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=2.0)
        raise


def main():
    folder = Path(__file__).resolve().parents[1]
    data = yaml.safe_load(
        (folder / "config/key_poses.yaml").read_text()
    )
    names = data["joint_names"]

    expected = [f"joint_{j}" for j in range(1, 7)]
    if names != expected:
        raise RuntimeError("Verifier l'ordre des articulations du CSV.")

    filename = folder / "results/retimed/quintic_red_joints.csv"
    with filename.open() as file:
        rows = list(csv.DictReader(file))

    if len(rows) < 2:
        raise RuntimeError("CSV vide ou incomplet.")

    approach = RobotTrajectory()
    approach.joint_trajectory.joint_names = names
    previous_ns = -1

    for row in rows:
        point = JointTrajectoryPoint()
        point.positions = [float(row[f"q{j}"]) for j in range(1, 7)]
        point.velocities = [float(row[f"dq{j}"]) for j in range(1, 7)]
        point.accelerations = [float(row[f"ddq{j}"]) for j in range(1, 7)]
        t = float(row["time_s"])

        values = point.positions + point.velocities + point.accelerations
        if not all(math.isfinite(v) for v in values) or not math.isfinite(t):
            raise RuntimeError("Valeur non finie dans le CSV.")

        ns = round(t * 1_000_000_000)
        if ns < 0 or ns <= previous_ns:
            raise RuntimeError("Temps invalides dans le CSV.")

        point.time_from_start.sec = ns // 1_000_000_000
        point.time_from_start.nanosec = ns % 1_000_000_000
        previous_ns = ns
        approach.joint_trajectory.points.append(point)

    target = list(approach.joint_trajectory.points[0].positions)
    endpoint = list(approach.joint_trajectory.points[-1].positions)

    rclpy.init()
    node = Node("et06_execute_pick")

    try:
        planner = node.create_client(GetMotionPlan, "/plan_kinematic_path")
        executor = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")

        start = current_positions(node, names)
        home_error = max(
            abs(a - b) for a, b in zip(start, data["home"]["joints"])
        )
        print(f"Ecart a HOME : {home_error:.6f} rad")

        if home_error > 0.01:
            print("Attention : le transfert ne commencera pas en HOME.")

        request = GetMotionPlan.Request()
        req = request.motion_plan_request
        req.group_name = data["planning_group"]
        req.pipeline_id = "ompl"
        req.planner_id = "RRTConnect"
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 1
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1

        req.start_state.is_diff = True
        req.start_state.joint_state.name = names
        req.start_state.joint_state.position = start

        goal = Constraints()
        for name, position in zip(names, target):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = position
            joint.tolerance_above = 1e-5
            joint.tolerance_below = 1e-5
            joint.weight = 1.0
            goal.joint_constraints.append(joint)

        req.goal_constraints = [goal]

        print("Planification du transfert...", flush=True)
        response = call_service(node, planner, request)
        plan = response.motion_plan_response

        if plan.error_code.val != 1 or not plan.trajectory.joint_trajectory.points:
            raise RuntimeError(
                f"Planification echouee : code {plan.error_code.val}"
            )

        print("Transfert calcule.")
        print(f"Approche : {len(rows)} points, duree {t:.6f} s.")
        print("SIMULATION UNIQUEMENT. Ne pas modifier la scene.")

        if os.environ.get("ET06_AUTO_CONFIRM") != "1":
            answer = input(
                "Taper OUI pour executer les deux mouvements : "
            )
            if answer.strip().upper() != "OUI":
                raise RuntimeError("Execution annulee par l'utilisateur.")
        else:
            print("Execution autorisee par le programme principal.")

        # Verifier que le robot n'a pas bouge pendant la confirmation.
        check_position(node, names, start, 0.001)

        print("Execution du transfert vers pre_pick...", flush=True)
        execute(node, executor, plan.trajectory)

        # Verifier le raccord avant d'envoyer l'approche.
        check_position(node, names, target, 0.001)

        print("Execution de l'approche quintique...", flush=True)
        execute(node, executor, approach)

        check_position(node, names, endpoint, 0.001)
        print("Transfert et approche termines.")
        print("Piece non attachee : fin du test 4A + 4B.")

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
