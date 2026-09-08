#!/usr/bin/env python3
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node

from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity

# Reutilise les fonctions du script voisin, sans executer son main().
from check_scene import box, call_service


def main():
    folder = Path(__file__).resolve().parents[1]
    data = yaml.safe_load(
        (folder / "config/key_poses.yaml").read_text()
    )

    names = ["home", "pre_pick", "pick", "pre_place", "place"]
    states = {
        name: [float(v) for v in data[name]["joints"]]
        for name in names
    }

    rclpy.init()
    node = Node("et06_find_obstacle")

    apply_client = node.create_client(
        ApplyPlanningScene, "/apply_planning_scene"
    )
    validity_client = node.create_client(
        GetStateValidity, "/check_state_validity"
    )

    # Derniere geometrie connue, restauree si aucune candidate ne convient.
    original_center = [0.55, -0.22, 0.975]
    original_size = [0.08, 0.08, 0.35]
    selected = False

    def apply_object(obj):
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        req.scene.robot_state.is_diff = True
        req.scene.world.collision_objects = [obj]
        result = call_service(node, apply_client, req)
        if not result.success:
            raise RuntimeError("Modification de scene refusee.")

    def place_obstacle(center, size):
        apply_object(box(
            data["frame_id"], "et06_obstacle", center, size
        ))

    def check(q):
        req = GetStateValidity.Request()
        req.group_name = data["planning_group"]
        req.robot_state.is_diff = True
        req.robot_state.joint_state.name = data["joint_names"]
        req.robot_state.joint_state.position = q
        return call_service(node, validity_client, req)

    try:
        # Reference sans poste : les autres objets restent dans la scene.
        obj = CollisionObject()
        obj.id = "et06_obstacle"
        obj.operation = CollisionObject.REMOVE
        apply_object(obj)

        for name in names:
            if not check(states[name]).valid:
                raise RuntimeError(
                    f"{name} est invalide meme sans le poste."
                )

        # Segment articulaire de reference, pas une ligne cartesienne.
        samples = []
        for k in range(1, 20):
            s = k / 20.0
            q = [
                (1.0 - s) * a + s * b
                for a, b in zip(states["home"], states["pre_pick"])
            ]
            if not check(q).valid:
                raise RuntimeError(
                    "Le segment de reference rencontre deja un autre "
                    f"obstacle a s={s:.2f}. Diagnostic a approfondir."
                )
            samples.append((s, q))

        print("Reference sans poste valide aux points testes.", flush=True)

        # Recherche bornee : plusieurs positions et deux largeurs.
        count = 0
        for width in [0.04, 0.08]:
            for x in [0.45, 0.55, 0.65, 0.75, 0.85]:
                for y in [-0.06, -0.10, -0.14, -0.18, -0.22]:
                    for top in [1.05, 1.15, 1.25]:
                        bottom = 0.80
                        center = [x, y, (bottom + top) / 2.0]
                        size = [width, width, top - bottom]

                        count += 1
                        place_obstacle(center, size)

                        if count % 20 == 0:
                            print(
                                f"{count} candidates testees...",
                                flush=True,
                            )

                        # Toutes les extremites doivent rester libres.
                        if not all(check(states[n]).valid for n in names):
                            continue

                        for s, q in samples:
                            result = check(q)
                            touches_post = any(
                                c.contact_body_1 == "et06_obstacle"
                                or c.contact_body_2 == "et06_obstacle"
                                for c in result.contacts
                            )

                            if not result.valid and touches_post:
                                selected = True
                                print("\nCANDIDATE TROUVEE", flush=True)
                                print(f"Centre : {center}", flush=True)
                                print(f"Dimensions : {size}", flush=True)
                                print(
                                    f"Reference bloquee a s={s:.2f}",
                                    flush=True,
                                )
                                print(
                                    "Les cinq configurations sont valides.",
                                    flush=True,
                                )
                                print(
                                    "Le poste reste a cette position dans RViz.",
                                    flush=True,
                                )
                                return

        print(
            "\nAucune candidate trouvee dans cette grille.",
            flush=True,
        )

    finally:
        try:
            if not selected:
                place_obstacle(original_center, original_size)
                print("Position initiale du poste restauree.", flush=True)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()

