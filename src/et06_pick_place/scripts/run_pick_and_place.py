#!/usr/bin/env python3
import os
from pathlib import Path
import subprocess
import sys


SCRIPTS_FOLDER = Path(__file__).resolve().parent

STEPS = [
    (
        "Creation et verification de la scene",
        "setup_scene.py",
        False,
    ),
    (
        "HOME -> PRE_PICK -> PICK",
        "execute_pick_approach.py",
        True,
    ),
    (
        "Attachement de la piece",
        "attach_workpiece.py",
        False,
    ),
    (
        "Decollage de la piece",
        "lift_workpiece.py",
        True,
    ),
    (
        "Transport vers PRE_PLACE",
        "transfer_workpiece.py",
        True,
    ),
    (
        "Approche de depot",
        "execute_place_approach.py",
        True,
    ),
    (
        "Detachement de la piece",
        "detach_workpiece.py",
        False,
    ),
]


def run_step(number, title, filename, needs_confirmation):
    script = SCRIPTS_FOLDER / filename

    if not script.is_file():
        raise RuntimeError(f"Script introuvable : {script}")

    print()
    print("=" * 70)
    print(f"ETAPE {number}/{len(STEPS)} : {title}")
    print(f"Script : {filename}")
    print("=" * 70)

    environment = os.environ.copy()
    environment["ET06_AUTO_CONFIRM"] = "1"

    result = subprocess.run(
        [sys.executable, str(script)],
        cwd=str(SCRIPTS_FOLDER),
        env=environment,
    )

    if result.returncode != 0:
        raise RuntimeError(
            f"Echec de l'etape {number} : {title}"
        )

    print(f"Etape {number} terminee.")


def main():
    print("CYCLE PICK-AND-PLACE ET06")
    print("Robot : FANUC M-10iA")
    print("SIMULATION UNIQUEMENT")
    print()
    print("Conditions requises :")
    print("- demo.launch.py actif ;")
    print("- robot initialement en HOME ;")
    print("- aucun autre mouvement en cours ;")
    print("- aucun autre script de scene en cours.")
    print()

    answer = input(
        "Taper DEMARRER pour lancer la sequence : "
    )

    if answer.strip() != "DEMARRER":
        print("Sequence annulee.")
        return

    for number, step in enumerate(STEPS, start=1):
        run_step(number, *step)

    print()
    print("=" * 70)
    print("CYCLE TERMINE")
    print("La piece a ete deposee et detachee.")
    print("=" * 70)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nSequence interrompue par l'utilisateur.")
        sys.exit(1)
    except Exception as error:
        print(f"\nARRET DU CYCLE : {error}")
        sys.exit(1)
