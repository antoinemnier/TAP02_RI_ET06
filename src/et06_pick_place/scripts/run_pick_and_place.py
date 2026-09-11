#!/usr/bin/env python3
import os
from pathlib import Path
import subprocess
import sys


SCRIPTS_FOLDER = Path(__file__).resolve().parent

STEPS = [
    (
        "Create and verify the planning scene",
        "setup_scene.py",
        False,
    ),
    (
        "HOME -> PRE_PICK -> PICK",
        "execute_pick_approach.py",
        True,
    ),
    (
        "Attach the workpiece",
        "attach_workpiece.py",
        False,
    ),
    (
        "Lift the workpiece",
        "lift_workpiece.py",
        True,
    ),
    (
        "Transfer to PRE_PLACE",
        "transfer_workpiece.py",
        True,
    ),
    (
        "Placement approach",
        "execute_place_approach.py",
        True,
    ),
    (
        "Detach the workpiece",
        "detach_workpiece.py",
        False,
    ),
]


def run_step(number, title, filename, needs_confirmation):
    script = SCRIPTS_FOLDER / filename

    if not script.is_file():
        raise RuntimeError(f"Script not found : {script}")

    print()
    print("=" * 70)
    print(f"STEP {number}/{len(STEPS)} : {title}")
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

    print(f"Step {number} terminee.")


def main():
    print("ET06 PICK-AND-PLACE CYCLE")
    print("Robot : FANUC M-10iA")
    print("SIMULATION ONLY")
    print()
    print("Requirements :")
    print("- demo.launch.py running ;")
    print("- robot initially at HOME ;")
    print("- no other motion currently running ;")
    print("- no other planning-scene script currently running.")
    print()

    answer = input(
        "Type START to run the sequence : "
    )

    if answer.strip().upper() != "START":
        print("Sequence cancelled.")
        return

    for number, step in enumerate(STEPS, start=1):
        run_step(number, *step)

    print()
    print("=" * 70)
    print("CYCLE COMPLETED")
    print("The workpiece was placed and detached.")
    print("=" * 70)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nSequence interrupted by the user.")
        sys.exit(1)
    except Exception as error:
        print(f"\nCYCLE STOPPED : {error}")
        sys.exit(1)
