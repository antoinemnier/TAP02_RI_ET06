#!/usr/bin/env python3

from pathlib import Path
import py_compile
import shutil
import subprocess
import sys


WORKSPACE = Path(__file__).resolve().parents[1]

SCRIPTS = (
    WORKSPACE
    / "src"
    / "et06_pick_place"
    / "scripts"
)

MATLAB = WORKSPACE / "matlab"

BACKUP = WORKSPACE / ".english_migration_backup"


# ------------------------------------------------------------
# Script file renaming
# ------------------------------------------------------------

RENAMES = {
    "check_scene.py": "setup_scene.py",
    "test_plan.py": "benchmark_planners.py",
    "approach_profiles.py": "generate_approach_profiles.py",
    "cartesian_approach.py": "compute_pick_approach.py",
    "cartesian_place.py": "compute_place_approach.py",
    "check_retimed.py": "validate_retimed_trajectory.py",
    "execute_pick.py": "execute_pick_approach.py",
    "attach_piece.py": "attach_workpiece.py",
    "lift_piece.py": "lift_workpiece.py",
    "transfer_piece.py": "transfer_workpiece.py",
    "execute_place.py": "execute_place_approach.py",
    "detach_piece.py": "detach_workpiece.py",
    "run_pick_place.py": "run_pick_and_place.py",
}


# ------------------------------------------------------------
# References to renamed files and modules
# ------------------------------------------------------------

REFERENCE_REPLACEMENTS = {
    "from check_scene import": "from setup_scene import",
    "from execute_pick import": "from execute_pick_approach import",
    "from attach_piece import": "from attach_workpiece import",

    "check_scene.py": "setup_scene.py",
    "test_plan.py": "benchmark_planners.py",
    "approach_profiles.py": "generate_approach_profiles.py",
    "cartesian_approach.py": "compute_pick_approach.py",
    "cartesian_place.py": "compute_place_approach.py",
    "check_retimed.py": "validate_retimed_trajectory.py",
    "execute_pick.py": "execute_pick_approach.py",
    "attach_piece.py": "attach_workpiece.py",
    "lift_piece.py": "lift_workpiece.py",
    "transfer_piece.py": "transfer_workpiece.py",
    "execute_place.py": "execute_place_approach.py",
    "detach_piece.py": "detach_workpiece.py",
    "run_pick_place.py": "run_pick_and_place.py",
}


# ------------------------------------------------------------
# Conservative translation of user-visible text and comments
# ------------------------------------------------------------

TEXT_REPLACEMENTS = {
    # Main program
    "CYCLE PICK-AND-PLACE ET06":
        "ET06 PICK-AND-PLACE CYCLE",
    "Conditions requises":
        "Requirements",
    "demo.launch.py actif":
        "demo.launch.py running",
    "robot initialement en HOME":
        "robot initially at HOME",
    "aucun autre mouvement en cours":
        "no other motion currently running",
    "aucun autre script de scene en cours":
        "no other planning-scene script currently running",
    "Taper DEMARRER pour lancer la sequence":
        "Type START to run the sequence",
    'answer.strip() != "DEMARRER"':
        'answer.strip().upper() != "START"',
    "Sequence annulee.":
        "Sequence cancelled.",
    "ETAPE":
        "STEP",
    "Etape":
        "Step",
    "Creation et verification de la scene":
        "Create and verify the planning scene",
    "Attachement de la piece":
        "Attach the workpiece",
    "Decollage de la piece":
        "Lift the workpiece",
    "Transport vers PRE_PLACE":
        "Transfer to PRE_PLACE",
    "Approche de depot":
        "Placement approach",
    "Detachement de la piece":
        "Detach the workpiece",
    "CYCLE TERMINE":
        "CYCLE COMPLETED",
    "La piece a ete deposee et detachee.":
        "The workpiece was placed and detached.",
    "ARRET DU CYCLE":
        "CYCLE STOPPED",

    # Confirmation
    "Execution autorisee par le programme principal.":
        "Execution authorized by the main program.",
    "Taper OUI pour executer les deux mouvements":
        "Type YES to execute both motions",
    "Taper OUI pour executer le decollage":
        "Type YES to execute the lift",
    "Taper OUI pour executer le transfert":
        "Type YES to execute the transfer",
    "Taper OUI pour executer le depot":
        "Type YES to execute the placement approach",
    'answer.strip().upper() != "OUI"':
        'answer.strip().upper() != "YES"',
    'answer.strip() != "OUI"':
        'answer.strip().upper() != "YES"',
    "Execution annulee par l'utilisateur.":
        "Execution cancelled by the user.",

    # General
    "SIMULATION UNIQUEMENT":
        "SIMULATION ONLY",
    "Aucun mouvement n'a ete commande.":
        "No motion was commanded.",
    "Aucun mouvement commande.":
        "No motion was commanded.",
    "Aucune execution demandee.":
        "No execution was requested.",
    "Aucune execution de mouvement.":
        "No motion was executed.",
    "Aucun mouvement commande":
        "No motion was commanded",
    "Calcul hors ligne uniquement.":
        "Offline computation only.",
    "Programme interrompu.":
        "Program interrupted.",
    "Sequence interrompue par l'utilisateur.":
        "Sequence interrupted by the user.",
    "Script introuvable":
        "Script not found",
    "ARRET":
        "STOP",

    # Planning scene
    "Scene appliquee":
        "Planning scene applied",
    "quatre objets ET06":
        "four ET06 objects",
    "Les cinq etats sont valides dans cette scene.":
        "All five states are valid in this planning scene.",
    "Les trajets entre ces etats restent a verifier.":
        "Paths between these states still need verification.",
    "Scene a ajuster avant de planifier le cycle.":
        "The scene must be adjusted before planning the cycle.",
    "La piece reste un objet du monde, non attache.":
        "The workpiece remains an unattached world object.",
    "VALIDE":
        "VALID",
    "INVALIDE":
        "INVALID",

    # Planning
    "Planification du transfert vers pre_pick":
        "Planning the transfer to PRE_PICK",
    "Planification du transfert":
        "Planning the transfer",
    "Planification vers PRE_PLACE avec la piece":
        "Planning to PRE_PLACE with the attached workpiece",
    "Planification echouee":
        "Planning failed",
    "Transfert calcule.":
        "Transfer plan computed.",
    "Code planification":
        "Planning result code",
    "Code resultat":
        "Result code",
    "Code execution":
        "Execution result code",
    "Temps rapporte par MoveIt":
        "Time reported by MoveIt",
    "Temps rapporte":
        "Reported planning time",
    "Temps total de l'appel":
        "Total service-call time",
    "Nombre de points retournes":
        "Number of returned points",
    "Nombre de points":
        "Number of points",
    "Duree du transfert":
        "Transfer duration",
    "Duree MoveIt":
        "MoveIt duration",
    "Trajectoire retournee avec succes.":
        "Trajectory returned successfully.",
    "Pas de trajectoire exploitable.":
        "No usable trajectory was returned.",
    "Fraction calculee":
        "Computed fraction",
    "Chemin complet retourne, avec collisions activees.":
        "Complete path returned with collision checking enabled.",
    "Approche incomplete ou en echec.":
        "The approach is incomplete or failed.",
    "Approche incomplete":
        "Incomplete approach",
    "Timing provisoire":
        "Temporary timing",
    "profil bleu pas encore applique":
        "blue profile not yet applied",
    "Profil cubique/quintique pas encore applique.":
        "Cubic/quintic profile not yet applied.",

    # State and execution
    "Ecart a HOME":
        "Error relative to HOME",
    "Ecart articulaire maximal":
        "Maximum joint error",
    "Le robot n'est pas a la position attendue.":
        "The robot is not at the expected position.",
    "Attention : le transfert ne commencera pas en HOME.":
        "Warning: the transfer will not start at HOME.",
    "Execution du transfert vers pre_pick":
        "Executing transfer to PRE_PICK",
    "Execution de l'approche quintique":
        "Executing the quintic approach",
    "Transfert et approche termines.":
        "Transfer and approach completed.",
    "Piece non attachee : fin du test 4A + 4B.":
        "Workpiece not attached: end of the 4A + 4B test.",
    "Decollage termine.":
        "Lift completed.",
    "Transfert termine : robot a PRE_PLACE.":
        "Transfer completed: robot at PRE_PLACE.",
    "Piece toujours attachee. Depot non effectue.":
        "The workpiece remains attached. Placement not performed.",
    "Approche de depot terminee.":
        "Placement approach completed.",
    "Detachement pas encore effectue.":
        "Detachment has not been performed yet.",

    # Attachment and detachment
    "La piece est deja attachee a tool0.":
        "The workpiece is already attached to tool0.",
    "La piece est attachee a un autre lien.":
        "The workpiece is attached to another link.",
    "La piece est absente du monde.":
        "The workpiece is missing from the world.",
    "La piece n'est pas attachee":
        "The workpiece is not attached",
    "Attachement confirme":
        "Attachment confirmed",
    "La piece n'est plus un objet independant du monde.":
        "The workpiece is no longer an independent world object.",
    "Etat avec piece attachee":
        "State with attached workpiece",
    "La piece reste attachee a la fin du programme.":
        "The workpiece remains attached at the end of the program.",
    "La piece reste attachee, meme si l'etat est invalide.":
        "The workpiece remains attached at the end of the program.",
    "La piece reste attachee.":
        "The workpiece remains attached.",
    "La piece reste attachee. Aucun transfert vers PLACE effectue.":
        "The workpiece remains attached. No transfer to PLACE was performed.",
    "Detachement confirme.":
        "Detachment confirmed.",
    "La piece est maintenant un objet du monde.":
        "The workpiece is now a world object.",
    "Repere de la piece":
        "Workpiece frame",
    "Position de la piece":
        "Workpiece position",
    "La piece est encore attachee.":
        "The workpiece is still attached.",
    "La piece n'a pas ete remise correctement dans le monde.":
        "The workpiece was not correctly restored to the world.",

    # Profiles
    "Mouvement":
        "Motion",
    "Limites cartésiennes":
        "Cartesian limits",
    "Chemin lu":
        "Input path",
    "Limites articulaires effectives de vitesse":
        "Effective joint velocity limits",
    "Limites articulaires effectives d'acceleration":
        "Effective joint acceleration limits",
    "Duree minimale cubique":
        "Minimum cubic duration",
    "Duree minimale quintique":
        "Minimum quintic duration",
    "Duree commune initiale":
        "Initial common duration",
    "Duree commune finale":
        "Final common duration",
    "Duree commune ajustee":
        "Adjusted common duration",
    "a la duree commune initiale":
        "at the initial common duration",
    "a la duree initiale":
        "at the initial duration",
    "apres ajustement":
        "after adjustment",
    "Rapport maximal vitesse / limite":
        "Maximum velocity-to-limit ratio",
    "Rapport maximal acceleration / limite":
        "Maximum acceleration-to-limit ratio",
    "Rapport final vitesse / limite":
        "Final velocity-to-limit ratio",
    "Rapport final acceleration / limite":
        "Final acceleration-to-limit ratio",
    "Vitesses maximales par joint":
        "Maximum velocities per joint",
    "Accelerations maximales par joint":
        "Maximum accelerations per joint",
    "Profil quintique bleu":
        "Blue quintic profile",
    "La piece doit rester attachee pendant la descente.":
        "The workpiece must remain attached during the descent.",

    # Benchmark
    "Resultats":
        "Results",
    "Essai":
        "Trial",
    "BILAN":
        "SUMMARY",
    "Succes":
        "Success",
    "Codes retour":
        "Return codes",
    "Tous les essais":
        "All trials",
    "Essais reussis uniquement":
        "Successful trials only",
    "Temps total appel":
        "Total call time",
    "Longueur articulaire":
        "Joint-space path length",
    "Duree trajectoire":
        "Trajectory duration",
    "Norme acceleration articulaire RMS":
        "Joint acceleration RMS norm",
    "Trajectoires avec accelerations exploitables":
        "Trajectories with usable acceleration data",
    "Detail des echecs":
        "Failure details",
    "Aucun.":
        "None.",

    # Common comments
    "Verification":
        "Verification",
    "Verifier":
        "Check",
    "Lecture":
        "Reading",
    "Depart explicite":
        "Explicit start state",
    "Depart impose":
        "Specified start state",
    "Meme duree":
        "Same duration",
    "Durees":
        "Durations",
    "Deplacement":
        "Motion",

    # MATLAB output
    "HOME theorique":
        "theoretical HOME",
    "angles mesures":
        "measured joint values",
    "TF arrondi recopie du terminal":
        "rounded TF copied from the terminal",
    "Difference de position":
        "Position difference",
    "Erreur de position":
        "Position error",
    "Ecart de rotation":
        "Rotation difference",
    "Verification interne":
        "Internal verification",
    "Validation numerique":
        "Numerical validation",
    "cinematique directe":
        "forward kinematics",
    "transformation MoveIt":
        "MoveIt transformation",
    "transformation DH":
        "DH transformation",
    "Chemin cartesien MoveIt":
        "MoveIt Cartesian path",
    "Erreur position initiale":
        "Initial position error",
    "Erreur position finale":
        "Final position error",
    "Ecart transversal maximal":
        "Maximum transverse error",
    "Ecart orientation maximal":
        "Maximum orientation difference",
    "verification cartesienne":
        "Cartesian verification",
    "Vitesse cartesienne maximale estimee":
        "Estimated maximum Cartesian velocity",
    "Acceleration maximale estimee hors bords":
        "Estimated maximum acceleration excluding boundaries",
}


SUPPORTED_SUFFIXES = {
    ".py",
    ".m",
    ".md",
    ".yaml",
    ".yml",
    ".txt",
}


EXCLUDED_PARTS = {
    ".git",
    ".english_migration_backup",
    "build",
    "install",
    "log",
    "__pycache__",
    "results",
}


def run(command):
    subprocess.run(
        command,
        cwd=WORKSPACE,
        check=True,
    )


def ensure_git_is_clean():
    result = subprocess.run(
        ["git", "status", "--porcelain"],
        cwd=WORKSPACE,
        check=True,
        capture_output=True,
        text=True,
    )

    if result.stdout.strip():
        raise SystemExit(
            "Git working tree is not clean.\n"
            "Commit the working cycle before migration."
        )


def backup_workspace_files():
    if BACKUP.exists():
        raise SystemExit(
            f"Backup already exists: {BACKUP}\n"
            "Remove it only after reviewing the previous migration."
        )

    BACKUP.mkdir(parents=True)

    for root in [
        WORKSPACE / "src" / "et06_pick_place",
        MATLAB,
    ]:
        if root.exists():
            destination = BACKUP / root.relative_to(WORKSPACE)
            shutil.copytree(root, destination)

    for name in ["README.md", "REPRISE.md"]:
        source = WORKSPACE / name

        if source.is_file():
            destination = BACKUP / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, destination)


def rename_scripts():
    for old_name, new_name in RENAMES.items():
        old_path = SCRIPTS / old_name
        new_path = SCRIPTS / new_name

        if new_path.exists() and not old_path.exists():
            print(f"Already renamed: {new_name}")
            continue

        if not old_path.exists():
            print(f"Not found, skipped: {old_name}")
            continue

        if new_path.exists():
            raise RuntimeError(
                f"Cannot rename {old_name}: {new_name} already exists."
            )

        run([
            "git",
            "mv",
            str(old_path.relative_to(WORKSPACE)),
            str(new_path.relative_to(WORKSPACE)),
        ])

        print(f"Renamed: {old_name} -> {new_name}")


def is_excluded(path):
    return any(part in EXCLUDED_PARTS for part in path.parts)


def candidate_files():
    files = []

    roots = [
        WORKSPACE / "src" / "et06_pick_place",
        MATLAB,
    ]

    for root in roots:
        if not root.exists():
            continue

        for path in root.rglob("*"):
            if not path.is_file():
                continue

            if is_excluded(path):
                continue

            if path.suffix.lower() in SUPPORTED_SUFFIXES:
                files.append(path)

    for name in ["README.md", "REPRISE.md"]:
        path = WORKSPACE / name

        if path.is_file():
            files.append(path)

    return sorted(set(files))


def replace_text(path):
    original = path.read_text(encoding="utf-8")
    updated = original

    replacements = {}
    replacements.update(REFERENCE_REPLACEMENTS)
    replacements.update(TEXT_REPLACEMENTS)

    for old, new in sorted(
        replacements.items(),
        key=lambda item: len(item[0]),
        reverse=True,
    ):
        updated = updated.replace(old, new)

    if updated != original:
        path.write_text(updated, encoding="utf-8")
        print(f"Translated: {path.relative_to(WORKSPACE)}")


def compile_python_files():
    failures = []

    for path in SCRIPTS.glob("*.py"):
        try:
            py_compile.compile(
                str(path),
                doraise=True,
            )
        except py_compile.PyCompileError as error:
            failures.append((path, str(error)))

    if failures:
        print("\nPython syntax errors:")

        for path, error in failures:
            print(f"\n{path}")
            print(error)

        raise SystemExit(1)


def find_remaining_french():
    terms = [
        "aucun",
        "piece",
        "scene",
        "mouvement",
        "duree",
        "erreur",
        "chemin",
        "vitesse",
        "acceleration",
        "attache",
        "transfert",
        "depot",
        "reussi",
        "verifier",
        "resultat",
        "calcul",
    ]

    matches = []

    for path in candidate_files():
        for line_number, line in enumerate(
            path.read_text(encoding="utf-8").splitlines(),
            start=1,
        ):
            lower = line.lower()

            if any(term in lower for term in terms):
                matches.append(
                    f"{path.relative_to(WORKSPACE)}:"
                    f"{line_number}: {line.strip()}"
                )

    return matches


def main():
    if "--apply" not in sys.argv:
        print("No modification was made.")
        print()
        print("This migration will:")
        print("- rename Python scripts;")
        print("- update imports and script references;")
        print("- translate known comments and messages;")
        print("- compile all Python scripts;")
        print("- report remaining French text.")
        print()
        print("Run with:")
        print(
            "  python3 tools/migrate_project_to_english.py --apply"
        )
        return

    ensure_git_is_clean()
    backup_workspace_files()
    rename_scripts()

    for path in candidate_files():
        replace_text(path)

    compile_python_files()

    remaining = find_remaining_french()

    print()
    print("Migration completed.")
    print(f"Backup: {BACKUP}")
    print("Python syntax check: successful.")

    if remaining:
        report = WORKSPACE / "english_migration_remaining.txt"
        report.write_text(
            "\n".join(remaining) + "\n",
            encoding="utf-8",
        )

        print()
        print(
            f"Remaining possible French text: {len(remaining)} lines"
        )
        print(f"Report: {report}")
    else:
        print("No matching French terms were found.")

    print()
    print("Next steps:")
    print("1. Review: git diff")
    print("2. Test the full simulated cycle.")
    print("3. Correct remaining text manually.")
    print("4. Commit only after the cycle succeeds.")


if __name__ == "__main__":
    main()
