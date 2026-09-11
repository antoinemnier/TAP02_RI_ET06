#!/usr/bin/env python3

from pathlib import Path
import sys

import numpy as np
import yaml
from scipy.interpolate import CubicSpline


# ------------------------------------------------------------
# 1. Lecture des arguments
# ------------------------------------------------------------

if len(sys.argv) != 3:
    raise SystemExit(
        "Utilisation :\n"
        "  python3 retime_approach.py pick red\n"
        "  python3 retime_approach.py place blue"
    )

motion = sys.argv[1].lower()
case = sys.argv[2].lower()

if motion not in ("pick", "place"):
    raise SystemExit(
        "Mouvement incorrect : choisir 'pick' ou 'place'."
    )

if case not in ("red", "blue"):
    raise SystemExit(
        "Cas incorrect : choisir 'red' ou 'blue'."
    )


# ------------------------------------------------------------
# 2. Fichiers et chemins
# ------------------------------------------------------------

package_folder = Path(__file__).resolve().parents[1]

if motion == "pick":
    path_file = (
        package_folder
        / "results"
        / "cartesian"
        / "pre_pick_to_pick_path.csv"
    )
else:
    path_file = (
        package_folder
        / "results"
        / "cartesian"
        / "pre_place_to_place_path.csv"
    )

limits_file = (
    package_folder.parent
    / "et06_m10ia_moveit_config"
    / "config"
    / "joint_limits.yaml"
)

output_folder = package_folder / "results" / "retimed"
output_folder.mkdir(parents=True, exist_ok=True)

if not path_file.is_file():
    raise SystemExit(f"Chemin articulaire introuvable : {path_file}")

if not limits_file.is_file():
    raise SystemExit(f"Limites articulaires introuvables : {limits_file}")

print(f"Mouvement : {motion}")
print(f"Limites cartésiennes : {case}")
print(f"Chemin lu : {path_file}")


# ------------------------------------------------------------
# 3. Lecture et interpolation du chemin q(s)
# ------------------------------------------------------------

path_data = np.loadtxt(
    path_file,
    delimiter=",",
    skiprows=1,
)

if path_data.ndim != 2 or path_data.shape[1] != 7:
    raise SystemExit(
        "Le CSV doit contenir les colonnes "
        "s,joint_1,...,joint_6."
    )

s_path = path_data[:, 0]
q_path = path_data[:, 1:7]

if not np.all(np.isfinite(path_data)):
    raise SystemExit("Le chemin contient une valeur non finie.")

if not np.all(np.diff(s_path) > 0):
    raise SystemExit(
        "La progression s du chemin doit être strictement croissante."
    )

if abs(s_path[0]) > 1e-9 or abs(s_path[-1] - 1.0) > 1e-9:
    raise SystemExit(
        "La progression du chemin doit commencer à 0 et finir à 1."
    )

# Spline géométrique commune aux deux lois temporelles.
path_spline = CubicSpline(
    s_path,
    q_path,
    axis=0,
    extrapolate=False,
)


# ------------------------------------------------------------
# 4. Limites articulaires utilisées
# ------------------------------------------------------------

limits_config = yaml.safe_load(limits_file.read_text())
joint_limits = limits_config["joint_limits"]

# Même choix que dans les planifications précédentes.
velocity_scale = 0.1
acceleration_scale = 0.1

velocity_limits = velocity_scale * np.array([
    joint_limits[f"joint_{joint}"]["max_velocity"]
    for joint in range(1, 7)
])

acceleration_limits = acceleration_scale * np.array([
    joint_limits[f"joint_{joint}"]["max_acceleration"]
    for joint in range(1, 7)
])

print(
    "Limites articulaires effectives de vitesse [rad/s] :"
)
print(velocity_limits)

print(
    "Limites articulaires effectives d'acceleration [rad/s2] :"
)
print(acceleration_limits)


# ------------------------------------------------------------
# 5. Limites cartésiennes du cahier des charges
# ------------------------------------------------------------

path_length = 0.10

if case == "red":
    cartesian_velocity_limit = 0.200
    cartesian_acceleration_limit = 0.300
else:
    cartesian_velocity_limit = 0.100
    cartesian_acceleration_limit = 0.020

# Durée minimale analytique pour chaque loi temporelle.
minimum_cubic_duration = max(
    1.5 * path_length / cartesian_velocity_limit,
    np.sqrt(
        6.0 * path_length
        / cartesian_acceleration_limit
    ),
)

minimum_quintic_duration = max(
    1.875 * path_length / cartesian_velocity_limit,
    np.sqrt(
        (10.0 / np.sqrt(3.0))
        * path_length
        / cartesian_acceleration_limit
    ),
)

# Même durée pour comparer les deux profils, avec marge de 5 %.
initial_duration = 1.05 * max(
    minimum_cubic_duration,
    minimum_quintic_duration,
)

print(
    f"Duree minimale cubique : "
    f"{minimum_cubic_duration:.6f} s"
)

print(
    f"Duree minimale quintique : "
    f"{minimum_quintic_duration:.6f} s"
)

print(
    f"Duree commune initiale : "
    f"{initial_duration:.6f} s"
)


# ------------------------------------------------------------
# 6. Lois temporelles
# ------------------------------------------------------------

# Grille d'analyse. Ce n'est pas une fréquence de contrôleur.
u = np.linspace(0.0, 1.0, 2001)


def evaluate(profile, duration):
    """Évalue q, dq/dt et ddq/dt² sur la durée indiquée."""

    if profile == "cubic":
        s = 3.0 * u**2 - 2.0 * u**3

        ds_dt = (
            6.0 * u - 6.0 * u**2
        ) / duration

        d2s_dt2 = (
            6.0 - 12.0 * u
        ) / duration**2

    elif profile == "quintic":
        s = (
            10.0 * u**3
            - 15.0 * u**4
            + 6.0 * u**5
        )

        ds_dt = (
            30.0 * u**2
            - 60.0 * u**3
            + 30.0 * u**4
        ) / duration

        d2s_dt2 = (
            60.0 * u
            - 180.0 * u**2
            + 120.0 * u**3
        ) / duration**2

    else:
        raise ValueError(f"Profil inconnu : {profile}")

    # Protection contre de très petites erreurs numériques.
    s = np.clip(s, 0.0, 1.0)

    q = path_spline(s)
    dq_ds = path_spline(s, 1)
    d2q_ds2 = path_spline(s, 2)

    velocity = dq_ds * ds_dt[:, None]

    acceleration = (
        d2q_ds2 * ds_dt[:, None] ** 2
        + dq_ds * d2s_dt2[:, None]
    )

    return q, velocity, acceleration


# ------------------------------------------------------------
# 7. Vérification à la durée initiale
# ------------------------------------------------------------

required_factors = {}

for profile in ("cubic", "quintic"):
    q, velocity, acceleration = evaluate(
        profile,
        initial_duration,
    )

    velocity_ratio = np.max(
        np.abs(velocity) / velocity_limits
    )

    acceleration_ratio = np.max(
        np.abs(acceleration) / acceleration_limits
    )

    # Si la durée est multipliée par k :
    # - la vitesse est divisée par k ;
    # - l'accélération est divisée par k².
    required_factors[profile] = max(
        1.0,
        velocity_ratio,
        np.sqrt(acceleration_ratio),
    )

    print(
        f"\n{profile}, a la duree commune initiale :"
    )
    print(
        "  Rapport maximal vitesse / limite : "
        f"{velocity_ratio:.6f}"
    )
    print(
        "  Rapport maximal acceleration / limite : "
        f"{acceleration_ratio:.6f}"
    )


# ------------------------------------------------------------
# 8. Durée finale commune
# ------------------------------------------------------------

required_factor = max(required_factors.values())

if required_factor <= 1.0:
    final_duration = initial_duration
else:
    # Marge supplémentaire de 5 % après l'ajustement articulaire.
    final_duration = (
        initial_duration
        * required_factor
        * 1.05
    )

print(
    f"\nDuree commune finale : {final_duration:.6f} s"
)


# ------------------------------------------------------------
# 9. Export des deux profils
# ------------------------------------------------------------

header = (
    "time_s,"
    "q1,q2,q3,q4,q5,q6,"
    "dq1,dq2,dq3,dq4,dq5,dq6,"
    "ddq1,ddq2,ddq3,ddq4,ddq5,ddq6"
)

for profile in ("cubic", "quintic"):
    q, velocity, acceleration = evaluate(
        profile,
        final_duration,
    )

    peak_velocity = np.max(
        np.abs(velocity),
        axis=0,
    )

    peak_acceleration = np.max(
        np.abs(acceleration),
        axis=0,
    )

    velocity_ratio = np.max(
        peak_velocity / velocity_limits
    )

    acceleration_ratio = np.max(
        peak_acceleration / acceleration_limits
    )

    print(f"\n{profile}, apres ajustement :")

    print(
        "  Vitesses maximales par joint [rad/s] :"
    )
    print(peak_velocity)

    print(
        "  Accelerations maximales par joint [rad/s2] :"
    )
    print(peak_acceleration)

    print(
        "  Rapport final vitesse / limite : "
        f"{velocity_ratio:.6f}"
    )

    print(
        "  Rapport final acceleration / limite : "
        f"{acceleration_ratio:.6f}"
    )

    time = u * final_duration

    output_data = np.column_stack((
        time,
        q,
        velocity,
        acceleration,
    ))

    output_file = output_folder / (
        f"{profile}_{case}_{motion}_joints.csv"
    )

    np.savetxt(
        output_file,
        output_data,
        delimiter=",",
        header=header,
        comments="",
    )

    print(f"  Fichier : {output_file}")


print("\nCalcul hors ligne uniquement.")
print("Aucune execution de mouvement.")

