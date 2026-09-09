#!/usr/bin/env python3
from pathlib import Path
import numpy as np
import yaml
from scipy.interpolate import CubicSpline

folder = Path(__file__).resolve().parents[1]

# Chemin geometrique exporte depuis MATLAB
path_file = folder / "results/cartesian/pre_pick_to_pick_path.csv"
data = np.loadtxt(path_file, delimiter=",", skiprows=1)

s_path = data[:, 0]
q_path = data[:, 1:7]

assert np.all(np.diff(s_path) > 0), "Progression non croissante."

# Une meme interpolation geometrique pour les deux profils.
path = CubicSpline(s_path, q_path, axis=0, extrapolate=False)

# Limites articulaires du projet
limits_file = (
    folder.parent / "et06_m10ia_moveit_config"
    / "config/joint_limits.yaml"
)

config = yaml.safe_load(limits_file.read_text())
limits = config["joint_limits"]

# Choix explicite : conserver les facteurs du test MoveIt precedent.
velocity_scale = 0.1
acceleration_scale = 0.1

v_limit = velocity_scale * np.array([
    limits[f"joint_{j}"]["max_velocity"] for j in range(1, 7)
])

a_limit = acceleration_scale * np.array([
    limits[f"joint_{j}"]["max_acceleration"] for j in range(1, 7)
])

# Duree commune des profils rouges deja calcules
L = 0.10
v_cart = 0.200
a_cart = 0.300

T_cubic = max(1.5*L/v_cart, np.sqrt(6*L/a_cart))
T_quintic = max(
    1.875*L/v_cart,
    np.sqrt((10/np.sqrt(3))*L/a_cart),
)
T_initial = 1.05 * max(T_cubic, T_quintic)

# Grille d'analyse, pas frequence imposee au controleur
u = np.linspace(0.0, 1.0, 2001)


def evaluate(profile, T):
    if profile == "cubic":
        s = 3*u**2 - 2*u**3
        ds = (6*u - 6*u**2) / T
        dds = (6 - 12*u) / T**2
    else:
        s = 10*u**3 - 15*u**4 + 6*u**5
        ds = (30*u**2 - 60*u**3 + 30*u**4) / T
        dds = (60*u - 180*u**2 + 120*u**3) / T**2

    s = np.clip(s, 0.0, 1.0)

    q = path(s)
    dq_ds = path(s, 1)
    d2q_ds2 = path(s, 2)

    velocity = dq_ds * ds[:, None]
    acceleration = (
        d2q_ds2 * ds[:, None]**2
        + dq_ds * dds[:, None]
    )

    return q, velocity, acceleration


# Calcul du facteur d'allongement necessaire pour chaque profil
required = {}

print(f"Duree rouge initiale : {T_initial:.6f} s")
print("Limites articulaires effectives de vitesse :", v_limit)
print("Limites articulaires effectives d'acceleration :", a_limit)

for profile in ["cubic", "quintic"]:
    q, velocity, acceleration = evaluate(profile, T_initial)

    ratio_v = np.max(np.abs(velocity) / v_limit)
    ratio_a = np.max(np.abs(acceleration) / a_limit)

    # Si T est multiplie par k :
    # les vitesses sont divisees par k,
    # les accelerations sont divisees par k^2.
    required[profile] = max(1.0, ratio_v, np.sqrt(ratio_a))

    print(f"\n{profile}, a la duree initiale :")
    print(f"  Rapport maximal vitesse / limite : {ratio_v:.6f}")
    print(f"  Rapport maximal acceleration / limite : {ratio_a:.6f}")

# Meme duree finale pour une comparaison equivalente.
factor = max(required.values())
T_final = T_initial if factor <= 1.0 else T_initial * factor * 1.05

print(f"\nDuree commune ajustee : {T_final:.6f} s")

output = folder / "results/retimed"
output.mkdir(parents=True, exist_ok=True)
header = "time_s,q1,q2,q3,q4,q5,q6,dq1,dq2,dq3,dq4,dq5,dq6,ddq1,ddq2,ddq3,ddq4,ddq5,ddq6"

for profile in ["cubic", "quintic"]:
    q, velocity, acceleration = evaluate(profile, T_final)

    print(f"\n{profile}, apres ajustement :")
    print("  Vitesses maximales par joint [rad/s] :")
    print(np.max(np.abs(velocity), axis=0))
    print("  Accelerations maximales par joint [rad/s2] :")
    print(np.max(np.abs(acceleration), axis=0))

    table = np.column_stack((u*T_final, q, velocity, acceleration))
    filename = output / f"{profile}_red_joints.csv"

    np.savetxt(
        filename,
        table,
        delimiter=",",
        header=header,
        comments="",
    )
    print(f"  Fichier : {filename}")

print("\nCalcul hors ligne uniquement. Aucune execution.")
