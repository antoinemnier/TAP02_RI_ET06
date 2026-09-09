#!/usr/bin/env python3
import math
import csv
from pathlib import Path
import sys
# Deplacement vertical : pre_pick -> pick
L = 0.10
z_initial = 0.95

# Utilisation : python3 approach_profiles.py red
# ou          : python3 approach_profiles.py blue
case = sys.argv[1] if len(sys.argv) > 1 else "red"

if case == "red":
    v_max = 0.200
    a_max = 0.300
elif case == "blue":
    v_max = 0.100
    a_max = 0.020
else:
    raise SystemExit("Choisir red ou blue.")

print(f"Limites utilisees : {case}")
# Durees minimales calculees analytiquement
T_cubic = max(
    1.5 * L / v_max,
    math.sqrt(6.0 * L / a_max),
)

T_quintic = max(
    1.875 * L / v_max,
    math.sqrt((10.0 / math.sqrt(3.0)) * L / a_max),
)

# Meme duree pour comparer les profils + marge de 5 %
T = 1.05 * max(T_cubic, T_quintic)

print(f"Duree minimale cubique : {T_cubic:.6f} s")
print(f"Duree minimale quintique : {T_quintic:.6f} s")
print(f"Duree commune retenue : {T:.6f} s")

folder = Path(__file__).resolve().parents[1]
output = folder / "results" / "approach_profiles"
output.mkdir(parents=True, exist_ok=True)

for profile in ["cubic", "quintic"]:
    rows = []

    # 201 instants pour tracer et examiner les profils
    for k in range(201):
        u = k / 200.0
        t = u * T

        if profile == "cubic":
            s = 3*u**2 - 2*u**3
            ds = (6*u - 6*u**2) / T
            dds = (6 - 12*u) / T**2
        else:
            s = 10*u**3 - 15*u**4 + 6*u**5
            ds = (30*u**2 - 60*u**3 + 30*u**4) / T
            dds = (60*u - 180*u**2 + 120*u**3) / T**2

        z = z_initial - L*s
        vz = -L*ds
        az = -L*dds

        rows.append([t, z, vz, az])

    filename = output / f"{profile}_{case}.csv"

    with filename.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["time_s", "z_m", "vz_m_s", "az_m_s2"])
        writer.writerows(rows)

    peak_v = max(abs(row[2]) for row in rows)
    peak_a = max(abs(row[3]) for row in rows)

    print(f"\nProfil : {profile}")
    print(f"Vitesse maximale echantillonnee : {peak_v:.6f} m/s")
    print(f"Acceleration maximale echantillonnee : {peak_a:.6f} m/s2")

    # Deux extremites + trois points intermediaires
    print("Cinq points : temps, position z, vitesse z, acceleration z")
    for k in [0, 50, 100, 150, 200]:
        print("  " + ", ".join(f"{value:.6f}" for value in rows[k]))

    print(f"Fichier : {filename}")
