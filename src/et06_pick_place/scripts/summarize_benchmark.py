#!/usr/bin/env python3
import json
import math
import statistics
import sys
from collections import Counter
from pathlib import Path


def joint_length(points):
    return sum(
        math.sqrt(sum(
            (b - a) ** 2
            for a, b in zip(p["positions"], q["positions"])
        ))
        for p, q in zip(points[:-1], points[1:])
    )


def report_metric(label, values):
    if not values:
        print(f"  {label} : aucune donnee")
        return

    print(
        f"  {label} : "
        f"mediane={statistics.median(values):.6g}, "
        f"min={min(values):.6g}, "
        f"max={max(values):.6g}"
    )


def main():
    if len(sys.argv) != 2:
        raise SystemExit(
            "Usage : python3 summarize_benchmark.py DOSSIER_RESULTATS"
        )

    folder = Path(sys.argv[1])
    files = sorted(folder.glob("*.json"))

    if not files:
        raise SystemExit(f"Aucun fichier JSON dans : {folder}")

    records = [json.loads(path.read_text()) for path in files]

    for planner in ["RRTConnect", "RRTstar"]:
        runs = [r for r in records if r["planner"] == planner]
        successful = [
            r for r in runs
            if r["error_code"] == 1 and r["points"]
        ]

        print(f"\n=== {planner} ===")
        print(f"Succes : {len(successful)}/{len(runs)}")
        print(
            "Codes retour :",
            dict(Counter(r["error_code"] for r in runs)),
        )

        print("\nTous les essais :")
        report_metric(
            "Temps total appel [s]",
            [r["service_wall_time_s"] for r in runs],
        )

        print("\nEssais reussis uniquement :")
        report_metric(
            "Temps rapporte MoveIt [s]",
            [r["planning_time_s"] for r in successful],
        )
        report_metric(
            "Longueur articulaire [rad]",
            [joint_length(r["points"]) for r in successful],
        )
        durations = []
        for r in successful:
            first_time = r["points"][0]["time_s"]
            last_time = r["points"][-1]["time_s"]
            durations.append(last_time - first_time)

        report_metric("Duree trajectoire [s]", durations)
        print("\nDetail des echecs :")
        failures = [
            r for r in runs
            if r["error_code"] != 1 or not r["points"]
        ]

        if not failures:
            print("  Aucun.")

        for r in failures:
            print(
                f"  Essai {r['trial']:02d} : "
                f"code={r['error_code']}, "
                f"points={len(r['points'])}, "
                f"temps={r['planning_time_s']:.6f} s"
            )
            print(f"    message={r.get('error_message', '')!r}")
            print(f"    source={r.get('error_source', '')!r}")


if __name__ == "__main__":
    main()
