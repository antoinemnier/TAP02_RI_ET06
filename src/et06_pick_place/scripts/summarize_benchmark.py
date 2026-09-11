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
        print(f"  {label} : no data")
        return

    print(
        f"  {label} : "
        f"mediane={statistics.median(values):.6g}, "
        f"min={min(values):.6g}, "
        f"max={max(values):.6g}"
    )

def acceleration_rms(points):
    if len(points) < 2:
        return None

    joint_count = len(points[0]["positions"])
    if joint_count == 0:
        return None

    for p in points:
        if len(p["accelerations"]) != joint_count:
            return None
        if not all(math.isfinite(a) for a in p["accelerations"]):
            return None

    duration = points[-1]["time_s"] - points[0]["time_s"]
    if duration <= 0:
        return None

    integral = 0.0

    for p, q in zip(points[:-1], points[1:]):
        dt = q["time_s"] - p["time_s"]
        if dt <= 0:
            return None

        a2_start = sum(a * a for a in p["accelerations"])
        a2_end = sum(a * a for a in q["accelerations"])
        integral += 0.5 * (a2_start + a2_end) * dt

    return math.sqrt(integral / duration)

def main():
    if len(sys.argv) != 2:
        raise SystemExit(
            "Usage: python3 summarize_benchmark.py RESULTS_DIRECTORY"
        )

    folder = Path(sys.argv[1])
    files = sorted(folder.glob("*.json"))

    if not files:
        raise SystemExit(f"No JSON file found in : {folder}")

    records = [json.loads(path.read_text()) for path in files]

    for planner in ["RRTConnect", "RRTstar"]:
        runs = [r for r in records if r["planner"] == planner]
        successful = [
            r for r in runs
            if r["error_code"] == 1 and r["points"]
        ]

        print(f"\n=== {planner} ===")
        print(f"Success : {len(successful)}/{len(runs)}")
        print(
            "Return codes :",
            dict(Counter(r["error_code"] for r in runs)),
        )

        print("\nAll trials :")
        report_metric(
            "Total call time [s]",
            [r["service_wall_time_s"] for r in runs],
        )

        print("\nSuccesssful trials only :")
        report_metric(
            "Reported planning time MoveIt [s]",
            [r["planning_time_s"] for r in successful],
        )
        report_metric(
            "Joint-space path length [rad]",
            [joint_length(r["points"]) for r in successful],
        )
        durations = []
        for r in successful:
            first_time = r["points"][0]["time_s"]
            last_time = r["points"][-1]["time_s"]
            durations.append(last_time - first_time)

        report_metric("Trajectory duration [s]", durations)
        rms_values = []

        for r in successful:
            value = acceleration_rms(r["points"])
            if value is not None:
                rms_values.append(value)

        report_metric(
            "Joint acceleration RMS norm [rad/s^2]",
            rms_values,
        )

        print(
            f"  Trajectories with usable acceleration data : "
            f"{len(rms_values)}/{len(successful)}"
        )
        print("\nFailure details :")
        failures = [
            r for r in runs
            if r["error_code"] != 1 or not r["points"]
        ]

        if not failures:
            print("  None.")

        for r in failures:
            print(
                f"  Trial {r['trial']:02d} : "
                f"code={r['error_code']}, "
                f"points={len(r['points'])}, "
                f"temps={r['planning_time_s']:.6f} s"
            )
            print(f"    message={r.get('error_message', '')!r}")
            print(f"    source={r.get('error_source', '')!r}")


if __name__ == "__main__":
    main()
