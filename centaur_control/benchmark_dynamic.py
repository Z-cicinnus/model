"""批量验证当前控制器。python -m centaur_control.benchmark_dynamic --suite basic"""

import argparse
from concurrent.futures import ProcessPoolExecutor
import csv
import hashlib
import json
import os
from pathlib import Path
import sys

# Avoid CPU oversubscription with multiple independent simulations.
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
import mujoco
import numpy as np
from .run_dynamic_task import run


def cases(suite):
    result = {}
    if suite == "basic":
        for velocity in (-0.2, 0.2):
            for axis in ("roll", "pitch"):
                for angle in (-15, -10, -5, 5, 10, 15):
                    cmd = [velocity, 0.0, 0.0, 0.0]
                    cmd[2 if axis == "roll" else 3] = np.deg2rad(angle)
                    result[f"v{velocity:+.2f}_{axis}{angle:+d}"] = cmd
        for yaw in (-0.05, 0.05):
            for roll in (-15, 0, 15):
                result[f"turn{yaw:+.2f}_roll{roll:+d}"] = [
                    0.05,
                    yaw,
                    np.deg2rad(roll),
                    0.0,
                ]
    else:
        for velocity in (0.05, 0.25, 0.5):
            for yaw in (0.05, 0.1, 0.2):
                for roll in (0, -15):
                    result[f"v{velocity:.2f}_turn{yaw:.2f}_roll{roll:+d}"] = [
                        velocity,
                        yaw,
                        np.deg2rad(roll),
                        0.0,
                    ]
    return result


def summarize(path, tracking_start=10.0):
    a = np.genfromtxt(path, delimiter=",", names=True)
    a = np.atleast_1d(a)
    active = a[a["t"] >= 4.0]
    steady = a[a["t"] >= tracking_start]
    if not len(steady):
        raise ValueError("Incomplete trace: no steady-state evaluation interval")
    dt = float(np.median(np.diff(a["t"])))
    force = np.column_stack([active[f"{leg}_N"] for leg in ("FL", "FR", "RL", "RR")])
    slip = np.column_stack(
        [active[f"{leg}_slip_mps"] for leg in ("FL", "FR", "RL", "RR")]
    )
    low = force.min(axis=1) < 1.0
    longest = current = 0
    for item in low:
        current = current + 1 if item else 0
        longest = max(longest, current)
    result = {}
    for label, actual, target in (
        ("vx_rmse", "vx", "vx_ref"),
        ("yaw_rmse", "yaw_rate", "yaw_ref"),
        ("roll_rmse_deg", "roll_deg", "roll_ref_deg"),
        ("pitch_rmse_deg", "pitch_deg", "pitch_ref_deg"),
    ):
        result[label] = float(np.sqrt(np.mean((steady[actual] - steady[target]) ** 2)))
    result.update(
        min_wheel_N=float(force.min()),
        low_load_seconds=float(low.sum() * dt),
        longest_low_load_seconds=float(longest * dt),
        slip_rms_mps=float(np.sqrt(np.nanmean(slip**2))),
        peak_util=float(active["peak_leg_wheel_util"].max()),
        saturated_seconds=float(np.sum(active["peak_leg_wheel_util"] >= 0.99) * dt),
    )
    # Screening criteria, not a formal stability certificate.
    result["screen_pass"] = bool(
        result["vx_rmse"] <= 0.03
        and result["yaw_rmse"] <= 0.03
        and result["roll_rmse_deg"] <= 1.0
        and result["pitch_rmse_deg"] <= 1.0
        and result["low_load_seconds"] == 0.0
        and result["saturated_seconds"] == 0.0
    )
    return result


def simulate(job):
    name, command, folder = job
    path = Path(folder) / (name + ".csv")
    try:
        run(0.0, path, command=command, duration=12.0, diagnostics=True)
        return {"case": name, "status": "complete", **summarize(path)}
    except Exception as error:
        return {
            "case": name,
            "status": "error",
            "error": str(error),
            "screen_pass": False,
        }


def plot(folder, rows):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    good = [r for r in rows if r["status"] == "complete"]
    if not good:
        return
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    ids = np.arange(len(good))
    axes[0].bar(ids, [r["min_wheel_N"] for r in good])
    axes[0].axhline(1, color="r", linestyle="--")
    axes[0].set_ylabel("Minimum wheel load (N)")
    axes[1].bar(ids, [max(r["roll_rmse_deg"], r["pitch_rmse_deg"]) for r in good])
    axes[1].axhline(1, color="r", linestyle="--")
    axes[1].set_ylabel("Max attitude RMSE (deg)")
    axes[2].bar(ids, [r["slip_rms_mps"] for r in good])
    axes[2].set_ylabel("Contact slip RMS (m/s)")
    axes[2].set_xticks(ids, [r["case"] for r in good], rotation=90, fontsize=7)
    fig.tight_layout()
    fig.savefig(Path(folder) / "overview.png", dpi=150)
    plt.close(fig)
    for row in good:
        a = np.genfromtxt(
            Path(folder) / (row["case"] + ".csv"), delimiter=",", names=True
        )
        fig, axes = plt.subplots(6, 1, figsize=(10, 12), sharex=True)
        for ax, measured, reference in (
            (axes[0], "vx", "vx_ref"),
            (axes[1], "yaw_rate", "yaw_ref"),
            (axes[2], "roll_deg", "roll_ref_deg"),
            (axes[3], "pitch_deg", "pitch_ref_deg"),
        ):
            ax.plot(a["t"], a[measured], label=measured)
            ax.plot(a["t"], a[reference], "--", label=reference)
            ax.legend(loc="upper right")
        for leg in ("FL", "FR", "RL", "RR"):
            axes[4].plot(a["t"], a[leg + "_N"], label=leg)
        axes[4].set_ylabel("Wheel load (N)")
        axes[5].set_xlabel("Time (s)")
        axes[4].legend()
        axes[5].plot(a["t"], a["peak_leg_wheel_util"])
        axes[5].axhline(0.99, color="r", linestyle="--")
        axes[5].set_ylabel("Peak torque utilization")
        fig.suptitle(row["case"])
        fig.tight_layout()
        fig.savefig(Path(folder) / (row["case"] + ".png"), dpi=100)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--suite", choices=("basic", "sweep"), default="basic")
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--output")
    parser.add_argument("--case", help="Run/view one exact case identifier")
    parser.add_argument("--view", action="store_true")
    args = parser.parse_args()
    definitions = cases(args.suite)
    if args.workers < 1:
        parser.error("--workers must be positive")
    if args.case:
        if args.case not in definitions:
            parser.error("Unknown case. Available: " + ", ".join(definitions))
        definitions = {args.case: definitions[args.case]}
    if args.view and not args.case:
        parser.error("--view requires --case")
    folder = Path(args.output or f"dynamic_benchmark/{args.suite}")
    folder.mkdir(parents=True, exist_ok=True)
    targets = [folder / (name + ".csv") for name in definitions]
    if any(p.exists() for p in targets):
        parser.error(
            "Output already contains these traces. Choose a new --output folder."
        )
    model_path = Path(__file__).with_name("centaur_unified.xml")
    metadata = {
        "mujoco": mujoco.__version__,
        "python": sys.version,
        "duration_s": 12,
        "tracking_window_s": [10, 12],
        "contact_window_s": [4, 12],
        "controller_sha256": hashlib.sha256(
            Path(__file__).with_name("run_dynamic_task.py").read_bytes()
        ).hexdigest(),
        "model_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
        "commands_vx_yaw_roll_pitch_SI": definitions,
    }
    (folder / "manifest.json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )
    if args.view:
        run(
            0.0,
            targets[0],
            view=True,
            command=next(iter(definitions.values())),
            duration=12.0,
            diagnostics=True,
        )
        rows = [{"case": args.case, "status": "complete", **summarize(targets[0])}]
    else:
        jobs = [(name, cmd, str(folder)) for name, cmd in definitions.items()]
        rows = []
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            for row in pool.map(simulate, jobs):
                rows.append(row)
                print(
                    f"[{len(rows)}/{len(jobs)}] {row['case']}: {row['status']}, pass={row['screen_pass']}",
                    flush=True,
                )
                (folder / "results.json").write_text(
                    json.dumps(rows, indent=2), encoding="utf-8"
                )
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with (folder / "summary.csv").open("w", newline="", encoding="utf-8-sig") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    plot(folder, rows)
    print(f"Saved {folder.resolve()}")
    if any(r["status"] == "error" for r in rows):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
