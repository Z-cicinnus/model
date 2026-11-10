"""沿零倾角连续可用区间搜索当前控制器的倾角上限。"""

import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
import mujoco
import numpy as np
from .run_dynamic_task import run
from .benchmark_dynamic import summarize


def evaluate(folder, axis, sign, velocity, degrees):
    cmd = [velocity, 0.0, 0.0, 0.0]
    cmd[axis] = sign * np.deg2rad(degrees)
    ramp = max(1.5, degrees / 5.0)
    duration = 4.0 + ramp + 4.0
    path = folder / f"angle_{degrees:05.2f}.csv"
    result = {
        "angle_deg": degrees,
        "ramp_s": ramp,
        "duration_s": duration,
        "tracking_start_s": duration - 2.0,
    }
    try:
        run(
            0.0,
            path,
            command=cmd,
            duration=duration,
            diagnostics=True,
            command_ramp=ramp,
        )
        result.update(summarize(path, tracking_start=duration - 2.0))
        trace = np.genfromtxt(path, delimiter=",", names=True)
        active = trace[trace["t"] >= 4.0]
        result["other_ground_contacts"] = int(active["other_ground_contacts"].max())
        result["min_leg_limit_margin_rad"] = float(
            active["min_leg_limit_margin_rad"].min()
        )
        conditions = {
            "tracking": result["vx_rmse"] <= 0.03
            and result["yaw_rmse"] <= 0.03
            and max(result["roll_rmse_deg"], result["pitch_rmse_deg"]) <= 1.0,
            "contact": result["longest_low_load_seconds"] <= 0.020001,
            "torque": result["saturated_seconds"] <= 0.020001,
            "joint_limits": result["min_leg_limit_margin_rad"] >= -0.02,
            "ground_collision": result["other_ground_contacts"] == 0,
        }
        result["limit_pass"] = all(conditions.values())
        result["failed_criteria"] = [k for k, v in conditions.items() if not v]
    except Exception as error:
        result.update(
            limit_pass=False, failed_criteria=["simulation_error"], error=str(error)
        )
    print(
        f"{folder.name} {degrees:.2f} deg: {result['limit_pass']} {result['failed_criteria']}",
        flush=True,
    )
    return result


def scan(job):
    name, axis, sign, velocity, root, cap = job
    folder = Path(root) / name
    folder.mkdir(parents=True)
    trials = []

    def test(angle):
        r = evaluate(folder, axis, sign, velocity, angle)
        trials.append(r)
        (folder / "trials.json").write_text(
            json.dumps(trials, indent=2), encoding="utf-8"
        )
        return r["limit_pass"]

    low = 0.0
    if not test(0.0):
        return {"case": name, "status": "baseline_failed", "trials": trials}
    high = None
    for angle in np.arange(5.0, cap + 0.01, 5.0):
        if not test(float(angle)):
            high = float(angle)
            break
        low = float(angle)
    if high is not None:
        while high - low > 0.5:
            mid = (low + high) / 2.0
            if test(mid):
                low = mid
            else:
                high = mid
    result = {
        "case": name,
        "velocity_mps": velocity,
        "axis": "roll" if axis == 2 else "pitch",
        "sign": sign,
        "status": "bracketed" if high is not None else "cap_reached",
        "last_pass_deg": low,
        "first_fail_deg": high,
        "trials": trials,
    }
    (folder / "limit.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="dynamic_benchmark/limits")
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--cap", type=int, default=60, choices=range(20, 61, 5))
    parser.add_argument(
        "--view-case",
        choices=[
            f"{mode}_{direction}"
            for mode in ("stationary", "forward")
            for direction in ("left", "right", "front", "back")
        ],
    )
    parser.add_argument("--angle", type=float, default=15.0)
    args = parser.parse_args()
    if args.view_case:
        if not np.isfinite(args.angle) or not 0 <= args.angle <= 60:
            parser.error("--angle must be in [0,60]")
        mode, direction = args.view_case.split("_")
        axis, sign = {
            "left": (2, -1),
            "right": (2, 1),
            "front": (3, 1),
            "back": (3, -1),
        }[direction]
        command = [0.0 if mode == "stationary" else 0.2, 0.0, 0.0, 0.0]
        command[axis] = sign * np.deg2rad(args.angle)
        folder = Path(args.output)
        folder.mkdir(parents=True, exist_ok=True)
        trace = folder / f"view_{args.view_case}_{args.angle:.2f}.csv"
        if trace.exists():
            parser.error("Choose a new --output directory")
        ramp = max(1.5, args.angle / 5.0)
        run(
            0.0,
            trace,
            view=True,
            command=command,
            duration=8.0 + ramp,
            diagnostics=True,
            command_ramp=ramp,
        )
        print(trace.resolve())
        return
    if args.workers < 1:
        parser.error("--workers must be positive")
    root = Path(args.output)
    if root.exists() and any(root.iterdir()):
        parser.error("Choose an empty --output directory")
    root.mkdir(parents=True, exist_ok=True)
    metadata = {
        "mujoco": mujoco.__version__,
        "resolution_deg": 0.5,
        "cap_deg": args.cap,
        "ramp_average_deg_per_s": 5,
        "tracking_last_seconds": 2,
        "max_continuous_low_load_s": 0.02,
        "max_total_saturation_s": 0.02,
        "attitude_rmse_threshold_deg": 1,
        "joint_penetration_tolerance_rad": 0.02,
        "controller_sha256": hashlib.sha256(
            Path(__file__).with_name("run_dynamic_task.py").read_bytes()
        ).hexdigest(),
    }
    (root / "manifest.json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )
    jobs = [
        (f"{mode}_{direction}", axis, sign, v, str(root), args.cap)
        for mode, v in (("stationary", 0.0), ("forward", 0.2))
        for direction, axis, sign in (
            ("left", 2, -1),
            ("right", 2, 1),
            ("front", 3, 1),
            ("back", 3, -1),
        )
    ]
    results = []
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for result in pool.map(scan, jobs):
            results.append(result)
            (root / "limits.json").write_text(
                json.dumps(results, indent=2), encoding="utf-8"
            )
    print(root.resolve())


if __name__ == "__main__":
    main()
