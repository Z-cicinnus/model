"""Smoke integration only: zero controls are NOT standing/locomotion control."""
import argparse
from pathlib import Path
import mujoco
import numpy as np

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/centaur/centaur.xml")
    parser.add_argument("--steps", type=int, default=500)
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(str(Path(args.model).resolve()))
    data = mujoco.MjData(model)
    if model.nkey:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    for _ in range(args.steps):
        data.ctrl[:] = 0
        mujoco.mj_step(model, data)
        if not np.isfinite(data.qpos).all():
            raise RuntimeError("Non-finite qpos during uncontrolled smoke test")
    print(f"Uncontrolled simulation ended: t={data.time:.3f}s; no stability claim")

if __name__ == "__main__":
    main()
