"""Print model dimensions, joints, actuators after required assets are installed."""
import argparse
from pathlib import Path
import mujoco

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/centaur/centaur.xml")
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(str(Path(args.model).resolve()))
    print(f"nbody={model.nbody}, njnt={model.njnt}, nq={model.nq}, nv={model.nv}, nu={model.nu}")
    for idx in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx)
        print(f"ACTUATOR {idx:02d}: {name}, gear={model.actuator_gear[idx,0]:g}")

if __name__ == "__main__":
    main()
