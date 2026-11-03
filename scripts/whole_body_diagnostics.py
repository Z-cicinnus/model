"""Extract MuJoCo generalized mass matrix and wrist/wheel contact Jacobians.

This is a diagnostic of the current model configuration, not a controller.
It requires all 55 original STL assets to be installed locally.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np

BODIES = {
    "left_wrist": "r1a7_left_wrist_yaw_link",
    "right_wrist": "r1a7_right_wrist_yaw_link",
    "front_left_wheel": "as2w_FL_foot",
    "front_right_wheel": "as2w_FR_foot",
    "rear_left_wheel": "as2w_RL_foot",
    "rear_right_wheel": "as2w_RR_foot",
}

def diagnose(model, data, include_matrices=False):
    import mujoco

    mujoco.mj_forward(model, data)
    mass = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, mass, data.qM)
    eig = np.linalg.eigvalsh(mass)
    if not np.isfinite(eig).all():
        raise ValueError("Non-finite mass matrix eigenvalues")
    result = {
        "nq": int(model.nq),
        "nv": int(model.nv),
        "nu": int(model.nu),
        "total_mass_kg": float(model.body_mass[1:].sum()),
        "mass_matrix_shape": list(mass.shape),
        "mass_matrix_symmetry_error": float(np.max(abs(mass - mass.T))),
        "mass_matrix_min_eigenvalue": float(eig.min()),
        "mass_matrix_max_eigenvalue": float(eig.max()),
        "body_jacobians": {},
        "wheel_ground_contact_jacobians": [],
    }
    if include_matrices:
        result["mass_matrix"] = mass.tolist()

    for label, name in BODIES.items():
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            result["body_jacobians"][label] = {"missing_body": name}
            continue
        jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
        mujoco.mj_jac(model, data, jp, jr, data.xpos[body_id], body_id)
        item = {
            "body": name,
            "point_world_m": data.xpos[body_id].tolist(),
            "reference": "body origin, not a calibrated wrist TCP or wheel center",
            "shape": list(jp.shape),
            "linear_rank": int(np.linalg.matrix_rank(jp)),
            "angular_rank": int(np.linalg.matrix_rank(jr)),
        }
        if include_matrices:
            item["linear"] = jp.tolist()
            item["angular"] = jr.tolist()
        result["body_jacobians"][label] = item

    feet = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name): name
        for name in BODIES.values() if name.startswith("as2w_")
    }
    for index in range(data.ncon):
        contact = data.contact[index]
        b1 = int(model.geom_bodyid[contact.geom1])
        b2 = int(model.geom_bodyid[contact.geom2])
        if b1 in feet and b2 == 0:
            body_id = b1
        elif b2 in feet and b1 == 0:
            body_id = b2
        else:
            continue
        jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
        mujoco.mj_jac(model, data, jp, jr, contact.pos, body_id)
        item = {
            "contact_index": index,
            "wheel_body": feet[body_id],
            "point_world_m": contact.pos.tolist(),
            "shape": list(jp.shape),
        }
        if include_matrices:
            item["linear"] = jp.tolist()
            item["angular"] = jr.tolist()
        result["wheel_ground_contact_jacobians"].append(item)

    result["caveats"] = [
        "Only existing foot-to-world ground contacts have contact Jacobians.",
        "Wrist Jacobians use body origins, not validated gripper tool centers.",
        "Diagnostic results do not demonstrate stable whole-body control.",
    ]
    return result

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model", type=Path, default=Path("models/centaur/centaur.xml")
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--full-matrices", action="store_true")
    args = parser.parse_args()
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    if model.nkey:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    result = diagnose(model, data, args.full_matrices)
    result["model_path"] = str(args.model)
    output = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output + "\n", encoding="utf-8")
    print(output)

if __name__ == "__main__":
    main()
