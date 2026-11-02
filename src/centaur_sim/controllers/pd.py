"""Joint PD for direct torque motors only; no wheel/velocity-servo assumptions."""
from dataclasses import dataclass
from typing import Mapping
import numpy as np
import mujoco

@dataclass(frozen=True)
class JointPDGain:
    kp: float
    kd: float

class MotorPDController:
    def __init__(self, gains: Mapping[str, JointPDGain]):
        self.gains = dict(gains)

    def compute_control(self, model, data, joint_targets: Mapping[str, float]) -> np.ndarray:
        ctrl = np.zeros(model.nu, dtype=float)
        for joint_name, target in joint_targets.items():
            if joint_name not in self.gains:
                raise KeyError(f"Missing PD gains for {joint_name}")
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if jid < 0:
                raise KeyError(f"Unknown joint: {joint_name}")
            if model.jnt_type[jid] not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
                raise ValueError("PD baseline supports only scalar hinge/slide joints")
            matches = [
                i for i in range(model.nu)
                if model.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT
                and model.actuator_trnid[i, 0] == jid
            ]
            if len(matches) != 1:
                raise ValueError(f"Expected one direct actuator for {joint_name}; found {len(matches)}")
            aid = matches[0]
            if (model.actuator_dyntype[aid] != mujoco.mjtDyn.mjDYN_NONE
                or model.actuator_gaintype[aid] != mujoco.mjtGain.mjGAIN_FIXED
                or model.actuator_biastype[aid] != mujoco.mjtBias.mjBIAS_NONE):
                raise ValueError(f"Actuator for {joint_name} is not a direct motor")
            ratio = model.actuator_gear[aid, 0] * model.actuator_gainprm[aid, 0]
            if abs(ratio) < 1e-12:
                raise ValueError("Invalid motor gear/gain")
            q = float(data.qpos[model.jnt_qposadr[jid]])
            qd = float(data.qvel[model.jnt_dofadr[jid]])
            g = self.gains[joint_name]
            u = (g.kp * (float(target) - q) - g.kd * qd) / ratio
            if model.actuator_ctrllimited[aid]:
                u = np.clip(u, *model.actuator_ctrlrange[aid])
            ctrl[aid] = u
        return ctrl
