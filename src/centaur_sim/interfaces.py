"""Shared whole-body command and controller protocol."""
from dataclasses import dataclass
from typing import Optional, Protocol, Sequence
import numpy as np

@dataclass(frozen=True)
class WholeBodyCommand:
    forward_velocity_mps: Optional[float] = None
    yaw_rate_radps: Optional[float] = None
    base_height_m: Optional[float] = None
    left_ee_pose: Optional[Sequence[float]] = None
    right_ee_pose: Optional[Sequence[float]] = None

class WholeBodyController(Protocol):
    def compute_control(self, model, data, command: WholeBodyCommand) -> np.ndarray:
        """Return one actuator control vector with shape (model.nu,)."""
        ...
