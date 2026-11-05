"""兼容原模块导入；新代码请直接使用对应职责模块。"""

from pathlib import Path
import sys

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from centaur_control.models.state import set_joint
from centaur_control.models.state import nominal_state
from centaur_control.models.collision import collision_pairs
from centaur_control.kinematics.forward import forward_kinematics
from centaur_control.kinematics.forward import pose_error
from centaur_control.kinematics.jacobians import jac_site
from centaur_control.kinematics.inverse import inverse_kinematics
from centaur_control.dynamics.stability import DIRECTIONS
from centaur_control.dynamics.stability import support_points
from centaur_control.dynamics.stability import convex_hull
from centaur_control.dynamics.stability import stability
from centaur_control.dynamics.rigid_body import mass_properties
from centaur_control.dynamics.rigid_body import upper_body_ids
from centaur_control.dynamics.rigid_body import dynamics
