"""兼容原模块导入；新代码请直接使用对应职责模块。"""

from pathlib import Path
import sys

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from centaur_control.paths import MODEL_PATH
from centaur_control.paths import PROJECT_ROOT as ROOT
from centaur_control.models.definitions import HAND_SITES
from centaur_control.models.definitions import WHEEL_SITES
from centaur_control.models.definitions import object_id
from centaur_control.models.definitions import actuator_dofs
from centaur_control.kinematics.jacobians import site_linear_jacobian
from centaur_control.kinematics.jacobians import wheel_contact_jacobian
from centaur_control.kinematics.velocity_qp import solve
