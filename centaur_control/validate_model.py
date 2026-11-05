"""兼容旧入口；实际实现位于 centaur_control.tests.validate_model。"""

from pathlib import Path
import sys

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from centaur_control.tests.validate_model import *

if __name__ == "__main__":
    main()
