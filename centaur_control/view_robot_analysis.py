"""兼容旧入口；实际实现位于 centaur_control.visualization.viewer。"""

from pathlib import Path
import sys

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from centaur_control.visualization.viewer import *

if __name__ == "__main__":
    main()
