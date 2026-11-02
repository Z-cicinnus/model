"""Load MJCF with MuJoCo; no false fallback model."""
from pathlib import Path
import mujoco

def load_model(path: str | Path):
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    model = mujoco.MjModel.from_xml_path(str(path))
    return model, mujoco.MjData(model)
