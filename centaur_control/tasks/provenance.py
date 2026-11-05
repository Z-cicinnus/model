"""记录实验参数、模型版本和执行状态，区分未完成与完成的结果。"""

from datetime import datetime, timezone
import hashlib
import json
import platform

import mujoco
import numpy as np

from centaur_control.paths import MODEL_PATH


def manifest(directory, task, parameters, status):
    record = {"task": task, "status": status, "parameters": parameters,
              "timestamp_utc": datetime.now(timezone.utc).isoformat(),
              "model_sha256": hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest(),
              "mujoco_version": mujoco.__version__, "numpy_version": np.__version__,
              "python_version": platform.python_version()}
    (directory / f"{task}_manifest.json").write_text(
        json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return record
