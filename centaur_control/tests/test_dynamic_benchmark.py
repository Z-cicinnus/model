import csv
from pathlib import Path
import tempfile
import unittest

import mujoco
import numpy as np
from centaur_control.benchmark_dynamic import cases, summarize
from centaur_control.run_dynamic_task import reference


class DynamicBenchmarkTests(unittest.TestCase):
    def test_command_coverage_and_continuity(self):
        self.assertEqual(len(cases("basic")), 30)
        self.assertEqual(len(cases("sweep")), 18)
        for t in (4, 8, 20, 24, 28, 32, 36, 40):
            np.testing.assert_allclose(
                reference(t - 1e-7, 0.2, 0.05, 0.05),
                reference(t + 1e-7, 0.2, 0.05, 0.05),
                atol=1e-6,
            )

    def test_material_point_velocity_includes_wheel_rotation(self):
        model = mujoco.MjModel.from_xml_string(
            '<mujoco><worldbody><body pos="0 0 .1"><freejoint/>'
            '<geom type="sphere" size=".1" mass="1"/>'
            "</body></worldbody></mujoco>"
        )
        data = mujoco.MjData(model)
        data.qvel[0] = 0.2
        data.qvel[4] = 2.0
        mujoco.mj_forward(model, data)
        jac = np.zeros((3, model.nv))
        mujoco.mj_jac(model, data, jac, None, np.zeros(3), 1)
        np.testing.assert_allclose(jac @ data.qvel, 0.0, atol=1e-12)
        data.qvel[4] = 0
        np.testing.assert_allclose(jac @ data.qvel, [0.2, 0, 0])

    def test_unloading_duration_and_tracking_screen(self):
        fields = [
            "t",
            "vx",
            "vx_ref",
            "yaw_rate",
            "yaw_ref",
            "roll_deg",
            "roll_ref_deg",
            "pitch_deg",
            "pitch_ref_deg",
            "peak_leg_wheel_util",
        ]
        fields += [
            f"{leg}_{metric}"
            for leg in ("FL", "FR", "RL", "RR")
            for metric in ("N", "slip_mps")
        ]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "trace.csv"
            with path.open("w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=fields)
                writer.writeheader()
                for i in range(1200):
                    row = dict.fromkeys(fields, 0.0)
                    row["t"] = i * 0.01
                    for leg in ("FL", "FR", "RL", "RR"):
                        row[leg + "_N"] = 100.0
                    if i in (600, 601):
                        row["FL_N"] = 0.0
                    writer.writerow(row)
            result = summarize(path)
            self.assertAlmostEqual(result["low_load_seconds"], 0.02)
            self.assertAlmostEqual(result["longest_low_load_seconds"], 0.02)
            self.assertFalse(result["screen_pass"])
            self.assertEqual(result["vx_rmse"], 0.0)


if __name__ == "__main__":
    unittest.main()
