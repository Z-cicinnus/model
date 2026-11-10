import unittest
import io
import tempfile
import json
from argparse import Namespace
from pathlib import Path
import mujoco
from centaur_control.models.state import nominal_state
from centaur_control.run_comparison_suite import SuiteTask
import numpy as np
from centaur_control.run_comparison_suite import protocol
from centaur_control.evaluate_comparison_suite import longest_run, evaluate
from centaur_control.compare_foundation import recovery_time

class SuiteTests(unittest.TestCase):
    def test_protocol(self):
        p=protocol();cases=p['cases']
        self.assertEqual(len(cases),21)
        self.assertEqual(len(set(c['id'] for c in cases)),21)
        self.assertEqual(p['physics_hz'],100)
        for task in ('move','posture'):
            for f in (40,120):
                v=[c['force'] for c in cases if c['task']==task and max(abs(x) for x in c['force'])==f]
                np.testing.assert_equal(np.sum(v,axis=0),[0,0,0])
                self.assertEqual(len(v),4)
    def test_continuous_not_total_duration(self):
        self.assertEqual(longest_run([1,1,0,1,1,1],.01),.03)
        self.assertEqual(longest_run([], .01),0)
    def test_force_direction_removal_and_right_arm(self):
        m=mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parents[1]/"scene.xml"))
        d=nominal_state(m)
        args=Namespace(task="move",arm="both",force=[-120,0,0],angle=5,height=None,lift=.02,manual=None,replay=None)
        task=SuiteTask(args,io.StringIO());task.setup(m,d,m.body("base_link").id)
        d.time=10.2;task.update(m,d,None);task.apply(m,d)
        np.testing.assert_allclose(d.xfrc_applied[task.root],[-120,0,0,0,0,0])
        d.time=10.5;task.update(m,d,None);task.apply(m,d)
        np.testing.assert_allclose(d.xfrc_applied[task.root],0)
        args.task="upper";args.arm="right";args.force=[0,0,0];d.time=50
        task.update(m,d,None)
        self.assertEqual(task.joints["left_shoulder_pitch_joint"],task.nominal["left_shoulder_pitch_joint"])
        self.assertNotEqual(task.joints["right_shoulder_pitch_joint"],0)

    def test_failed_trials_remain_in_denominator(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);p=protocol();(root/"protocol.json").write_text(json.dumps(p))
            for c in p["cases"]:
                trial=root/c["id"];trial.mkdir()
                (trial/"config.json").write_text(json.dumps(dict(status="failed",physics_hz=100,control_hz=100,mujoco_version="test",sha256={},wall_seconds=0,error="synthetic failure")))
            report=evaluate(root)
            self.assertEqual(report["expected_cases"],21)
            self.assertEqual(report["recorded_cases"],21)
            self.assertEqual(report["passed_cases"],0)
            self.assertEqual(report["completed_cases"],0)

    def test_recovery_hold(self):
        t=np.arange(0,4,.01);near=(t>=1)&(t<1.5)|(t>=2)
        self.assertEqual(recovery_time(t,t>=1,near,1),1)
        self.assertIsNone(recovery_time(t,t>=1,np.zeros(len(t),bool),1))

if __name__=='__main__':unittest.main()
