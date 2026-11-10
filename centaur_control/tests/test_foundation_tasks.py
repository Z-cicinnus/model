import io
import unittest
from argparse import Namespace
from pathlib import Path
import mujoco as mj
import numpy as np
from centaur_control.run_foundation_tasks import FoundationTask
from centaur_control.models.state import nominal_state

class FoundationTests(unittest.TestCase):
    def setUp(self):
        self.m=mj.MjModel.from_xml_path(str(Path(__file__).resolve().parents[1]/'scene.xml'))
        self.d=nominal_state(self.m)
        self.task=FoundationTask(Namespace(task='upper',height=None,lift=.02,angle=5,manual=None,replay=None),io.StringIO())
        self.task.setup(self.m,self.d,self.m.body('base_link').id)
    def test_references_do_not_teleport(self):
        self.d.time=50;before=self.d.qpos.copy()
        self.task.update(self.m,self.d,None);self.task.apply(self.m,self.d)
        np.testing.assert_array_equal(self.d.qpos,before)
        self.assertGreater(self.task.height,self.task.h0)
    def test_force_does_not_accumulate(self):
        t=self.task;t.update(self.m,self.d,None)
        t.wrenches=[dict(body='base_link',world_wrench=[10,0,0,0,0,0])]
        t.apply(self.m,self.d);t.apply(self.m,self.d)
        self.assertEqual(self.d.xfrc_applied[t.root,0],10)
        t.wrenches=[];t.apply(self.m,self.d)
        self.assertEqual(self.d.xfrc_applied[t.root,0],0)
    def test_invalid_target(self):
        self.task.manual={'joints':{'left_shoulder_pitch_joint':100}}
        with self.assertRaises(ValueError):self.task.update(self.m,self.d,None)
    def test_continuity(self):
        for kind in ['move','posture','upper']:
            self.task.args.task=kind
            for t in range(6,72,6):
                np.testing.assert_allclose(self.task.preset(t-1e-7),self.task.preset(t+1e-7),atol=1e-6)
