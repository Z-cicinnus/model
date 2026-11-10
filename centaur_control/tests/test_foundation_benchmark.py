import io
import unittest
from argparse import Namespace
from pathlib import Path
import numpy as np
import mujoco
from centaur_control.benchmark_foundation import Experiment, pulse
from centaur_control.compare_foundation import recovery_time
from centaur_control.models.state import nominal_state

class BenchmarkTests(unittest.TestCase):
    def test_pulse_boundaries_and_impulse(self):
        t=np.arange(0,54,.01)
        force=np.array([5 if pulse(x,10,.5) else 0 for x in t])
        self.assertAlmostEqual(force.sum()*.01,2.5)
        self.assertFalse(pulse(10.5-1e-12,10,.5))
        self.assertTrue(pulse(10-1e-12,10,.5))
    def test_recovery_requires_continuous_hold(self):
        t=np.arange(0,5,.01);mask=(t>=1)
        near=np.ones(len(t),bool);near[(t>=1.4)&(t<2)]=False
        self.assertAlmostEqual(recovery_time(t,mask,near,1),1)
        self.assertIsNone(recovery_time(t,mask,np.zeros(len(t),bool),1))
    def test_pulse_is_removed_and_single_arm_does_not_change_height(self):
        m=mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parents[1]/'scene.xml'))
        self.assertAlmostEqual(m.opt.timestep,.01)
        d=nominal_state(m)
        args=Namespace(task='move',condition='disturbed',angle=5,height=None,lift=.02,manual=None,replay=None,force=5,force_start=10,force_duration=.5)
        task=Experiment(args,io.StringIO());task.setup(m,d,m.body('base_link').id)
        d.time=10.2;task.update(m,d,None);task.apply(m,d)
        self.assertEqual(d.xfrc_applied[task.root,1],5)
        d.time=10.5;task.update(m,d,None);task.apply(m,d)
        self.assertEqual(d.xfrc_applied[task.root,1],0)
        args.task='upper';d.time=50;task.update(m,d,None)
        self.assertEqual(task.joints['right_shoulder_pitch_joint'],task.nominal['right_shoulder_pitch_joint'])
        self.assertNotEqual(task.joints['left_shoulder_pitch_joint'],0)
        h=task.height;args.condition='baseline';task.update(m,d,None)
        self.assertEqual(task.height,h)
