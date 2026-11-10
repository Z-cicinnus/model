import io
import unittest
from argparse import Namespace
from pathlib import Path
import numpy as np
import mujoco as mj
from centaur_control.models.state import nominal_state
from centaur_control.controllers.wbc import WholeBodyController
from centaur_control.run_wbc_suite import WBCTask


class WBCTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model=mj.MjModel.from_xml_path(str(Path(__file__).resolve().parents[1]/'scene.xml'))

    def state(self):
        m=self.model;d=nominal_state(m)
        for k in ('FL','FR','RL','RR'):
            a=m.actuator(k+'_foot_pd').id
            m.actuator_biasprm[a,:]=0;m.actuator_gainprm[a,0]=1
            m.actuator_ctrlrange[a]=m.actuator_forcerange[a]
        return m,d,WholeBodyController(m,d)

    def test_joint_and_actuator_force_limits(self):
        m,d,c=self.state()
        self.assertEqual(c.force_upper[m.actuator('left_shoulder_pitch').id],60)
        self.assertEqual(c.force_upper[m.actuator('FL_foot_pd').id],15)
        self.assertTrue(np.all(c.force_upper>c.force_lower))
        c.update(m,d,[0,0,0,0],d.xpos[c.root,2],{})
        mj.mj_forward(m,d)
        self.assertLessEqual(np.max(d.actuator_force-c.force_upper),1e-5)
        self.assertLessEqual(np.max(c.force_lower-d.actuator_force),1e-5)
        self.assertLess(c.stats['constraint_violation'],.02)

    def test_rolling_includes_wheel_spin(self):
        m,d,c=self.state();d.qvel[:]=0;d.qvel[0]=.2
        d.qvel[c.dofs[c.wheels]]=.2/c.radius
        for gid in c.gids:
            jac=np.zeros((3,m.nv))
            mj.mj_jac(m,d,jac,None,d.geom_xpos[gid]-[0,0,c.radius],int(m.geom_bodyid[gid]))
            np.testing.assert_allclose(jac@d.qvel,0,atol=1e-8)

    def test_moving_contact_acceleration(self):
        m,d,c=self.state();d.qvel[:]=0;d.qvel[0]=.2
        d.qvel[c.dofs[c.wheels]]=.2/c.radius
        mj.mj_forward(m,d)
        for gid in c.gids:
            point=d.geom_xpos[gid]-[0,0,c.radius]
            jp=np.zeros((3,m.nv));jr=jp.copy();dp=jp.copy()
            body=int(m.geom_bodyid[gid])
            mj.mj_jac(m,d,jp,jr,point,body)
            mj.mj_jacDot(m,d,dp,None,point,body)
            omega=jr@d.qvel
            bias=dp@d.qvel+np.cross(omega,np.cross(omega,[0,0,c.radius]))
            np.testing.assert_allclose(bias,0,atol=1e-8)

    def test_no_force_or_baseline_oracle(self):
        m,d,c=self.state();h=d.xpos[c.root,2]
        d.ctrl[:]=0;c.update(m,d,[0,0,0,0],h,{})
        first=d.ctrl.copy()
        d.ctrl[:]=.5;d.xfrc_applied[c.root,0]=120
        c.update(m,d,[0,0,0,0],h,{})
        np.testing.assert_allclose(first,d.ctrl,atol=1e-8)

    def test_task_upper_reference_and_force_timing(self):
        m,d,c=self.state()
        args=Namespace(task='upper',arm='left',force=[120,0,0],angle=5,height=None,lift=.02,manual=None,replay=None)
        task=WBCTask(args,io.StringIO());task.qplog=io.StringIO();task.setup(m,d,c.root)
        observed=[]
        def fake(m,d,command,height,joints):
            observed.append(dict(joints));d.ctrl[:]=.123
        task.controller.update=fake
        d.time=26;task.update(m,d,None);task.apply(m,d)
        self.assertEqual(observed[-1]['left_shoulder_pitch_joint'],-.4)
        self.assertEqual(observed[-1]['right_shoulder_pitch_joint'],0)
        np.testing.assert_allclose(d.ctrl,.123)
        d.time=10.2;task.update(m,d,None);task.apply(m,d)
        np.testing.assert_allclose(d.xfrc_applied[c.root],[120,0,0,0,0,0])
        d.time=10.5;task.update(m,d,None);task.apply(m,d)
        np.testing.assert_allclose(d.xfrc_applied[c.root],0)

if __name__=='__main__':unittest.main()
