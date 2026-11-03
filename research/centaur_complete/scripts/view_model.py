"""Assembly viewer. Default is fixed-base inspection, --floating enables free base."""
from pathlib import Path
import argparse
import time
import numpy as np
import mujoco
import mujoco.viewer

parser=argparse.ArgumentParser(description='四轮足半人马装配查看器')
parser.add_argument('--floating',action='store_true',help='运行浮动基座版本；仅有演示关节保持，不是移动控制器')
parser.add_argument('--self-collision',action='store_true',help='加载碰撞修订版本')
args=parser.parse_args()
root=Path(__file__).resolve().parents[1]
file='centaur.xml' if args.floating else 'fixed_base.xml'
if args.self_collision:
    file=file.replace('.xml','_自碰撞.xml')
m=mujoco.MjModel.from_xml_path(str(root/file));d=mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m,d,0);mujoco.mj_forward(m,d)
qadr=np.array([m.jnt_qposadr[m.actuator_trnid[i,0]] for i in range(m.nu)])
vadr=np.array([m.jnt_dofadr[m.actuator_trnid[i,0]] for i in range(m.nu)])
target=d.qpos[qadr].copy()
# Demonstration gains only, not identified physical damping or a full-body controller.
kp=np.array([180 if m.actuator(i).name.startswith('as2w_') else 20 for i in range(m.nu)])
kd=np.array([10 if m.actuator(i).name.startswith('as2w_') else 2 for i in range(m.nu)])
with mujoco.viewer.launch_passive(m,d) as viewer:
    viewer.cam.lookat[:]=[.04,0,.55];viewer.cam.distance=1.9
    viewer.cam.azimuth=145;viewer.cam.elevation=-12
    viewer.opt.geomgroup[3]=0;viewer.opt.sitegroup[:]=0
    while viewer.is_running():
        start=time.perf_counter()
        mujoco.mj_forward(m,d)
        command=d.qfrc_bias[vadr]+kp*(target-d.qpos[qadr])-kd*d.qvel[vadr]
        d.ctrl[:]=np.clip(command,m.actuator_ctrlrange[:,0],m.actuator_ctrlrange[:,1])
        mujoco.mj_step(m,d);viewer.sync()
        time.sleep(max(0,m.opt.timestep-(time.perf_counter()-start)))

