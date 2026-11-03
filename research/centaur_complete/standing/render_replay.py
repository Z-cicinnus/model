"""Render recorded states reconstructed from joint errors and known targets; no new physics run."""
import os,sys
if sys.platform.startswith('linux') and not os.environ.get('DISPLAY'):
 os.environ.setdefault('MUJOCO_GL','egl')
import argparse
import numpy as np
import mujoco
import imageio.v2 as imageio
from scipy.spatial.transform import Rotation
from PIL import Image,ImageDraw
from controller import ROOT,StandController
p=argparse.ArgumentParser();p.add_argument('--case',default='arms');args=p.parse_args()
m=mujoco.MjModel.from_xml_path(str(ROOT/'centaur_self_collision.xml'));d=mujoco.MjData(m);c=StandController(m,d)
a=np.load(ROOT/'standing/results'/f'{args.case}.npz')['samples']
cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,.55];cam.distance=1.8;cam.azimuth=145;cam.elevation=-12
opt=mujoco.MjvOption();opt.geomgroup[3]=0;opt.sitegroup[:]=0
out=ROOT/'standing/results'/f'{args.case}_replay.mp4'
with mujoco.Renderer(m,height=480,width=640) as renderer,imageio.get_writer(str(out),fps=25,codec='libx264',quality=7) as writer:
 for row in a[::4]:
  d.qpos[:]=c.qref;d.qpos[:3]=row[1:4];quat=Rotation.from_euler('xyz',row[4:7]).as_quat();d.qpos[3:7]=quat[[3,0,1,2]]
  d.qpos[c.qadr]=c.target(row[0]-m.opt.timestep,args.case=='arms')-row[50:83]
  mujoco.mj_forward(m,d);renderer.update_scene(d,cam,scene_option=opt)
  im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im);draw.rectangle((0,0,640,48),fill=(20,30,40));draw.text((12,8),f'Floating-base standing | {args.case} | t={row[0]:.2f} s',fill='white');draw.text((12,27),f'Roll {np.rad2deg(row[4]):+.3f} deg   Pitch {np.rad2deg(row[5]):+.3f} deg   z {row[3]:.4f} m',fill='white');writer.append_data(np.array(im))
print(out)
