"""Floating-base standing tests. External forces occur only in named push tests."""
from pathlib import Path
import argparse,json,hashlib,time
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from controller import ROOT,PoseController as StandController

def run(case,duration,viewer=False,record=False):
 file=ROOT/'centaur_self_collision.xml'
 reference=json.loads((ROOT/'static_support/results.json').read_text())
 if hashlib.sha256(file.read_bytes()).hexdigest()!=reference['model_sha256']:raise RuntimeError('Model differs from static analysis; rerun scripts/analyze_static_support.py first.')
 m=mujoco.MjModel.from_xml_path(str(file));d=mujoco.MjData(m);c=StandController(m,d);c.reset(d)
 assert not m.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
 assert np.all(m.body_gravcomp==0)
 base=m.body('as2w_base_link').id;legs=['FL','FR','RL','RR'];wheels={m.geom(f'as2w_{s}_foot_collision_1').id:i for i,s in enumerate(legs)}
 out=ROOT/'pose_feedback/results';out.mkdir(exist_ok=True)
 view=None;renderer=None;writer=None
 if viewer:
  import mujoco.viewer as mv
  view=mv.launch_passive(m,d);view.cam.lookat[:]=[0,0,.55];view.cam.distance=1.9;view.cam.azimuth=145;view.cam.elevation=-12;view.opt.geomgroup[3]=0;view.opt.sitegroup[:]=0
 if record:
  import imageio.v2 as imageio
  renderer=mujoco.Renderer(m,height=480,width=640);writer=imageio.get_writer(str(out/(case+'.mp4')),fps=25,codec='libx264',quality=7)
  camera=mujoco.MjvCamera();camera.lookat[:]=[0,0,.55];camera.distance=1.9;camera.azimuth=145;camera.elevation=-12
  opt=mujoco.MjvOption();opt.geomgroup[3]=0;opt.sitegroup[:]=0
 rows=[];maxrp=0;minheight=1;maxheight=0;maxxy=0;saturations=0;internal_steps=0;maxutil=0;allfour=0;lastqvel=0
 total=int(round(duration/m.opt.timestep));stride=max(1,round(.01/m.opt.timestep));vidstride=max(1,round(.04/m.opt.timestep))
 try:
  for k in range(total):
   wall=time.perf_counter();d.xfrc_applied[:]=0;external=np.zeros(3)
   if case in ['push_x','push_y'] and 3<=d.time<3.2:external[0 if case=='push_x' else 1]=30;d.xfrc_applied[base,:3]=external
   assert not np.any(d.qfrc_applied)
   if case=='yaw' and 3<=d.time<3.2:d.xfrc_applied[base,5]=5
   target=c.target(d.time,case=='arms');d.ctrl[:]=c.command(d,case=='arms');saturations+=int(np.any(c.last_raw<m.actuator_ctrlrange[:,0]) or np.any(c.last_raw>m.actuator_ctrlrange[:,1]))
   maxutil=max(maxutil,float(np.max(abs(d.ctrl)/np.maximum(abs(m.actuator_ctrlrange[:,0]),abs(m.actuator_ctrlrange[:,1])))))
   mujoco.mj_step(m,d)
   if not np.isfinite(d.qpos).all():raise RuntimeError('Nonfinite state')
   angles=Rotation.from_quat(d.qpos[[4,5,6,3]]).as_euler('xyz');maxrp=max(maxrp,float(max(abs(angles[:2]))));maxxy=max(maxxy,float(np.linalg.norm(d.qpos[:2])));minheight=min(minheight,float(d.qpos[2]));maxheight=max(maxheight,float(d.qpos[2]))
   internal_steps+=int(any(m.geom_bodyid[x.geom1]>0 and m.geom_bodyid[x.geom2]>0 and x.dist< -1e-6 for x in d.contact))
   if k%stride==0:
    mujoco.mj_forward(m,d);fz=np.zeros(4)
    for ci,con in enumerate(d.contact):
     wheel=con.geom1 if con.geom1 in wheels else con.geom2 if con.geom2 in wheels else None
     if wheel is None or (m.geom_bodyid[con.geom1]!=0 and m.geom_bodyid[con.geom2]!=0):continue
     force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force);world=con.frame.reshape(3,3).T@force[:3]
     fz[wheels[wheel]]+=world[2]*(1 if con.geom2==wheel else -1)
    rows.append(np.r_[d.time,d.qpos[:3],angles,d.qvel[:6],fz,d.ctrl,target-d.qpos[c.qadr],external])
    allfour+=int(np.all(fz>1))
   if renderer and k%vidstride==0:
    renderer.update_scene(d,camera,scene_option=opt);writer.append_data(renderer.render())
   if view and k%stride==0:
    if not view.is_running():raise RuntimeError('Viewer closed before test finished')
    view.sync()
   if view:time.sleep(max(0,m.opt.timestep-(time.perf_counter()-wall)))
 finally:
  if view:view.close()
  if writer:writer.close()
  if renderer:renderer.close()
 a=np.array(rows);tail=a[a[:,0]>=duration-1];warn=[int(x.number) for x in d.warning]
 # Columns: t,xyz,rpy,base_v6,wheel_fz4,ctrl33,joint_error33,push3.
 metric={'case':case,'duration_s':duration,'model_sha256':reference['model_sha256'],'mujoco_version':mujoco.__version__,'max_roll_pitch_deg':float(np.rad2deg(maxrp)),'height_range_m':[minheight,maxheight],'max_xy_displacement_m':maxxy,'final_position_m':d.qpos[:3].tolist(),'final_roll_pitch_yaw_deg':np.rad2deg(angles).tolist(),'final_second_base_velocity_rms':np.sqrt(np.mean(tail[:,7:13]**2,axis=0)).tolist(),'final_second_wheel_load_mean_N':np.mean(tail[:,13:17],axis=0).tolist(),'final_second_joint_error_max_rad':float(np.max(abs(tail[:,50:83]))),'all_four_loaded_sample_fraction':allfour/len(rows),'max_torque_limit_fraction':maxutil,'saturated_steps':saturations,'internal_contact_steps':internal_steps,'engine_warnings':warn,'push_N':30 if case.startswith('push') else 0,'push_duration_s':.2 if case.startswith('push') or case=='yaw' else 0,'push_yaw_torque_Nm':5 if case=='yaw' else 0,'controller':'static torque feedforward, joint PD and longitudinal/yaw base feedback','no_virtual_support':True}
 metric['max_yaw_deg']=float(np.rad2deg(np.max(np.abs(a[:,6]))))
 metric['acceptance_limits']={'roll_pitch_deg':2,'height_error_m':.01,'xy_displacement_m':.03,'internal_contact_steps':0,'saturated_steps':0}
 if case=='yaw':metric['acceptance_limits'].update({'max_yaw_deg':5,'final_yaw_abs_deg':1})
 metric['passed']=bool(maxrp<np.deg2rad(2) and max(abs(minheight-c.qref[2]),abs(maxheight-c.qref[2]))<.01 and maxxy<.03 and not internal_steps and not saturations and not any(warn) and (case!='yaw' or (metric['max_yaw_deg']<5 and abs(metric['final_roll_pitch_yaw_deg'][2])<1)))
 np.savez_compressed(out/(case+'.npz'),samples=a,columns=np.array(['time']+['x','y','z']+['roll','pitch','yaw']+['vx','vy','vz','wx','wy','wz']+['fz_'+s for s in legs]+['ctrl_'+m.actuator(i).name for i in range(m.nu)]+['error_'+m.actuator(i).name for i in range(m.nu)]+['push_x','push_y','push_z']))
 (out/(case+'.json')).write_text(json.dumps(metric,ensure_ascii=False,indent=2));print(json.dumps(metric,ensure_ascii=False),flush=True)
 return metric
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--case',choices=['hold','push_x','push_y','arms','yaw','all'],default='hold');p.add_argument('--duration',type=float);p.add_argument('--viewer',action='store_true');p.add_argument('--record',action='store_true');args=p.parse_args()
 results=[run(case,args.duration or (30 if case=='hold' else 15),args.viewer,args.record) for case in (['hold','push_x','push_y','arms'] if args.case=='all' else [args.case])]
 if not all(x['passed'] for x in results):raise SystemExit('One or more cases failed the stated acceptance limits')
