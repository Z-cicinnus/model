"""Whole-system matrices and finite-difference verification. Run from any directory."""
from pathlib import Path
import json, hashlib
import numpy as np
import mujoco
R=Path(__file__).resolve().parents[1]; OUT=Path(__file__).resolve().parent/'results'; OUT.mkdir(exist_ok=True)
file=R/'centaur_self_collision.xml'; m=mujoco.MjModel.from_xml_path(str(file))
s=json.loads((R/'static_support/results.json').read_text()); assert s['model_sha256']==hashlib.sha256(file.read_bytes()).hexdigest()
q0=np.array(s['cases'][0]['qpos']); base=m.body('as2w_base_link').id
hands=[m.body('r1a7_'+side+'_wrist_yaw_link').id for side in ['left','right']]
legs=['FL','FR','RL','RR']; wheels=[m.geom('as2w_'+l+'_foot_collision_1').id for l in legs]
points=np.array([s['cases'][0]['contact_positions_world_m'][l] for l in legs])
d=mujoco.MjData(m);d.qpos[:]=q0;mujoco.mj_forward(m,d)
bodies=hands+[int(m.geom_bodyid[g]) for g in wheels]
local=[np.zeros(3) for _ in hands]+[d.xmat[b].reshape(3,3).T@(p-d.xpos[b]) for b,p in zip(bodies[2:],points)]
labels=['left_wrist_origin','right_wrist_origin']+legs
names=[]
for v in range(m.nv):
 j=int(m.dof_jntid[v]);names.append(m.joint(j).name+(':'+str(v-m.jnt_dofadr[j]) if j==0 else ''))
upper=[i for i,n in enumerate(names) if n.startswith('r1a7_')]; arm=[i for i,n in enumerate(names) if 'shoulder' in n or 'elbow' in n or 'wrist' in n]
def state(q,v=None):
 d=mujoco.MjData(m);d.qpos[:]=q
 if v is not None:d.qvel[:]=v
 mujoco.mj_forward(m,d);return d
def matrices(d):
 M=np.zeros((m.nv,m.nv));mujoco.mj_fullM(m,M,d.qM);J=[]
 for b,p in zip(bodies,local):
  jp=np.zeros((3,m.nv));jr=jp.copy();mujoco.mj_jac(m,d,jp,jr,d.xpos[b]+d.xmat[b].reshape(3,3)@p,b);J.append(np.vstack([jp,jr]))
 C=np.zeros((3,m.nv));mujoco.mj_jacSubtreeCom(m,d,C,base)
 return M,np.array(J),C
cases={'home':{},'both_forward':{'left_shoulder_pitch_joint':-.3,'right_shoulder_pitch_joint':-.3},'left_only':{'left_shoulder_pitch_joint':-.3},'opposed':{'left_shoulder_pitch_joint':-.3,'right_shoulder_pitch_joint':.3}}
reports=[];rng=np.random.default_rng(42)
for label,changes in cases.items():
 q=q0.copy()
 for n,x in changes.items():q[m.joint('r1a7_'+n).qposadr[0]]+=x
 d=state(q);M,J,C=matrices(d);eps=1e-6;err=0.;cerr=0.
 for k in range(m.nv):
  vel=np.eye(m.nv)[k];qp=q.copy();qm=q.copy();mujoco.mj_integratePos(m,qp,vel,eps);mujoco.mj_integratePos(m,qm,vel,-eps);dp=state(qp);dm=state(qm)
  cerr=max(cerr,float(np.max(abs((dp.subtree_com[base]-dm.subtree_com[base])/(2*eps)-C[:,k]))))
  for i,(b,p) in enumerate(zip(bodies,local)):
   Rp=dp.xmat[b].reshape(3,3);Rm=dm.xmat[b].reshape(3,3);R0=d.xmat[b].reshape(3,3)
   linear=(dp.xpos[b]+Rp@p-dm.xpos[b]-Rm@p)/(2*eps)
   W=((Rp-Rm)/(2*eps))@R0.T; angular=np.array([W[2,1]-W[1,2],W[0,2]-W[2,0],W[1,0]-W[0,1]])/2
   err=max(err,float(np.max(abs(np.r_[linear,angular]-J[i,:,k]))))
 eig=np.linalg.eigvalsh(M);v=rng.normal(0,.1,m.nv);a=rng.normal(0,.2,m.nv)
 dv=state(q,v);bias=dv.qfrc_bias.copy();dv.qacc[:]=a;mujoco.mj_rne(m,dv,1,dv.qfrc_inverse)
 residual=float(np.max(abs(dv.qfrc_inverse-(M@a+bias))))
 # Unit generalized shoulder torque response; incremental about zero-velocity equilibrium.
 t=np.zeros(m.nv);t[m.joint('r1a7_left_shoulder_pitch_joint').dofadr[0]]=1
 free=np.linalg.solve(M,t);A=J[2:,:3,:].reshape(12,m.nv);X=np.linalg.solve(M,A.T)
 lam=-np.linalg.pinv(A@X,rcond=1e-10)@(A@free);constrained=free+X@lam
 B=M[np.ix_(range(6),upper)];normalized=B/np.sqrt(np.outer(np.diag(M)[:6],np.diag(M)[upper]))
 internal=[float(c.dist) for c in d.contact if m.geom_bodyid[c.geom1]>0 and m.geom_bodyid[c.geom2]>0 and c.dist < -1e-6]
 legal=all(not m.jnt_limited[j] or m.jnt_range[j,0]<=q[m.jnt_qposadr[j]]<=m.jnt_range[j,1] for j in range(1,m.njnt))
 np.savez(OUT/(label+'.npz'),qpos=q,mass_matrix=M,jacobians=J,com_jacobian=C,com=d.subtree_com[base],gravity_bias=d.qfrc_bias,velocity=v,velocity_bias=bias,contact_jacobian=A,unit_torque=t,free_acceleration=free,ideal_contact_acceleration=constrained,ideal_contact_force_increment=lam,normalized_base_upper_block=normalized)
 for name,arr in [('mass_matrix',M),('left_jacobian',J[0]),('right_jacobian',J[1]),('contact_jacobian',A),('com_jacobian',C)]:np.savetxt(OUT/(label+'_'+name+'.csv'),arr,delimiter=',')
 row=dict(case=label,com_world_m=d.subtree_com[base].tolist(),min_mass_eigenvalue=float(eig.min()),mass_symmetry_error=float(abs(M-M.T).max()),jacobian_fd_max_error=err,com_fd_max_error=cerr,inverse_dynamics_residual=residual,base_upper_normalized_frobenius=float(np.linalg.norm(normalized)),free_base_acceleration=free[:6].tolist(),ideal_contact_base_acceleration=constrained[:6].tolist(),ideal_contact_constraint_residual=float(abs(A@constrained).max()),internal_contact_count=len(internal),joint_limits_legal=bool(legal))
 assert eig.min()>0 and err<1e-6 and cerr<1e-6 and residual<1e-8 and row['ideal_contact_constraint_residual']<1e-7
 reports.append(row)
(OUT/'report.json').write_text(json.dumps(dict(model_sha256=hashlib.sha256(file.read_bytes()).hexdigest(),mujoco_version=mujoco.__version__,nq=m.nq,nv=m.nv,total_mass_kg=float(m.body_mass.sum()),dof_columns=names,jacobian_order=labels,contact_local_points=local[2:],cases=reports),ensure_ascii=False,indent=2,default=lambda x:x.tolist()))
print(json.dumps(reports,indent=2))
