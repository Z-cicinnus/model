"""Weighted whole-body inverse-dynamics QP for the frozen flat-floor suite.

Variables: generalized acceleration, four 3D wheel forces, mimic reactions.
No external-force oracle or future reference is used. Original servo gains
and limits are retained. The QP is independent of the baseline control output.
"""
import time
import numpy as np
import mujoco as mj
import clarabel
from scipy import sparse
from scipy.spatial.transform import Rotation


class WholeBodyController:
    def __init__(self, m, d):
        self.root = m.body('base_link').id
        self.gids = [m.geom(k+'_foot_wheel').id for k in ('FL','FR','RL','RR')]
        self.joints = m.actuator_trnid[:, 0].copy()
        self.dofs = m.jnt_dofadr[self.joints]
        self.pos = m.jnt_qposadr[self.joints]
        self.nominal = d.qpos[self.pos].copy()
        self.wheels = np.array([m.actuator(k+'_foot_pd').id for k in ('FL','FR','RL','RR')])
        self.legs = np.array([m.actuator(k+'_'+j+'_pd').id for k in ('FL','FR','RL','RR') for j in ('hip','thigh','calf')])
        self.offsets = d.geom_xpos[self.gids]-d.xpos[self.root]
        self.radius = m.geom_size[self.gids[0],0]
        self.unactuated = np.setdiff1d(np.arange(m.nv), self.dofs)
        if not np.allclose(m.actuator_gear[:,0],1):
            raise ValueError('WBC adapter requires unit joint transmission')
        # The source model contains only linear joint mimic equalities.
        self.E = np.zeros((m.neq,m.nv))
        for i in range(m.neq):
            if m.eq_type[i] != mj.mjtEq.mjEQ_JOINT or np.any(m.eq_data[i,2:5]):
                raise ValueError('Unsupported nonlinear model equality')
            j,k=m.eq_obj1id[i],m.eq_obj2id[i]
            self.E[i,m.jnt_dofadr[j]]=1
            self.E[i,m.jnt_dofadr[k]]=-m.eq_data[i,1]
        self.force_lower=np.where(m.actuator_forcelimited,m.actuator_forcerange[:,0],-np.inf)
        self.force_upper=np.where(m.actuator_forcelimited,m.actuator_forcerange[:,1],np.inf)
        joint_limited=m.jnt_actfrclimited[self.joints]
        self.force_lower=np.maximum(self.force_lower,np.where(joint_limited,m.jnt_actfrcrange[self.joints,0],-np.inf))
        self.force_upper=np.minimum(self.force_upper,np.where(joint_limited,m.jnt_actfrcrange[self.joints,1],np.inf))
        self.stats = {}

    def update(self, m, d, command, height, joint_targets):
        start=time.perf_counter()
        nv=m.nv; n=nv+12+m.neq
        H=np.eye(n)*1e-4; g=np.zeros(n)
        def objective(A,b,weight):
            nonlocal H,g
            A=np.atleast_2d(A);b=np.atleast_1d(b)
            H += weight*(A.T@A);g -= weight*(A.T@b)
        def acc_rows(J):
            return np.pad(J,((0,0),(0,n-nv)))
        R=d.xmat[self.root].reshape(3,3)
        roll,pitch,yaw=Rotation.from_matrix(R).as_euler('xyz')
        heading=Rotation.from_euler('z',yaw).as_matrix()
        v,w,r,p=command
        jp=np.zeros((3,nv));jr=jp.copy();dp=jp.copy();dr=jp.copy()
        mj.mj_jacBody(m,d,jp,jr,self.root)
        mj.mj_jacDot(m,d,dp,dr,d.xpos[self.root],self.root)
        velocity=jp@d.qvel;omega=jr@d.qvel
        lin=8*(heading[:,0]*v-velocity)
        lin[2]=140*(height-d.xpos[self.root,2])-24*velocity[2]
        desired=Rotation.from_euler('xyz',[r,p,yaw]).as_matrix()
        rot_error=Rotation.from_matrix(desired@R.T).as_rotvec()
        angular=160*rot_error+24*(np.array([0,0,w])-omega)
        objective(acc_rows(jp),lin-dp@d.qvel,100)
        objective(acc_rows(jr),angular-dr@d.qvel,100)
        qref=self.nominal.copy()
        for name,value in joint_targets.items():
            j=m.joint(name).id
            qref[np.where(self.joints==j)[0][0]]=value
        for i,gid in enumerate(self.gids):
            ids=self.legs[3*i:3*i+3]
            target_world=self.offsets[i]@heading.T
            target_world[2]=self.radius-height
            target_local=target_world@desired
            err=(target_local-(d.geom_xpos[gid]-d.xpos[self.root])@R)@R.T
            mj.mj_jacGeom(m,d,jp,None,gid)
            jac=jp[:,self.dofs[ids]]
            delta=np.linalg.solve(jac.T@jac+.0003*np.eye(3),jac.T@err)
            qref[ids]=d.qpos[self.pos[ids]]+delta
        # Joint-space regularization maintains the nominal wheel footprint.
        weights=np.full(m.nu,3.0);weights[self.legs]=300;weights[self.wheels]=0
        for a in np.flatnonzero(weights):
            row=np.zeros((1,n));row[0,self.dofs[a]]=1
            target=100*(qref[a]-d.qpos[self.pos[a]])-20*d.qvel[self.dofs[a]]
            objective(row,[target],weights[a])
        for i,a in enumerate(self.wheels):
            row=np.zeros((1,n));row[0,self.dofs[a]]=1
            speed=(v-w*self.offsets[i,1])/self.radius
            objective(row,[20*(speed-d.qvel[self.dofs[a]])],1)
        J=np.zeros((12,nv));active=[]
        for i,gid in enumerate(self.gids):
            body=int(m.geom_bodyid[gid]);center=d.geom_xpos[gid]
            point=center-np.array([0,0,self.radius])
            mj.mj_jac(m,d,jp,jr,point,body)
            mj.mj_jacDot(m,d,dp,dr,point,body)
            J[3*i:3*i+3]=jp
            # Rolling constraint uses a moving contact location, not a fixed material point.
            # Its Jacobian derivative adds omega x (p_dot_geometric - p_dot_material).
            omega_w=jr@d.qvel
            correction=np.cross(omega_w,np.cross(omega_w,[0,0,self.radius]))
            bias=dp@d.qvel+correction
            touching=center[2]-self.radius < .004
            active.append(touching)
            if touching:
                vel=jp@d.qvel
                target=-30*vel-bias
                target[2] -= 200*(center[2]-self.radius)
                # Soft lateral contact permits skid steering; normal/rolling are weighted higher.
                local=heading.T@jp;local_target=heading.T@target
                for axis,weight in enumerate((20,.2,100)):
                    objective(acc_rows(local[axis:axis+1]),[local_target[axis]],weight)
        M=np.zeros((nv,nv));mj.mj_fullM(m,M,d.qM)
        # Match the implicit velocity damping of the unchanged MuJoCo servos.
        M[np.diag_indices(nv)] += .01*m.dof_damping
        M[self.dofs,self.dofs] -= .01*m.actuator_biasprm[:,2]
        D=np.hstack((M,-J.T,-self.E.T))
        bias=d.qfrc_bias-d.qfrc_passive
        # External push is deliberately not included: it is an unknown disturbance.
        A=[D[self.unactuated]];lo=[-bias[self.unactuated]];hi=[-bias[self.unactuated]]
        if m.neq:
            eqerr=np.array([d.qpos[m.jnt_qposadr[m.eq_obj1id[i]]]-m.eq_data[i,0]-m.eq_data[i,1]*d.qpos[m.jnt_qposadr[m.eq_obj2id[i]]] for i in range(m.neq)])
            target=-100*eqerr-20*(self.E@d.qvel)
            A.append(acc_rows(self.E));lo.append(target);hi.append(target)
        servo_bias=m.actuator_biasprm[:,0]+m.actuator_biasprm[:,1]*d.actuator_length+m.actuator_biasprm[:,2]*d.actuator_velocity
        gain=m.actuator_gainprm[:,0]
        lower=np.maximum(self.force_lower,gain*m.actuator_ctrlrange[:,0]+servo_bias)
        upper=np.minimum(self.force_upper,gain*m.actuator_ctrlrange[:,1]+servo_bias)
        # Force clipping makes both feasible even when a servo's entire range is saturated.
        lower=np.clip(lower,self.force_lower,self.force_upper);upper=np.clip(upper,self.force_lower,self.force_upper)
        A.append(D[self.dofs]);lo.append(lower-bias[self.dofs]);hi.append(upper-bias[self.dofs])
        # Conservative square inside Coulomb cone, in world XY on the flat floor.
        for i,gid in enumerate(self.gids):
            mu=min(m.geom_friction[gid,0],m.geom_friction[m.geom('floor').id,0])/np.sqrt(2)
            F=np.zeros((5,n));k=nv+3*i
            F[0,k+2]=1
            for row,(axis,sign) in enumerate(((0,1),(0,-1),(1,1),(1,-1)),1):
                F[row,k+axis]=sign;F[row,k+2]=-mu
            A.append(F);lo.append(np.array([0,-np.inf,-np.inf,-np.inf,-np.inf]));hi.append(np.array([np.inf if active[i] else 0,0,0,0,0]))
        # Soft predictive joint-limit recovery, including passive finger joints.
        # Actual limit violations still fail the frozen evaluator.
        limited=np.flatnonzero(m.jnt_limited)
        B=np.zeros((len(limited),n));dt=.01
        for row,j in enumerate(limited):B[row,m.jnt_dofadr[j]]=.5*dt*dt
        q=d.qpos[m.jnt_qposadr[limited]];qd=d.qvel[m.jnt_dofadr[limited]]
        predicted=q+.05*qd
        unsafe=(predicted<m.jnt_range[limited,0])|(predicted>m.jnt_range[limited,1])
        if np.any(unsafe):
            recovery=100*(np.clip(predicted,m.jnt_range[limited,0],m.jnt_range[limited,1])-predicted)
            objective(B[unsafe]/(.5*dt*dt),recovery[unsafe],10)
        objective(np.pad(np.eye(12),((0,0),(nv,m.neq))),np.zeros(12),1e-5)
        matrix=np.vstack(A);lower_c=np.concatenate(lo);upper_c=np.concatenate(hi)
        scale=1/np.sqrt(np.maximum(np.diag(H),1e-6))
        scaled_A=matrix*scale
        row_scale=1/np.maximum(np.linalg.norm(scaled_A,axis=1),1e-8)
        scaled_A*=row_scale[:,None]
        sl=lower_c*row_scale;su=upper_c*row_scale
        eq=np.isfinite(sl)&np.isfinite(su)&(abs(sl-su)<1e-12)
        up=np.isfinite(su)&~eq;low=np.isfinite(sl)&~eq
        C=np.vstack((scaled_A[eq],scaled_A[up],-scaled_A[low]))
        rhs=np.r_[su[eq],su[up],-sl[low]]
        cones=[clarabel.ZeroConeT(int(eq.sum())),clarabel.NonnegativeConeT(int(up.sum()+low.sum()))]
        settings=clarabel.DefaultSettings();settings.verbose=False
        settings.max_iter=100
        solver=clarabel.DefaultSolver(sparse.csc_matrix(np.triu(H*scale[:,None]*scale[None,:])),
            g*scale,sparse.csc_matrix(C),rhs,cones,settings)
        result=solver.solve()
        if str(result.status) not in ('Solved','AlmostSolved'):
            self.stats=dict(status=str(result.status),seconds=time.perf_counter()-start)
            raise RuntimeError('WBC QP: '+str(result.status))
        x=np.array(result.x)*scale
        violation=float(max(np.max(lower_c-matrix@x),np.max(matrix@x-upper_c),0))
        if violation>.02 or not np.isfinite(x).all():
            self.stats=dict(status='residual_rejected',constraint_violation=violation,seconds=time.perf_counter()-start)
            raise RuntimeError('WBC constraint residual too large')
        tau=D[self.dofs]@x+bias[self.dofs]
        d.ctrl[:]=np.clip((tau-servo_bias)/gain,*m.actuator_ctrlrange.T)
        self.stats=dict(status=str(result.status),iterations=int(result.iterations),constraint_violation=violation,
                        seconds=time.perf_counter()-start,contact_forces=x[nv:nv+12].tolist(),
                        peak_force_util=float(np.max(abs(tau)/np.maximum(np.maximum(abs(self.force_lower),abs(self.force_upper)),1e-9))))
