"""Assemble AS2-W and R1-A7 into a single floating-base rigid-body tree."""
from pathlib import Path
import copy
import json
import xml.etree.ElementTree as E
import numpy as np
import mujoco

ROOT=Path(__file__).resolve().parents[1]

def fmt(v): return ' '.join(format(float(x),'.16g') for x in v)

def save(root,path):
    E.indent(root,space='  ')
    E.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)

def build():
    cfg=json.loads((ROOT/'assembly_config.json').read_text())
    # The user-confirmed installation is read only from assembly_config.json.
    for key, count in [('mount_position_in_as2w_base_m', 3), ('mount_quaternion_wxyz', 4)]:
        values = np.asarray(cfg[key], dtype=float)
        if values.shape != (count,) or not np.isfinite(values).all():
            raise ValueError(f'Invalid assembly parameter: {key}')
    if not np.isclose(np.linalg.norm(cfg['mount_quaternion_wxyz']), 1.0):
        raise ValueError('Mount quaternion must have unit length')
    lower=E.parse(ROOT/'source/AS2W.xml').getroot()
    upper=E.parse(ROOT/'source/r1_a7.xml').getroot()
    lower.set('model',cfg['model_name'])
    # Namespace all original AS2-W names and direct references before merging.
    names={x.get('name'):'as2w_'+x.get('name') for x in lower.iter() if x.get('name')}
    for x in lower.iter():
        for k in ['name','mesh','material','texture','joint','body','site','tendon']:
            if x.get(k) in names:x.set(k,names[x.get(k)])
    compiler=lower.find('compiler')
    compiler.attrib.clear()
    compiler.attrib.update(angle='radian',eulerseq='xyz',autolimits='true',inertiafromgeom='false',
                          fusestatic='false',balanceinertia='false',boundmass='0',boundinertia='0')
    opt=lower.find('option')
    opt.set('timestep',str(cfg['timestep_s']));opt.set('density','0');opt.set('viscosity','0')
    opt.set('iterations','80');opt.set('cone','elliptic')
    E.SubElement(lower,'size',nuser_jnt='1')
    vis=lower.find('visual');glob=vis.find('global')
    glob.set('offwidth','1920');glob.set('offheight','1080')
    vis.find('quality').set('shadowsize','2048')
    assets=lower.find('asset')
    for x in assets.findall('mesh'):
        x.set('file','meshes/as2w/'+Path(x.get('file')).name)
    # Use one common scene background; preserve all robot-specific assets.
    for x in list(assets):
        if x.tag=='texture' and x.get('type')=='skybox':assets.remove(x)
    E.SubElement(assets,'texture',name='centaur_sky',type='skybox',builtin='gradient',
                 rgb1='0.91 0.94 0.98',rgb2='0.98 0.99 1',width='512',height='3072')
    for x in upper.find('asset'):
        x=copy.deepcopy(x)
        if x.tag=='mesh': x.set('file','meshes/r1a7/'+Path(x.get('file')).name)
        assets.append(x)
    world=lower.find('worldbody')
    floor=world.find('geom');floor.set('pos','0 0 0');floor.set('contype','2');floor.set('conaffinity','1')
    floor.set('friction','0.8 0.002 0.0001');floor.set('solref','0.008 1')
    floor.set('rgba','0.86 0.88 0.91 1');floor.attrib.pop('material',None)
    base=world.find('body');base.set('pos',fmt([0,0,cfg['home_base_height_m']]))
    pelvis=copy.deepcopy(upper.find('worldbody/body'))
    pelvis.set('pos',fmt(cfg['mount_position_in_as2w_base_m']))
    pelvis.set('quat',fmt(cfg['mount_quaternion_wxyz']))
    base.append(pelvis)  # Weld by tree parentage, not a second floating robot.
    E.SubElement(base,'site',name='centaur_imu',pos='0 0 0',size='0.006',rgba='0 0.8 0 1')
    for g in base.findall('.//geom'):
        if 'visual' in g.get('name',''):
            g.set('contype','0');g.set('conaffinity','0');g.set('group','2')
            if g.get('mesh','').startswith('as2w_'):g.set('rgba','0.28 0.34 0.43 1')
        else:
            g.set('contype','1');g.set('conaffinity','2');g.set('group','3')
            g.set('rgba','0.4 0.6 0.8 0');g.set('friction','0.8 0.002 0.0001')
            g.set('solref','0.008 1')
        g.set('mass','0')
    if cfg['self_collision_enabled']:
        raise ValueError('Self-collision requires validated collision pairs; not configured in this assembly baseline.')
    lower.remove(lower.find('actuator'))
    act=E.SubElement(lower,'actuator')
    mapping=[]
    for j in base.findall('.//joint'):
        if not j.get('name','').startswith('as2w_'):continue
        lo,hi=map(float,j.get('actuatorfrcrange').split())
        aname=j.get('name')+'_motor'
        E.SubElement(act,'motor',name=aname,joint=j.get('name'),gear='1',
                     ctrlrange=fmt([lo,hi]),forcerange=fmt([lo,hi]),ctrllimited='true',forcelimited='true')
        # No identified physical damping or reflected rotor inertia in the source.
        j.set('damping','0');j.set('armature','0');j.set('frictionloss','0')
        mapping.append({'joint':j.get('name'),'actuator':aname,'unit':'N*m','torque_limits':[lo,hi]})
    for x in upper.find('actuator'):
        act.append(copy.deepcopy(x))
        mapping.append({'joint':x.get('joint'),'actuator':x.get('name'),'unit':'N*m',
                        'torque_limits':list(map(float,x.get('ctrlrange').split()))})
    sensor=E.SubElement(lower,'sensor')
    E.SubElement(sensor,'framequat',name='centaur_base_quat',objtype='body',objname='as2w_base_link')
    E.SubElement(sensor,'gyro',name='centaur_gyro',site='centaur_imu')
    E.SubElement(sensor,'accelerometer',name='centaur_accelerometer',site='centaur_imu')
    for x in mapping:
        E.SubElement(sensor,'jointpos',name=x['joint']+'_position',joint=x['joint'])
        E.SubElement(sensor,'jointvel',name=x['joint']+'_velocity',joint=x['joint'])
        E.SubElement(sensor,'actuatorfrc',name=x['actuator']+'_torque',actuator=x['actuator'])
    for key in lower.findall('keyframe'):lower.remove(key)
    E.SubElement(world,'light',pos='1 -1 2',dir='-0.3 0.2 -1',directional='true',diffuse='0.7 0.7 0.7')
    # First compile to obtain exact joint addresses, then construct a valid home.
    output=ROOT/'centaur.xml'
    save(lower,output)
    m=mujoco.MjModel.from_xml_path(str(output));q=m.qpos0.copy()
    for leg in ['FL','FR','RL','RR']:
        for suffix,value in zip(['hip','thigh','calf'],cfg['leg_home_angles_rad']):
            q[m.joint('as2w_'+leg+'_'+suffix).qposadr[0]]=value
        # Keep the wheel mesh frame aligned with world/body for a useful home pose.
        q[m.joint('as2w_'+leg+'_foot').qposadr[0]]=-sum(cfg['leg_home_angles_rad'][1:])
    keys=E.SubElement(lower,'keyframe')
    E.SubElement(keys,'key',name='home',qpos=fmt(q),ctrl=fmt(np.zeros(m.nu)))
    save(lower,output)
    # Identical geometry/dynamics with a welded base for assembly inspection.
    fixed=copy.deepcopy(lower);fixed.set('model',cfg['model_name']+'_固定基座检查')
    fb=fixed.find('worldbody/body');fb.remove(fb.find('freejoint'))
    key=fixed.find('keyframe/key');key.set('qpos',fmt(q[7:]))
    save(fixed,ROOT/'fixed_base.xml')
    for i,x in enumerate(mapping):x['index']=i
    (ROOT/'actuator_map.json').write_text(json.dumps(mapping,ensure_ascii=False,indent=2))
    print(json.dumps({'model':cfg['model_name'],'nq':m.nq,'nv':m.nv,'nu':m.nu,
                      'mass_kg':float(m.body_mass.sum()),'body_count':m.nbody-1},ensure_ascii=False))

if __name__=='__main__':build()
