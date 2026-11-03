"""Derived self-collision models; preserves user-confirmed source XML."""
from pathlib import Path
import copy,json
import mujoco
import xml.etree.ElementTree as E
R=Path(__file__).resolve().parents[1]
TARGETS={'as2w_base_link_collision_1':'base',
'as2w_FL_thigh_collision_1':'fl_thigh','as2w_FR_thigh_collision_1':'fr_thigh',
'r1a7_left_shoulder_yaw_link_collision0':'left_upper_arm',
'r1a7_right_shoulder_yaw_link_collision0':'right_upper_arm'}
def build():
 for source,out in [('centaur.xml','centaur_self_collision.xml'),('fixed_base.xml','fixed_base_self_collision.xml')]:
  r=E.parse(R/'source/user_confirmed'/source).getroot();r.set('model',r.get('model')+'_自碰撞')
  asset=r.find('asset')
  for body in r.iter('body'):
   for geom in list(body.findall('geom')):
    name=geom.get('name');key=TARGETS.get(name)
    if key:
     manifest=json.loads((R/f'meshes/collision_v1/{key}.json').read_text())
     body.remove(geom)
     for i,path in enumerate(manifest['parts']):
      meshname=f'collision_v1_{key}_{i}'
      E.SubElement(asset,'mesh',name=meshname,file=path)
      g=copy.deepcopy(geom);g.set('name',name+f'_convex{i}');g.set('mesh',meshname);body.append(g)
  for g in r.iter('geom'):
   if g.get('contype','1')!='0':g.set('contype','1');g.set('conaffinity','1')
  # Preserve original assembly pose as a named reference; open arms for home.
  cfg=json.loads((R/'collision_config.json').read_text())
  model=mujoco.MjModel.from_xml_path(str(R/source))
  key=r.find('keyframe/key');original=copy.deepcopy(key);original.set('name','assembly_reference')
  q=list(map(float,key.get('qpos').split()))
  for joint,value in cfg['home_joint_overrides_rad'].items():q[model.joint(joint).qposadr[0]]=value
  key.set('qpos',' '.join(format(v,'.16g') for v in q));r.find('keyframe').append(original)
  # Explicitly preserve root welded-group/child filtering in fixed-base models.
  # World-attached bodies are a special case in MuJoCo's parent filter.
  floating=mujoco.MjModel.from_xml_path(str(R/'centaur.xml'))
  base=floating.body('as2w_base_link').id
  group=[b for b in range(1,floating.nbody) if floating.body_weldid[b]==base]
  children=[b for b in range(1,floating.nbody) if floating.body_parentid[b] in group and b not in group]
  contact=E.SubElement(r,'contact')
  for a in group:
   for b in children:
    E.SubElement(contact,'exclude',body1=floating.body(a).name,body2=floating.body(b).name)
  cfg['manual_excludes']=[{'body1':floating.body(a).name,'body2':floating.body(b).name,'reason':'root welded group versus directly attached joint body'} for a in group for b in children]
  (R/'collision_config.json').write_text(json.dumps(cfg,ensure_ascii=False,indent=2))
  E.indent(r,space='  ');E.ElementTree(r).write(R/out,encoding='utf-8',xml_declaration=True)
if __name__=='__main__':build()
