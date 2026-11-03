from pathlib import Path
import trimesh,coacd,numpy as np,time,json
from concurrent.futures import ProcessPoolExecutor
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'meshes/collision_v1';OUT.mkdir(exist_ok=True)
def work(item):
 name,file,scale=item;t=time.time();mesh=trimesh.load(ROOT/'meshes'/file,force='mesh');mesh.vertices*=scale
 coacd.set_log_level('warn')
 parts=coacd.run_coacd(coacd.Mesh(mesh.vertices,mesh.faces),threshold=.025,max_convex_hull=24,preprocess_resolution=60,resolution=1000,mcts_nodes=10,mcts_iterations=40,mcts_max_depth=3,decimate=True,max_ch_vertex=128,seed=42)
 files=[]
 for i,(v,f) in enumerate(parts):
  path=OUT/f'{name}_{i}.obj';trimesh.Trimesh(v,f,process=False).export(path);files.append(str(path.relative_to(ROOT)))
 result={'name':name,'parts':files,'seconds':time.time()-t};(OUT/f'{name}.json').write_text(json.dumps(result));print(result,flush=True);return result
if __name__=='__main__':
 with ProcessPoolExecutor(max_workers=3) as e:list(e.map(work,[('base','as2w/part_2.stl',.001),('fl_thigh','as2w/part_6.stl',.001),('fr_thigh','as2w/part_14.stl',.001),('left_upper_arm','r1a7/left_shoulder_yaw_link.STL',1),('right_upper_arm','r1a7/right_shoulder_yaw_link.STL',1)]))
