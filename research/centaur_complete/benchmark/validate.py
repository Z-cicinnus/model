"""Interface checks independent of controller task success."""
from pathlib import Path
import json
import numpy as np
from env import CentaurEnv

def trace():
 e=CentaurEnv('hold');o,_=e.reset(42);assert o.shape==(101,)
 samples=[]
 for _ in range(10):
  o,r,term,trunc,info=e.step(np.zeros(33));samples.append(np.r_[o,r]);assert not term and not trunc
 return np.array(samples)
a=trace();b=trace();assert np.array_equal(a,b)
e=CentaurEnv('hold')
try:e.step(np.full(33,np.nan));raise AssertionError('NaN accepted')
except ValueError:pass
try:e.step(np.zeros(32));raise AssertionError('Wrong shape accepted')
except ValueError:pass
e.step(np.full(33,1e6));assert np.all(e.d.ctrl<=e.high) and e.clipped==1
for name in e.config['tasks']:
 t=CentaurEnv(name);bp,br,hp,hr=t.targets(0);assert np.allclose(bp,t.q0[:3]) and np.allclose(hp,t.hand0)
 bp,br,hp,hr=t.targets(12);assert np.isclose(bp[0]-t.q0[0],t.task['translation_m'])
out=Path(__file__).parent/'results';out.mkdir(exist_ok=True)
report={'passed':True,'determinism':'two resets, 10 identical zero-action steps, bitwise identical observations and rewards','observation_size':101,'action_size':33,'invalid_action_rejected':True,'action_clipping_verified':True,'task_start_and_terminal_translation_checked':True}
(out/'interface_validation.json').write_text(json.dumps(report,indent=2));print(report)
