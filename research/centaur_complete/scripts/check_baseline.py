from pathlib import Path
import json
import hashlib
import xml.etree.ElementTree as E
import mujoco

ROOT = Path(__file__).resolve().parents[1]

def signature(e):
    return (e.tag, sorted(e.attrib.items()), (e.text or '').strip(), [signature(c) for c in e])

cfg = json.loads((ROOT/'assembly_config.json').read_text())
report = {'stage': '1_model_baseline', 'mujoco_version': mujoco.__version__, 'models': {}}
for name, free_count in [('centaur.xml', 1), ('fixed_base.xml', 0)]:
    path = ROOT/name
    root = E.parse(path).getroot()
    assert signature(root) == signature(E.parse(ROOT/'source/user_confirmed'/name).getroot()), name
    pelvis = next(b for b in root.iter('body') if b.get('name') == 'r1a7_pelvis_link')
    assert list(map(float, pelvis.get('pos').split())) == cfg['mount_position_in_as2w_base_m']
    assert list(map(float, pelvis.get('quat').split())) == cfg['mount_quaternion_wxyz']
    assert len(list(root.iter('freejoint'))) == free_count
    model = mujoco.MjModel.from_xml_path(str(path))
    assert model.nu == 33
    report['models'][name] = {'matches_user_xml': True, 'compiles': True, 'freejoints': free_count, 'actuators': model.nu, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
report['status'] = 'passed'
(ROOT/'baseline_check.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
print(json.dumps(report, ensure_ascii=False, indent=2))
