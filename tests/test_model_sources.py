"""Static tests run even before original binary geometry is restored."""
from pathlib import Path
from xml.etree import ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PATHS = [
    ROOT / "models/as2_w/as2_w.xml",
    ROOT / "models/r1_a7/r1_a7.xml",
    ROOT / "models/centaur/centaur.xml",
]

def test_source_mjcf_are_well_formed():
    for path in PATHS:
        assert ET.parse(path).getroot().tag == "mujoco"

def test_unified_has_one_floating_base_and_33_actuators():
    xml = ET.parse(PATHS[2]).getroot()
    assert len(xml.findall(".//freejoint")) == 1
    actuators = xml.find("actuator")
    assert actuators is not None
    assert len(list(actuators)) == 33

def test_unified_references_55_shared_meshes():
    root = ET.parse(PATHS[2]).getroot()
    mesh_paths = [mesh.get("file") for mesh in root.findall(".//asset/mesh[@file]")]
    assert len(mesh_paths) == 55
    assert sum(path.startswith("../as2_w/meshes/") for path in mesh_paths) == 36
    assert sum(path.startswith("../r1_a7/meshes/") for path in mesh_paths) == 19
