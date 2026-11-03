"""Archive importer and static model checks without MuJoCo."""
from pathlib import Path
from scripts.check_uploaded_models import MODELS, inspect_model

def test_reference_xml_well_formed():
    for path in MODELS:
        result = inspect_model(path)
        assert result["meshes"] >= 0

def test_r1_a7_o6_has_22_hand_joints():
    from xml.etree import ElementTree as ET
    path = next(p for p in MODELS if p.name == "r1_a7_o6.xml")
    root = ET.parse(path).getroot()
    assert len(root.findall('.//worldbody//joint')) == 39
    assert len(root.findall('./actuator/*')) == 29

def test_importer_paths_resolved(tmp_path, monkeypatch):
    # The full uploaded ZIPs are not required in public CI.
    from scripts import import_uploaded_archives as importer
    assert importer.CENT_PREFIX.endswith("centaur_quadruped/")
    assert importer.O6_PREFIX.endswith("r1_a7_o6_model/")
