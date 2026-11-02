"""Real model loading is skipped without source geometry; never report it as passed."""
import pytest
from scripts.check_repo import MODELS, inspect_meshes

def test_integrated_mujoco_compilation():
    xml = MODELS["centaur"]
    if inspect_meshes(xml):
        pytest.skip("Original STL assets absent; integrated model not compiled")
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(xml))
    assert model.nv > 6
    assert model.nu == 33
