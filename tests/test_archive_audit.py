"""Offline source archive audit tests, independent of MuJoCo and source meshes."""
import hashlib
from pathlib import Path
from zipfile import ZipFile
from scripts.check_source_packages import inspect_archive, sha256_file

def test_source_zip_auditor(tmp_path: Path):
    path = tmp_path / "robot.zip"
    with ZipFile(path, "w") as archive:
        archive.writestr("robot/meshes/part_1.stl", b"mesh sample")
        archive.writestr("robot/model.xml", "<mujoco/>")
        archive.writestr("robot/model.urdf", "<robot/>")
    spec = {
        "mesh_prefix": "robot/meshes/",
        "mesh_count": 1,
        "urdf": "robot/model.urdf",
        "xml": "robot/model.xml",
        "sha256": sha256_file(path),
    }
    good = inspect_archive(path, spec, require_exact_hash=True)
    assert good["errors"] == []
    assert good["matches_reference_sha256"]
    bad = inspect_archive(path, {**spec, "mesh_count": 2})
    assert bad["errors"]
    assert sha256_file(path) == hashlib.sha256(path.read_bytes()).hexdigest()
