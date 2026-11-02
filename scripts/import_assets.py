"""Restore user-provided AS2-W and R1-A7 meshes and URDFs from local ZIP files.

Does not download third-party geometry or commit it to GitHub.
"""
import argparse
from pathlib import Path
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = {
    "as2w": {
        "mesh_prefix": "as2w_control/original/meshes/",
        "urdf_member": "as2w_control/original/as2w.urdf",
        "dest": ROOT / "models/as2_w",
        "urdf_name": "as2_w.urdf",
        "count": 36,
    },
    "r1a7": {
        "mesh_prefix": "R1_A7_MuJoCo/meshes/",
        "urdf_member": "R1_A7_MuJoCo/source/r1_a7.urdf",
        "dest": ROOT / "models/r1_a7",
        "urdf_name": "r1_a7.urdf",
        "count": 19,
    },
}

def import_archive(path: Path, key: str) -> None:
    spec = SOURCE[key]
    with ZipFile(path) as zf:
        members = [
            name for name in zf.namelist()
            if name.startswith(spec["mesh_prefix"])
            and name.lower().endswith(".stl")
            and "/" not in name[len(spec["mesh_prefix"]):]
        ]
        if len(members) != spec["count"]:
            raise ValueError(f"{key}: expected {spec['count']} meshes, found {len(members)}")
        names = [Path(name).name for name in members]
        if len(names) != len(set(names)):
            raise ValueError(f"{key}: duplicate mesh filenames")
        if spec["urdf_member"] not in zf.namelist():
            raise ValueError(f"{key}: source URDF not found")
        dest = spec["dest"]
        mesh_dir = dest / "meshes"
        mesh_dir.mkdir(parents=True, exist_ok=True)
        for name in members:
            (mesh_dir / Path(name).name).write_bytes(zf.read(name))
        (dest / spec["urdf_name"]).write_bytes(zf.read(spec["urdf_member"]))
        print(f"{key}: restored {len(members)} source STL files + URDF")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--as2w-zip", type=Path, required=True)
    parser.add_argument("--r1a7-zip", type=Path, required=True)
    args = parser.parse_args()
    import_archive(args.as2w_zip, "as2w")
    import_archive(args.r1a7_zip, "r1a7")

if __name__ == "__main__":
    main()
