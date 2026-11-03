"""Restore geometry assets from the user's uploaded Centaur and R1-A7 O6 ZIPs.

No network downloads, no implicit updates to repository-tracked XMLs, no
publication of original vendor mesh files. Extract only expected data files.
"""
from __future__ import annotations
import argparse
from pathlib import Path, PurePosixPath
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parents[1]
CENT_PREFIX = "Centaur_MuJoCo_Complete/centaur_quadruped/"
O6_PREFIX = "r1_a7_o6_model/"


def _copy(zf: ZipFile, member: str, dest: Path):
    info = zf.getinfo(member)
    if info.is_dir() or info.file_size > 50_000_000:
        raise ValueError(f"Invalid mesh asset: {member}")
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_bytes(zf.read(info))


def import_centaur(zip_path: Path) -> dict:
    out = {"centaur_meshes": 0, "as2_w_stl": 0, "r1_a7_stl": 0, "urdf": 0, "run_artifacts": 0}
    with ZipFile(zip_path) as archive:
        for member in archive.namelist():
            if not member.startswith(CENT_PREFIX):
                continue
            relative = member[len(CENT_PREFIX):]
            parts = PurePosixPath(relative).parts
            if not parts or any(part in ("..", "") for part in parts):
                continue
            if parts[0] == "meshes" and len(parts) >= 3:
                if Path(relative).suffix.lower() not in (".stl", ".obj", ".json"):
                    continue
                dest = ROOT / "research/centaur_complete" / relative
                _copy(archive, member, dest)
                out["centaur_meshes"] += 1
                if len(parts) == 3 and parts[1] == "as2w" and relative.lower().endswith(".stl"):
                    _copy(archive, member, ROOT / "models/as2_w/meshes" / parts[-1])
                    out["as2_w_stl"] += 1
                if len(parts) == 3 and parts[1] == "r1a7" and relative.lower().endswith(".stl"):
                    _copy(archive, member, ROOT / "models/r1_a7/meshes" / parts[-1])
                    out["r1_a7_stl"] += 1
            elif Path(relative).suffix.lower() in (".npz", ".mp4"):
                # Preserve replay data locally; never commit large artifacts to Git.
                _copy(archive, member, ROOT / "research/centaur_complete" / relative)
                out["run_artifacts"] += 1
            elif relative == "source/r1_a7.urdf":
                _copy(archive, member, ROOT / "research/centaur_complete/source/r1_a7.urdf")
                _copy(archive, member, ROOT / "models/r1_a7/r1_a7.urdf")
                out["urdf"] += 1
    if out["as2_w_stl"] != 36 or out["r1_a7_stl"] != 19:
        raise ValueError(f"Unexpected mesh inventory: {out}")
    return out


def import_r1_a7_o6(zip_path: Path) -> dict:
    out = {"r1_a7_body_stl": 0, "o6_hand_obj": 0}
    with ZipFile(zip_path) as archive:
        for member in archive.namelist():
            if not member.startswith(O6_PREFIX):
                continue
            relative = member[len(O6_PREFIX):]
            parts = PurePosixPath(relative).parts
            if not parts or any(part == ".." for part in parts):
                continue
            if len(parts) == 2 and parts[0] == "meshes" and relative.lower().endswith(".stl"):
                _copy(archive, member, ROOT / "models/r1_a7_o6" / relative)
                out["r1_a7_body_stl"] += 1
            elif len(parts) == 4 and parts[0:2] == ("hand", "meshes") and relative.lower().endswith(".obj"):
                _copy(archive, member, ROOT / "models/r1_a7_o6" / relative)
                out["o6_hand_obj"] += 1
    if out != {"r1_a7_body_stl": 18, "o6_hand_obj": 48}:
        raise ValueError(f"Unexpected O6 asset inventory: {out}")
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--centaur-zip", required=True, type=Path)
    parser.add_argument("--o6-zip", required=True, type=Path)
    args = parser.parse_args()
    print("Centaur assets:", import_centaur(args.centaur_zip))
    print("R1-A7 O6 assets:", import_r1_a7_o6(args.o6_zip))
    print("Mesh files are kept outside Git tracking; see docs/uploaded_archives_zh.md")

if __name__ == "__main__":
    main()
