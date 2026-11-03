"""Inspect robot model source ZIPs without redistributing third-party geometry."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile, BadZipFile

ARCHIVES = {
    "as2_w": {
        "sha256": "004a858cf96ab09ab5b04328c1e7ef962a95fff4b328756cf9382c494d1b3cf0",
        "mesh_prefix": "as2w_control/original/meshes/",
        "mesh_count": 36,
        "urdf": "as2w_control/original/as2w.urdf",
        "xml": "as2w_control/original/robot.xml",
    },
    "r1_a7": {
        "sha256": "fc70e54d5ee7d1308302ac4546911136e15d3f6ab574d5c2dc8b2a152c127693",
        "mesh_prefix": "R1_A7_MuJoCo/meshes/",
        "mesh_count": 19,
        "urdf": "R1_A7_MuJoCo/source/r1_a7.urdf",
        "xml": "R1_A7_MuJoCo/r1_a7.xml",
    },
}

def sha256_file(path: Path) -> str:
    hash_ = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            hash_.update(chunk)
    return hash_.hexdigest()

def inspect_archive(path: Path, spec: dict, require_exact_hash=False) -> dict:
    path = Path(path)
    actual = sha256_file(path)
    with ZipFile(path) as zf:
        names = zf.namelist()
        prefix = spec["mesh_prefix"]
        meshes = [
            name for name in names
            if name.startswith(prefix)
            and name.lower().endswith(".stl")
            and "/" not in name[len(prefix):]
        ]
        errors = []
        if len(meshes) != spec["mesh_count"]:
            errors.append(f"expected {spec['mesh_count']} meshes, found {len(meshes)}")
        if len(set(meshes)) != len(meshes):
            errors.append("duplicate mesh file paths")
        for required in (spec["urdf"], spec["xml"]):
            if required not in names:
                errors.append(f"missing {required}")
        if require_exact_hash and actual != spec["sha256"]:
            errors.append("ZIP SHA-256 differs from reference")
        return {
            "archive": str(path),
            "sha256": actual,
            "matches_reference_sha256": actual == spec["sha256"],
            "mesh_count": len(meshes),
            "required_files_present": all(
                item in names for item in (spec["urdf"], spec["xml"])
            ),
            "errors": errors,
        }

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--as2w-zip", type=Path, required=True)
    parser.add_argument("--r1a7-zip", type=Path, required=True)
    parser.add_argument("--require-exact-hash", action="store_true")
    args = parser.parse_args()
    results = {}
    try:
        for key, path in (
            ("as2_w", args.as2w_zip), ("r1_a7", args.r1a7_zip)
        ):
            results[key] = inspect_archive(
                path, ARCHIVES[key], args.require_exact_hash
            )
    except (OSError, BadZipFile) as exc:
        parser.exit(2, f"Cannot inspect ZIP: {exc}\n")
    print(json.dumps(results, ensure_ascii=False, indent=2))
    if any(item["errors"] for item in results.values()):
        raise SystemExit(1)

if __name__ == "__main__":
    main()
