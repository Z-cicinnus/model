"""Static verification of reconstructed reference models and mesh paths."""
from __future__ import annotations
import argparse
from pathlib import Path
from xml.etree import ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
REF = ROOT / "research/centaur_complete"
O6 = ROOT / "models/r1_a7_o6"
MODELS = [
    REF / "centaur.xml",
    REF / "centaur_self_collision.xml",
    REF / "fixed_base.xml",
    REF / "fixed_base_self_collision.xml",
    O6 / "r1_a7_o6.xml",
    O6 / "scene_o6.xml",
    O6 / "hand/linkerhand_o6_left.xml",
    O6 / "hand/linkerhand_o6_right.xml",
]


def inspect_model(path: Path) -> dict:
    tree = ET.parse(path)
    root = tree.getroot()
    compiler = root.find("compiler")
    meshdir = compiler.get("meshdir", "") if compiler is not None else ""
    missing = []
    mesh_refs = root.findall(".//mesh[@file]")
    for mesh in mesh_refs:
        f = mesh.get("file")
        dest = (path.parent / meshdir / f).resolve()
        if not dest.exists():
            missing.append(f)
    for incl in root.findall(".//include[@file]"):
        f = incl.get("file")
        if not (path.parent / f).is_file():
            missing.append(f"include:{f}")
    return {"path": str(path.relative_to(ROOT)), "meshes": len(mesh_refs), "missing": missing}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--strict", action="store_true", help="Fail if any required geometry is missing")
    p.add_argument("--compile", action="store_true", help="Compile models (requires MuJoCo)")
    args = p.parse_args()
    incomplete = False
    for xml in MODELS:
        try:
            info = inspect_model(xml)
        except (OSError, ET.ParseError) as exc:
            print(f"ERROR {xml}: {exc}")
            incomplete = True
            continue
        missing = info["missing"]
        incomplete = incomplete or bool(missing)
        print(f"{info['path']}: {info['meshes']} meshes; missing={len(missing)}")
        if missing:
            print("  examples:", ", ".join(missing[:3]))
        if args.compile and not missing:
            import mujoco
            model = mujoco.MjModel.from_xml_path(str(xml.resolve()))
            print(f"  loaded: nq={model.nq}, nv={model.nv}, nu={model.nu}")
    if incomplete and args.strict:
        raise SystemExit(1)

if __name__ == "__main__":
    main()
