"""Report missing original meshes and URDFs; optionally compile MJCF."""
import argparse
from pathlib import Path
from xml.etree import ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MODELS = {
    "as2_w": ROOT / "models/as2_w/as2_w.xml",
    "r1_a7": ROOT / "models/r1_a7/r1_a7.xml",
    "centaur": ROOT / "models/centaur/centaur.xml",
}

def inspect_meshes(xml: Path):
    tree = ET.parse(xml)
    compiler = tree.getroot().find("compiler")
    meshdir = "" if compiler is None else compiler.get("meshdir", "")
    missing = []
    for mesh in tree.findall(".//mesh[@file]"):
        path = (xml.parent / meshdir / mesh.get("file")).resolve()
        if not path.is_file():
            missing.append(str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path))
    return missing

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--strict", action="store_true", help="fail on missing model assets")
    parser.add_argument("--compile", action="store_true", help="compile all models when assets exist")
    args = parser.parse_args()
    problems = []
    for name, path in MODELS.items():
        if not path.is_file():
            problems.append(f"Missing XML: {path}")
            continue
        meshes = inspect_meshes(path)
        problems.extend(f"{name}: missing mesh {m}" for m in meshes)
        print(f"{name}: XML parsed; {len(meshes)} missing mesh references")
        if args.compile and not meshes:
            import mujoco
            model = mujoco.MjModel.from_xml_path(str(path))
            print(f"  compiled: nq={model.nq}, nv={model.nv}, nu={model.nu}")
    for subdir, name in [("as2_w","as2_w.urdf"),("r1_a7","r1_a7.urdf")]:
        if not (ROOT / "models" / subdir / name).is_file():
            problems.append(f"Missing source URDF: {subdir}/{name}")
    for item in problems[:15]:
        print("MISSING:", item)
    if len(problems) > 15:
        print(f"... and {len(problems)-15} other missing paths")
    if problems:
        print("Model execution and physical/control verification NOT established.")
        if args.strict:
            raise SystemExit(1)
    else:
        print("Asset presence check complete; this does not verify stable robot control.")

if __name__ == "__main__":
    main()
