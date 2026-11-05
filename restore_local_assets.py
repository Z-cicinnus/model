"""Restore binary assets from the user's original Wheel_Centaur.zip.

Do not overwrite existing source files; only place excluded binary assets.
"""
from __future__ import annotations
import argparse
from pathlib import Path
from zipfile import ZipFile

BINARY_EXTENSIONS = {".stl", ".obj", ".npz", ".png"}
ROOT = Path(__file__).resolve().parent

def restore(path: Path):
    count = 0
    with ZipFile(path) as zf:
        for member in zf.infolist():
            if member.is_dir() or not member.filename.startswith("Wheel_Centaur/"):
                continue
            name = member.filename[len("Wheel_Centaur/"):]
            if not name or Path(name).suffix.lower() not in BINARY_EXTENSIONS:
                continue
            relative = Path(name)
            if relative.is_absolute() or '..' in relative.parts:
                raise ValueError(f"Unsafe archive member: {name}")
            dest = ROOT / relative
            dest.parent.mkdir(parents=True, exist_ok=True)
            if not dest.is_file():
                dest.write_bytes(zf.read(member))
                count += 1
    print(f"Restored {count} missing binary assets")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--zip', type=Path, required=True)
    args = parser.parse_args()
    restore(args.zip)
