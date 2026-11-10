"""Reassemble exact historical ZIP files stored as binary Git shards.

Uses only Python's standard library and validates every chunk and full archive.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
MANIFEST=ROOT/'archives'/'manifest.json'

def reconstruct(version: str, output_dir: Path, *, overwrite: bool=False) -> Path:
    manifest=json.loads(MANIFEST.read_text(encoding='utf-8'))
    archives=manifest['archives']
    if version not in archives:
        raise ValueError(f'Unknown version {version!r}. Choose from {list(archives)}')
    record=archives[version]
    output_dir.mkdir(parents=True,exist_ok=True)
    target=output_dir/record['filename']
    if target.exists() and not overwrite:
        raise FileExistsError(f'{target} already exists; pass --overwrite explicitly')
    tmp=output_dir/(record['filename']+'.part')
    if tmp.exists():
        raise FileExistsError(f'Incomplete output present: {tmp}; remove manually after inspection')
    sha=hashlib.sha256()
    count=0
    try:
        with tmp.open('xb') as output:
            for part in record['parts']:
                relative=Path(part['path'])
                if relative.is_absolute() or '..' in relative.parts:
                    raise ValueError(f'Unsafe manifest path: {relative}')
                path=ROOT/relative
                if not path.is_file():
                    raise FileNotFoundError(f'Missing Git archive shard: {path}')
                data=path.read_bytes()
                if len(data)!=part['size']:
                    raise ValueError(f'Wrong shard size: {path}')
                if hashlib.sha256(data).hexdigest()!=part['sha256']:
                    raise ValueError(f'Shard SHA-256 mismatch: {path}')
                sha.update(data)
                count+=len(data)
                output.write(data)
        if count!=record['size'] or sha.hexdigest()!=record['sha256']:
            raise ValueError(f'ZIP size or SHA-256 mismatch: {version}')
        tmp.replace(target)
    except Exception:
        if tmp.exists():
            tmp.unlink()
        raise
    print(f'Verified {target}: {count:,} bytes SHA256={sha.hexdigest()}')
    return target

def main():
    p=argparse.ArgumentParser(description=__doc__)
    group=p.add_mutually_exclusive_group(required=True)
    group.add_argument('--all',action='store_true')
    group.add_argument('--version', choices=sorted(json.loads(MANIFEST.read_text(encoding='utf-8'))['archives']))
    p.add_argument('--output-dir',type=Path,default=Path('update_packages'))
    p.add_argument('--overwrite',action='store_true')
    args=p.parse_args()
    archives=json.loads(MANIFEST.read_text(encoding='utf-8'))['archives']
    versions=list(archives) if args.all else [args.version]
    for version in versions:
        reconstruct(version, args.output_dir,overwrite=args.overwrite)

if __name__=='__main__':
    main()
