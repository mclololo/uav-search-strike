"""Export Git files and real LFS heads into a new cloud workspace archive.

The default exports HEAD. --index exports the staged tree for pre-commit checks.
No untracked files, base model weights, YOLO weights or training data are included.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile


ROOT = Path(__file__).resolve().parents[1]
LFS_HEADER = b"version https://git-lfs.github.com/spec/v1\n"


def git(*arguments):
    return subprocess.check_output(["git", *arguments], cwd=ROOT)


def package(output, use_index=False):
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite: {output}")
    revision = git("write-tree").decode().strip() if use_index else "HEAD"
    archive = git("archive", "--format=tar", revision)
    output.parent.mkdir(parents=True, exist_ok=True)
    heads = []
    files = 0
    with tarfile.open(fileobj=io.BytesIO(archive), mode="r:") as source:
        with tarfile.open(output, mode="x:gz") as target:
            for member in source:
                path = Path(member.name)
                if path.is_absolute() or ".." in path.parts:
                    raise ValueError(f"Unsafe archive path: {member.name}")
                if member.name.startswith("src/object_det/models/") or "FM9G4B-V" in path.parts:
                    raise ValueError(f"External model found in Git: {member.name}")
                content = source.extractfile(member).read() if member.isfile() else None
                is_head = member.isfile() and member.name.startswith(
                    "tools/fm9g_grounding/deployment/") and path.suffix == ".safetensors"
                if is_head:
                    # Some Git versions already expand LFS in archives. Check the
                    # committed pointer in either case, not only archive content.
                    blob = git("show", f"{revision}:{member.name}")
                    if not blob.startswith(LFS_HEADER):
                        raise ValueError(f"Head is not managed by Git LFS: {member.name}")
                    pointer = dict(line.split(" ", 1) for line in blob.decode().splitlines())
                    if content.startswith(LFS_HEADER):
                        content = (ROOT / member.name).read_bytes()
                    digest = hashlib.sha256(content).hexdigest()
                    if len(content) != int(pointer["size"]) or "sha256:" + digest != pointer["oid"]:
                        raise ValueError(f"Missing or invalid LFS file; run git lfs pull: {member.name}")
                    member.size = len(content)
                    heads.append({"path": member.name, "sha256": digest, "bytes": len(content)})
                elif content is not None and content.startswith(LFS_HEADER):
                    raise ValueError(f"Unexpected LFS file: {member.name}")
                if member.isfile():
                    files += 1
                    # Windows checkouts can lose execute bits on ROS entrypoints.
                    if path.suffix in {".py", ".sh"}:
                        member.mode = 0o755
                target.addfile(member, io.BytesIO(content) if content is not None else None)
    result = {
        "revision": revision if use_index else git("rev-parse", "HEAD").decode().strip(),
        "files": files,
        "heads": heads,
        "archive": str(output),
        "archive_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
        "archive_bytes": output.stat().st_size,
    }
    print(json.dumps(result, indent=2))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--index", action="store_true", help="Export staged tree for pre-commit checks")
    args = parser.parse_args()
    package(args.output, args.index)


if __name__ == "__main__":
    main()
