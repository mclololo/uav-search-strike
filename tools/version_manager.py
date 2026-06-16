#!/usr/bin/env python3
"""
Lightweight snapshot-based version manager for the PX4/ROS workspace.
"""
from __future__ import annotations

import argparse
import json
import shutil
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable


WORKSPACE = Path("/home/ubuntu/catkin_ws")
STORE_DIR = WORKSPACE / ".versions"
MANIFEST = STORE_DIR / "manifest.json"


TRACKED_FILES = [
    Path("/home/ubuntu/model/mission_once.py"),
    Path("/home/ubuntu/model/prompts/mission_prompt.txt"),
    WORKSPACE / "src/mission_pkg/src/bt_ros.cpp",
    WORKSPACE / "src/mission_pkg/launch/llm_mission.launch",
    WORKSPACE / "src/mission_pkg/src/nodes/action/visual_servo.cpp",
    WORKSPACE / "src/mission_pkg/include/nodes/action/visual_servo.h",
    WORKSPACE / "src/mission_pkg/src/nodes/perception/target_tracker.cpp",
    WORKSPACE / "src/mission_pkg/include/nodes/perception/target_tracker.h",
]


@dataclass
class Snapshot:
    snapshot_id: str
    label: str
    created_at: str
    files: list[str]


def ensure_store() -> None:
    STORE_DIR.mkdir(parents=True, exist_ok=True)
    if not MANIFEST.exists():
        MANIFEST.write_text("[]", encoding="utf-8")


def load_manifest() -> list[dict]:
    ensure_store()
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


def save_manifest(data: list[dict]) -> None:
    MANIFEST.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def normalize_label(label: str) -> str:
    safe = "".join(ch if ch.isalnum() or ch in ("-", "_") else "-" for ch in label.strip())
    safe = safe.strip("-_")
    return safe or "snapshot"


def tracked_existing_files() -> list[Path]:
    return [path for path in TRACKED_FILES if path.exists()]


def relative_storage_path(path: Path) -> Path:
    return Path(path.as_posix().lstrip("/"))


def create_snapshot(label: str) -> Snapshot:
    ensure_store()
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    snapshot_id = f"{timestamp}-{normalize_label(label)}"
    snapshot_dir = STORE_DIR / snapshot_id
    snapshot_dir.mkdir(parents=True, exist_ok=False)

    files = tracked_existing_files()
    if not files:
        raise SystemExit("No tracked files found; nothing to snapshot.")

    for source in files:
        target = snapshot_dir / relative_storage_path(source)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)

    entry = Snapshot(
        snapshot_id=snapshot_id,
        label=label,
        created_at=datetime.now().isoformat(timespec="seconds"),
        files=[str(path) for path in files],
    )
    manifest = load_manifest()
    manifest.append(entry.__dict__)
    save_manifest(manifest)
    return entry


def list_snapshots() -> Iterable[Snapshot]:
    for item in load_manifest():
        yield Snapshot(**item)


def find_snapshot(snapshot_id: str) -> Snapshot:
    for snap in list_snapshots():
        if snap.snapshot_id == snapshot_id:
            return snap
    raise SystemExit(f"Snapshot not found: {snapshot_id}")


def restore_snapshot(snapshot_id: str) -> Snapshot:
    snapshot = find_snapshot(snapshot_id)
    snapshot_dir = STORE_DIR / snapshot.snapshot_id
    if not snapshot_dir.exists():
        raise SystemExit(f"Snapshot directory missing: {snapshot_dir}")

    for file_name in snapshot.files:
        source = snapshot_dir / relative_storage_path(Path(file_name))
        target = Path(file_name)
        if not source.exists():
            raise SystemExit(f"Missing file in snapshot: {source}")
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)

    return snapshot


def show_snapshot(snapshot_id: str) -> Snapshot:
    return find_snapshot(snapshot_id)


def print_snapshot(snapshot: Snapshot, verbose: bool = False) -> None:
    print(f"id: {snapshot.snapshot_id}")
    print(f"label: {snapshot.label}")
    print(f"created_at: {snapshot.created_at}")
    print(f"file_count: {len(snapshot.files)}")
    if verbose:
        print("files:")
        for file_name in snapshot.files:
            print(f"  {file_name}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Workspace snapshot version manager")
    sub = parser.add_subparsers(dest="command", required=True)

    snap = sub.add_parser("snapshot", help="Create a new snapshot")
    snap.add_argument("label", nargs="?", default="manual", help="Snapshot label")

    sub.add_parser("list", help="List snapshots")

    show = sub.add_parser("show", help="Show snapshot details")
    show.add_argument("snapshot_id", help="Snapshot id")

    restore = sub.add_parser("restore", help="Restore a snapshot")
    restore.add_argument("snapshot_id", help="Snapshot id")

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    if args.command == "snapshot":
        snapshot = create_snapshot(args.label)
        print("Snapshot created.")
        print_snapshot(snapshot)
        return

    if args.command == "list":
        snapshots = list(list_snapshots())
        if not snapshots:
            print("No snapshots found.")
            return
        for snapshot in snapshots:
            print(f"{snapshot.snapshot_id}  {snapshot.created_at}  {snapshot.label}")
        return

    if args.command == "show":
        print_snapshot(show_snapshot(args.snapshot_id), verbose=True)
        return

    if args.command == "restore":
        snapshot = restore_snapshot(args.snapshot_id)
        print("Snapshot restored.")
        print_snapshot(snapshot)
        return


if __name__ == "__main__":
    main()
