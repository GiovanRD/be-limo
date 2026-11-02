#!/usr/bin/env python3

import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def main():
    if len(sys.argv) != 3:
        print(
            "Usage: collect-manifests.py SOURCE_DIR DESTINATION_DIR",
            file=sys.stderr,
        )
        return 2

    source = Path(sys.argv[1]).resolve()
    destination = Path(sys.argv[2]).resolve()
    excluded = {".git", "build", "install", "log", "__pycache__"}
    packages = {}

    for manifest in source.glob("*/**/package.xml"):
        if excluded.intersection(manifest.parts):
            continue

        try:
            name = ET.parse(manifest).getroot().findtext("name")
        except ET.ParseError as exc:
            print(f"Invalid manifest {manifest}: {exc}", file=sys.stderr)
            return 1

        if not name:
            print(f"Missing package name in {manifest}", file=sys.stderr)
            return 1

        if name in packages:
            print(
                f"Duplicate package name '{name}':\n"
                f"  {packages[name]}\n"
                f"  {manifest}",
                file=sys.stderr,
            )
            return 1

        packages[name] = manifest

    destination.mkdir(parents=True, exist_ok=True)
    for name, manifest in packages.items():
        target_dir = destination / name
        target_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(manifest, target_dir / "package.xml")

    print(f"Collected {len(packages)} package manifests.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())