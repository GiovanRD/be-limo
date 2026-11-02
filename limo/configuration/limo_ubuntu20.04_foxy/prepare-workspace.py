#!/usr/bin/env python3
"""Prepare the physical LIMO workspace using symbolic links and temporary manifests."""

import argparse
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

PACKAGES = {
    "limo_ros2": "limo_ros2",
    "limo_start": "limo_start",
    "ydlidar_ros2_driver": "ydlidar_ros2_driver",
    "astra_camera": "ros2_astra_camera",
}

CAMERA_PACKAGES = {"astra_camera"}

LIMO_ROS_PACKAGES = (
    "limo_msgs",
    "limo_base",
    "limo_bringup",
    "limo_description",
)

CAMERA_ROS_PACKAGES = (
    "astra_camera_msgs",
    "astra_camera",
)


def validate_package(path, expected_name=None):
    """Validate a ROS package manifest."""
    manifest = path / "package.xml"

    if not manifest.is_file():
        raise ValueError("Missing package manifest: {}".format(manifest))

    package = ET.parse(str(manifest)).getroot()
    name = package.findtext("name")

    if not name:
        raise ValueError("Missing package name in {}".format(manifest))

    if expected_name and name != expected_name:
        raise ValueError(
            "Expected package {!r} in {}, found {!r}".format(
                expected_name, manifest, name
            )
        )

    return name


def package_directories(source, name):
    """Return the ROS package directories represented by a source entry."""
    directory = source / PACKAGES[name]

    if name == "limo_ros2":
        return [
            (directory / package_name, package_name)
            for package_name in LIMO_ROS_PACKAGES
        ]

    if name == "astra_camera":
        return [
            (directory / package_name, package_name)
            for package_name in CAMERA_ROS_PACKAGES
        ]

    return [(directory, name)]


def selected_packages(source, without_camera):
    """Validate and return the selected source directories."""
    selected = {
        name: source / relative
        for name, relative in PACKAGES.items()
        if not without_camera or name not in CAMERA_PACKAGES
    }

    for name, directory in selected.items():
        if not directory.is_dir():
            raise ValueError("Missing source directory: {}".format(directory))

        for package_dir, expected_name in package_directories(source, name):
            validate_package(package_dir, expected_name)

    return selected


def prepare_manifest(source, destination, without_camera):
    """Write a filtered manifest copy without modifying source files."""
    tree = ET.parse(str(source / "package.xml"))
    package = tree.getroot()

    for entry in list(package):
        if not entry.tag.endswith("depend"):
            continue

        dependency = (entry.text or "").strip()

        if (
            dependency in {"ament_python", "limo_car"}
            or dependency.startswith("gazebo")
            or (
                without_camera
                and dependency in {"astra_camera", "astra_camera_msgs"}
            )
        ):
            package.remove(entry)

    destination.mkdir(parents=True, exist_ok=True)
    tree.write(
        str(destination / "package.xml"),
        encoding="utf-8",
        xml_declaration=True,
    )


def prepare_manifests(source, destination, selected, without_camera):
    """Generate manifests in a dedicated temporary directory."""
    destination.mkdir(parents=True, exist_ok=True)

    for name in selected:
        for package_dir, package_name in package_directories(source, name):
            prepare_manifest(
                package_dir,
                destination / package_name,
                without_camera,
            )
            print("Prepared manifest: {}".format(package_name))


def prepare_symlinks(destination, selected):
    """Create package-source symlinks without overwriting existing content."""
    destination.mkdir(parents=True, exist_ok=True)

    for name, source_dir in selected.items():
        target = destination / name

        if target.is_symlink():
            current_target = target.resolve()

            if current_target == source_dir.resolve():
                print("Link already correct: {}".format(target))
                continue

            raise ValueError(
                "Refusing to replace an existing symlink: {}".format(target)
            )

        if target.exists():
            raise ValueError(
                "Refusing to overwrite existing path: {}".format(target)
            )

        target.symlink_to(source_dir.resolve(), target_is_directory=True)
        print("Created link: {} -> {}".format(target, source_dir.resolve()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "source",
        type=Path,
        help="Path to ros2_foxy/",
    )
    parser.add_argument(
        "destination",
        type=Path,
        help="Workspace src directory, or temporary manifest directory",
    )
    parser.add_argument(
        "--without-camera",
        action="store_true",
        help="Exclude camera packages",
    )
    parser.add_argument(
        "--manifests-only",
        action="store_true",
        help="Generate filtered manifests only; do not create symlinks",
    )
    args = parser.parse_args()

    source = args.source.resolve()
    destination = args.destination.expanduser().resolve()

    if not source.is_dir():
        parser.error("Source directory does not exist: {}".format(source))

    if (
        source == destination
        or source in destination.parents
        or destination in source.parents
    ):
        parser.error("Source and destination must be separate directories")

    try:
        selected = selected_packages(source, args.without_camera)

        if args.manifests_only:
            prepare_manifests(
                source,
                destination,
                selected,
                args.without_camera,
            )
        else:
            prepare_symlinks(destination, selected)

    except (ValueError, ET.ParseError, OSError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()