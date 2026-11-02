from glob import glob
import os

from setuptools import find_packages, setup

package_name = "teleoperation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jeremie",
    maintainer_email="jeremie.kreiss@univ-lorraine.fr",
    description="Reusable Tkinter teleoperation components for robotic platforms.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "teleop_limo_real = teleoperation.teleop_limo_real:main",
            "teleop_limo_simu = teleoperation.teleop_limo_simu:main",
        ],
    },
)