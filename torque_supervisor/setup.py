from setuptools import setup
import os
from glob import glob

package_name = "torque_supervisor"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rudolf Krecht",
    maintainer_email="krecht.rudolf@ga.sze.hu",
    description="Model-based cmd_vel supervisor with torque estimation and saturation prevention.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "torque_supervisor = torque_supervisor.torque_supervisor_node:main",
        ],
    },
)
