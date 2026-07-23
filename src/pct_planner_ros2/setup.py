import os

from glob import glob
from setuptools import find_packages, setup

package_name = "pct_planner_ros2"


def collect_pct_core_data_files():
    data_files = []
    for root, dirs, files in os.walk("PCT_planner"):
        dirs[:] = [name for name in dirs if name not in {"__pycache__", ".git"}]
        selected = [
            os.path.join(root, name)
            for name in files
            if not name.endswith((".pyc", ".pyo"))
        ]
        if selected:
            data_files.append((os.path.join("share", package_name, root), selected))
    return data_files


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml") + glob("config/*.rviz")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/scripts", glob("scripts/*")),
    ] + collect_pct_core_data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orin",
    maintainer_email="orin@localhost",
    description="ROS 2 adapter nodes for the PCT planner core.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pct_planner_node = pct_planner_ros2.pct_planner_node:main",
            "pct_tomography_node = pct_planner_ros2.pct_tomography_node:main",
            "pct_map_viz_node = pct_planner_ros2.pct_map_viz_node:main",
            "pct_check_env = pct_planner_ros2.check_env:main",
            "pct_export_tomogram_surface = pct_planner_ros2.tomogram_surface:main",
        ],
    },
)
