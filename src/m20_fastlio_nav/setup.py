from glob import glob
from setuptools import setup

package_name = "m20_fastlio_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/hook", glob("hook/*")),
        (f"share/{package_name}/config", glob("config/*.yaml") + glob("config/*.rviz")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/scripts", glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orin",
    maintainer_email="orin@example.com",
    description="Fast-LIO2 and Open3D localization integration for the M20 Nav2 stack.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "fastlio_odom_bridge = m20_fastlio_nav.fastlio_odom_bridge:main",
            "ego_odom_bridge = m20_fastlio_nav.ego_frame_bridges:ego_odom_bridge_main",
            "ego_cloud_frame_bridge = m20_fastlio_nav.ego_frame_bridges:ego_cloud_frame_bridge_main",
            "initialpose_3d_marker = m20_fastlio_nav.initialpose_3d_marker:main",
            "prepare_3d_nav_map = m20_fastlio_nav.prepare_3d_nav_map:main",
        ],
    },
)
