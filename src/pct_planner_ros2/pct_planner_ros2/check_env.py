import importlib
import os
import sys

from .pct_paths import (
    configure_planner_imports,
    configure_tomography_imports,
    default_pct_root,
    default_venv_site_packages,
    missing_ld_library_dirs,
)


def _check_import(module_name):
    module = importlib.import_module(module_name)
    version = getattr(module, "__version__", "ok")
    print(f"{module_name}: {version}")


def main():
    pct_root = os.environ.get("PCT_PLANNER_ROOT", default_pct_root())
    print(f"python: {sys.executable}")
    print(f"version: {sys.version.split()[0]}")
    print(f"PCT_PLANNER_ROOT: {pct_root}")
    print(f"venv site-packages hint: {default_venv_site_packages()}")

    for path in missing_ld_library_dirs(pct_root):
        print(f"LD_LIBRARY_PATH missing: {path}")

    for module_name in ("rclpy", "numpy", "scipy", "open3d", "cupy", "interactive_markers"):
        try:
            _check_import(module_name)
        except Exception as exc:
            print(f"{module_name}: FAILED ({exc})")

    configure_tomography_imports(pct_root)
    try:
        _check_import("tomogram")
    except Exception as exc:
        print(f"tomogram: FAILED ({exc})")

    configure_planner_imports(pct_root)
    for module_name in ("planner_wrapper", "lib.a_star", "lib.traj_opt", "lib.ele_planner"):
        try:
            _check_import(module_name)
        except Exception as exc:
            print(f"{module_name}: FAILED ({exc})")
