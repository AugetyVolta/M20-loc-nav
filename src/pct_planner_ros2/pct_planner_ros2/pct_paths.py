import os
import sys
from pathlib import Path

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover - usable from source before ROS env is sourced.
    get_package_share_directory = None

DEFAULT_VENV = Path(
    os.environ.get(
        "PCT_VENV",
        "/home/ubuntu/xlab/M20-loc-nav/.venv/m20_nav_jazzy",
    )
)


def expand_path(path_value):
    return Path(os.path.expandvars(os.path.expanduser(str(path_value)))).resolve()


def default_pct_root():
    env_root = os.environ.get("PCT_PLANNER_ROOT")
    if env_root:
        return str(expand_path(env_root))

    candidates = [
        Path(__file__).resolve().parents[1] / "PCT_planner",
    ]

    if get_package_share_directory is not None:
        try:
            candidates.append(Path(get_package_share_directory("pct_planner_ros2")) / "PCT_planner")
        except Exception:
            pass

    ws_root = os.environ.get("PCT_ROS2_WS")
    if ws_root:
        ws_path = expand_path(ws_root)
        candidates.extend(
            [
                ws_path / "src/pct_planner_ros2/PCT_planner",
                ws_path / "PCT_planner",
            ]
        )

    for candidate in candidates:
        if candidate.exists():
            return str(candidate)
    return str(candidates[0])


def default_venv_site_packages():
    python_dir = f"python{sys.version_info.major}.{sys.version_info.minor}"
    return str(DEFAULT_VENV / "lib" / python_dir / "site-packages")


def tomogram_stem(name):
    name = str(name)
    return name[:-7] if name.endswith(".pickle") else name


def prepend_python_paths(paths):
    for path in reversed([str(p) for p in paths]):
        if path not in sys.path:
            sys.path.insert(0, path)


def configure_planner_imports(pct_root):
    root = expand_path(pct_root)
    prepend_python_paths(
        [
            root / "planner/scripts",
            root / "planner",
            root / "planner/lib",
        ]
    )
    return root


def configure_tomography_imports(pct_root):
    root = expand_path(pct_root)
    prepend_python_paths(
        [
            root / "tomography/scripts",
            root / "tomography",
        ]
    )
    return root


def required_library_dirs(pct_root):
    root = expand_path(pct_root)
    return [
        root / "planner/lib/3rdparty/gtsam-4.1.1/install/lib",
        root / "planner/lib/3rdparty/osqp/install/lib",
        root / "planner/lib",
        root / "planner/lib/build/src/common/smoothing",
    ]


def missing_ld_library_dirs(pct_root):
    configured = set(filter(None, os.environ.get("LD_LIBRARY_PATH", "").split(":")))
    return [str(path) for path in required_library_dirs(pct_root) if path.exists() and str(path) not in configured]


def vector3(value, default):
    if value is None:
        value = default
    if isinstance(value, str):
        value = [float(part.strip()) for part in value.split(",") if part.strip()]
    result = [float(v) for v in value]
    if len(result) != 3:
        raise ValueError(f"expected a 3 element vector, got {value!r}")
    return result
