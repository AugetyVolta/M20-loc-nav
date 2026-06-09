#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Use: source ./source_m20_nav.sh"
  exit 2
fi

_m20_nav_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

_m20_source_if_exists() {
  local setup_file="$1"
  if [ -f "${setup_file}" ]; then
    source "${setup_file}"
    return 0
  fi
  return 1
}

_m20_prepend_path() {
  local var_name="$1"
  local value="$2"
  [ -n "${value}" ] || return 0
  [ -d "${value}" ] || return 0
  _m20_prepend_value "${var_name}" "${value}"
}

_m20_prepend_value() {
  local var_name="$1"
  local value="$2"
  [ -n "${value}" ] || return 0
  case ":${!var_name:-}:" in
    *":${value}:"*) ;;
    *) export "${var_name}=${value}:${!var_name:-}" ;;
  esac
}

_m20_ld_preload_has_realpath() {
  local target="$1"
  local item=""
  [ -n "${target}" ] || return 1
  [ -e "${target}" ] || return 1
  target="$(readlink -f "${target}" 2>/dev/null || echo "${target}")"
  IFS=":" read -ra _m20_ld_items <<< "${LD_PRELOAD:-}"
  for item in "${_m20_ld_items[@]}"; do
    [ -n "${item}" ] || continue
    if [ -e "${item}" ] && [ "$(readlink -f "${item}" 2>/dev/null || echo "${item}")" = "${target}" ]; then
      return 0
    fi
  done
  return 1
}

export M20_FASTLIO_WS="${M20_FASTLIO_WS:-${_m20_nav_script_dir}}"
export M20_PCT_PKG="${M20_PCT_PKG:-${M20_FASTLIO_WS}/src/pct_planner_ros2}"
export PCT_ROS2_WS="${PCT_ROS2_WS:-${M20_FASTLIO_WS}}"
export PCT_PLANNER_ROOT="${PCT_PLANNER_ROOT:-${M20_PCT_PKG}/PCT_planner}"
export PCT_VENV="${PCT_VENV:-/home/orin/venv/m20_nav_cupy}"
export PCT_VENV_SITE="${PCT_VENV_SITE:-${PCT_VENV}/lib/python3.10/site-packages}"
export M20_NAV_PYTHON="${M20_NAV_PYTHON:-/home/orin/venv/m20_nav/bin/python}"
export M20_NAV_CUPY_PYTHON="${M20_NAV_CUPY_PYTHON:-${PCT_VENV}/bin/python}"
export M20_MAP_PCD="${M20_MAP_PCD:-${M20_FASTLIO_WS}/maps/fastlio/m20_3d_map.pcd}"
export M20_LIV_WS="${M20_LIV_WS:-${HOME}/liv_ws}"
export M20_LIVEX_LIBUSB="${M20_LIVEX_LIBUSB:-/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0}"

_m20_source_if_exists "/opt/ros/humble/setup.bash" || echo "[m20_nav] missing /opt/ros/humble/setup.bash"
_m20_source_if_exists "${M20_LIV_WS}/install/setup.bash" || true
_m20_source_if_exists "${M20_FASTLIO_WS}/install/setup.bash" || true

_m20_prepend_path PATH "${PCT_VENV}/bin"
_m20_prepend_path PYTHONPATH "${PCT_VENV_SITE}"
_m20_prepend_path PYTHONPATH "${PCT_PLANNER_ROOT}/planner/lib"
_m20_prepend_path PYTHONPATH "${PCT_PLANNER_ROOT}/planner"
_m20_prepend_path PYTHONPATH "${PCT_PLANNER_ROOT}/planner/scripts"
_m20_prepend_path PYTHONPATH "${PCT_PLANNER_ROOT}/tomography"
_m20_prepend_path PYTHONPATH "${PCT_PLANNER_ROOT}/tomography/scripts"
_m20_prepend_path LD_LIBRARY_PATH "${PCT_PLANNER_ROOT}/planner/lib/3rdparty/gtsam-4.1.1/install/lib"
_m20_prepend_path LD_LIBRARY_PATH "${PCT_PLANNER_ROOT}/planner/lib/3rdparty/osqp/install/lib"
_m20_prepend_path LD_LIBRARY_PATH "${PCT_PLANNER_ROOT}/planner/lib"
_m20_prepend_path LD_LIBRARY_PATH "${PCT_PLANNER_ROOT}/planner/lib/build/src/common/smoothing"

if [ -d "${M20_FASTLIO_WS}/install/pct_planner_ros2" ]; then
  _m20_prepend_path AMENT_PREFIX_PATH "${M20_FASTLIO_WS}/install/pct_planner_ros2"
  _m20_prepend_path COLCON_PREFIX_PATH "${M20_FASTLIO_WS}/install"
  _m20_prepend_path PATH "${M20_FASTLIO_WS}/install/pct_planner_ros2/lib/pct_planner_ros2"
  _m20_prepend_path PYTHONPATH "${M20_FASTLIO_WS}/install/pct_planner_ros2/lib/python3.10/site-packages"
fi

if [ -f "${M20_LIVEX_LIBUSB}" ]; then
  if ! _m20_ld_preload_has_realpath "${M20_LIVEX_LIBUSB}"; then
    _m20_prepend_value LD_PRELOAD "${M20_LIVEX_LIBUSB}"
  fi
  M20_LIVOX_PRELOAD=(env "LD_PRELOAD=${LD_PRELOAD}")
else
  M20_LIVOX_PRELOAD=(env)
fi
M20_LIVEX_PRELOAD=("${M20_LIVOX_PRELOAD[@]}")

export M20_NAV_ENV_LOADED=1

echo "[m20_nav] workspace: ${M20_FASTLIO_WS}"
echo "[m20_nav] pct root:  ${PCT_PLANNER_ROOT}"
echo "[m20_nav] map pcd:   ${M20_MAP_PCD}"

unset -f _m20_source_if_exists
unset -f _m20_prepend_path
unset -f _m20_prepend_value
unset -f _m20_ld_preload_has_realpath
unset _m20_ld_items
unset _m20_nav_script_dir
