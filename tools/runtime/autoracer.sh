#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

usage() {
  cat <<'EOF'
Usage:
  tools/runtime/autoracer.sh map [--counts <value>] [--no-rviz] [--skip-preflight] [--dry-run] [-- <extra launch args>]
  tools/runtime/autoracer.sh nav --map <map.yaml> [--counts <value>] [--no-rviz] [--reverse] [--skip-preflight] [--no-capture] [--capture-raw-cloud] [--dry-run] [-- <extra launch args>]
  tools/runtime/autoracer.sh preflight map [--counts <value>] [-- <extra launch args>]
  tools/runtime/autoracer.sh preflight nav --map <map.yaml> [--counts <value>] [--no-capture] [--capture-raw-cloud] [-- <extra launch args>]

Final user-facing modes:
  map   Start the Stage-3 2D mapping stack.
  nav   Start the Stage-4 map navigation stack with companion diagnostics capture.
  preflight
        Run startup checks without launching map/nav.

Options:
  --counts <value>   Measured Hall counts per meter. Defaults to the current calibrated value: 13.545.
  --map <map.yaml>   Saved map YAML for nav mode. Required for nav.
  --no-rviz          Disable RViz.
  --reverse          Nav only: use Stage-4B reverse candidate params and allow_reverse:=true.
  --skip-preflight   Skip the default startup checks.
  --no-capture       Nav only: disable the default companion navigation capture.
  --capture-raw-cloud
                     Nav only: include /point_cloud_raw in the companion capture.
  --dry-run          Print the final command(s) without sourcing ROS or running them.
  -h, --help         Show this help.

Any arguments after "--" are passed directly to the underlying launch file.
EOF
}

die() {
  echo "autoracer.sh: $*" >&2
  exit 2
}

quote_command() {
  local first=1
  for arg in "$@"; do
    if [[ "${first}" -eq 0 ]]; then
      printf ' '
    fi
    first=0
    printf '%q' "${arg}"
  done
  printf '\n'
}

is_positive_number() {
  awk -v value="$1" 'BEGIN { exit !(value + 0 == value && value > 0) }'
}

require_counts() {
  local value="$1"
  is_positive_number "${value}" || die "--counts must be a measured positive number"
}

new_run_id() {
  date '+%Y-%m-%d-nav-%H%M%S'
}

source_runtime() {
  cd "${REPO_ROOT}"
  set +e
  set +u
  # shellcheck source=/dev/null
  source "${REPO_ROOT}/source_all.sh"
  local source_status="$?"
  set -e
  set -u
  return "${source_status}"
}

run_preflight() {
  local dry_run="$1"
  local runtime_mode="$2"
  local run_id="$3"
  local map_file="$4"
  local counts="$5"
  local capture_enabled="$6"
  local capture_raw_cloud="$7"
  shift 7

  local -a preflight_cmd=(
    python3 tools/runtime/preflight_check.py
    --mode "${runtime_mode}"
    --counts "${counts}"
    --run-id "${run_id}"
    --output-root "${REPO_ROOT}/docs/test-records/artifacts"
  )
  if [[ -n "${map_file}" ]]; then
    preflight_cmd+=(--map "${map_file}")
  fi
  if [[ "${capture_enabled}" == "true" ]]; then
    preflight_cmd+=(--capture-enabled)
  fi
  if [[ "${capture_raw_cloud}" == "true" ]]; then
    preflight_cmd+=(--capture-raw-cloud)
  fi
  preflight_cmd+=(-- "$@")

  if [[ "${dry_run}" == "true" ]]; then
    echo "# startup preflight"
    quote_command "${preflight_cmd[@]}"
    return 0
  fi

  "${preflight_cmd[@]}"
}

run_launch() {
  local dry_run="$1"
  local skip_preflight="$2"
  local runtime_mode="$3"
  local run_id="$4"
  local map_file="$5"
  local counts="$6"
  shift 6

  local -a launch_cmd=("$@")

  if [[ "${dry_run}" == "true" ]]; then
    if [[ "${skip_preflight}" == "false" ]]; then
      run_preflight true "${runtime_mode}" "${run_id}" "${map_file}" "${counts}" false false "${launch_cmd[@]:3}"
    fi
    quote_command "${launch_cmd[@]}"
    return 0
  fi

  source_runtime
  if [[ "${skip_preflight}" == "false" ]]; then
    run_preflight false "${runtime_mode}" "${run_id}" "${map_file}" "${counts}" false false "${launch_cmd[@]:3}"
  fi
  exec "${launch_cmd[@]}"
}

run_nav_with_capture() {
  local dry_run="$1"
  local skip_preflight="$2"
  local run_id="$3"
  local map_file="$4"
  local counts="$5"
  local capture_raw_cloud="$6"
  shift 6

  local -a nav_cmd=("$@")
  local -a capture_cmd=(python3 tools/diagnostics/nav_capture.py --run-id "${run_id}")
  if [[ "${capture_raw_cloud}" == "true" ]]; then
    capture_cmd+=(--raw-cloud)
  fi

  if [[ "${dry_run}" == "true" ]]; then
    if [[ "${skip_preflight}" == "false" ]]; then
      run_preflight true nav "${run_id}" "${map_file}" "${counts}" true "${capture_raw_cloud}" "${nav_cmd[@]:3}"
    fi
    echo "# navigation"
    quote_command "${nav_cmd[@]}"
    echo "# companion capture"
    quote_command "${capture_cmd[@]}"
    return 0
  fi

  source_runtime
  if [[ "${skip_preflight}" == "false" ]]; then
    run_preflight false nav "${run_id}" "${map_file}" "${counts}" true "${capture_raw_cloud}" "${nav_cmd[@]:3}"
  fi

  echo "Starting companion navigation capture. Artifacts will be written under docs/test-records/artifacts/."
  setsid "${capture_cmd[@]}" &
  local capture_pid="$!"

  cleanup_capture() {
    local status="$?"
    trap - EXIT INT TERM
    if kill -0 "${capture_pid}" 2>/dev/null; then
      kill -TERM "${capture_pid}" 2>/dev/null || true
      local waited=0
      while kill -0 "${capture_pid}" 2>/dev/null && [[ "${waited}" -lt 15 ]]; do
        sleep 1
        waited=$((waited + 1))
      done
      if kill -0 "${capture_pid}" 2>/dev/null; then
        kill -KILL "-${capture_pid}" 2>/dev/null || true
      fi
      wait "${capture_pid}" 2>/dev/null || true
    fi
    return "${status}"
  }

  trap cleanup_capture EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM

  "${nav_cmd[@]}"
}

main() {
  if [[ $# -lt 1 ]]; then
    usage
    exit 2
  fi

  local mode="$1"
  shift
  local preflight_only="false"

  if [[ "${mode}" == "preflight" ]]; then
    preflight_only="true"
    if [[ $# -lt 1 ]]; then
      die "preflight requires map or nav"
    fi
    mode="$1"
    shift
  fi

  case "${mode}" in
    -h|--help)
      usage
      exit 0
      ;;
    map|nav)
      ;;
    *)
      die "unknown mode: ${mode}"
      ;;
  esac

  local counts="13.545"
  local map_file=""
  local use_rviz="true"
  local reverse="false"
  local capture="true"
  local capture_raw_cloud="false"
  local skip_preflight="false"
  local dry_run="false"
  local -a extra_args=()

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --counts)
        [[ $# -ge 2 ]] || die "--counts requires a value"
        counts="$2"
        shift 2
        ;;
      --map)
        [[ $# -ge 2 ]] || die "--map requires a value"
        map_file="$2"
        shift 2
        ;;
      --no-rviz)
        use_rviz="false"
        shift
        ;;
      --reverse)
        reverse="true"
        shift
        ;;
      --skip-preflight)
        skip_preflight="true"
        shift
        ;;
      --no-capture)
        capture="false"
        shift
        ;;
      --capture-raw-cloud)
        capture_raw_cloud="true"
        shift
        ;;
      --dry-run)
        dry_run="true"
        shift
        ;;
      --)
        shift
        extra_args=("$@")
        break
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        die "unknown option for ${mode}: $1"
        ;;
    esac
  done

  require_counts "${counts}"
  local run_id
  run_id="$(new_run_id)"

  if [[ "${preflight_only}" == "true" ]]; then
    [[ "${dry_run}" == "false" ]] || die "--dry-run is not valid for preflight mode"
    [[ "${skip_preflight}" == "false" ]] || die "--skip-preflight is not valid for preflight mode"
    [[ "${use_rviz}" == "true" ]] || die "--no-rviz is not valid for preflight mode"
    if [[ "${mode}" == "map" ]]; then
      [[ -z "${map_file}" ]] || die "--map is only valid for nav mode"
      [[ "${reverse}" == "false" ]] || die "--reverse is only valid for nav mode"
      [[ "${capture}" == "true" ]] || die "--no-capture is only valid for nav mode"
      [[ "${capture_raw_cloud}" == "false" ]] || die "--capture-raw-cloud is only valid for nav mode"
    else
      [[ -n "${map_file}" ]] || die "nav mode requires --map <map.yaml>"
      [[ -f "${map_file}" ]] || die "map file does not exist: ${map_file}"
    fi
    local preflight_capture="${capture}"
    if [[ "${mode}" == "map" ]]; then
      preflight_capture="false"
    fi
    source_runtime
    run_preflight false "${mode}" "${run_id}" "${map_file}" "${counts}" "${preflight_capture}" "${capture_raw_cloud}" "${extra_args[@]}"
    return 0
  fi

  if [[ "${mode}" == "map" ]]; then
    [[ -z "${map_file}" ]] || die "--map is only valid for nav mode"
    [[ "${reverse}" == "false" ]] || die "--reverse is only valid for nav mode"
    [[ "${capture}" == "true" ]] || die "--no-capture is only valid for nav mode"
    [[ "${capture_raw_cloud}" == "false" ]] || die "--capture-raw-cloud is only valid for nav mode"
    local -a cmd=(
      ros2 launch autoracer_bringup stage3_mapping.launch.py
      "counts_per_meter:=${counts}"
      "use_rviz:=${use_rviz}"
    )
    cmd+=("${extra_args[@]}")
    run_launch "${dry_run}" "${skip_preflight}" map "${run_id}" "" "${counts}" "${cmd[@]}"
    return 0
  fi

  [[ -n "${map_file}" ]] || die "nav mode requires --map <map.yaml>"
  [[ -f "${map_file}" ]] || die "map file does not exist: ${map_file}"

  local -a cmd=(
    ros2 launch autoracer_bringup stage4_navigation.launch.py
    "counts_per_meter:=${counts}"
    "map:=${map_file}"
    "use_rviz:=${use_rviz}"
  )

  if [[ "${reverse}" == "true" ]]; then
    local reverse_params="${REPO_ROOT}/src/autoracer_robot_nav2/param/stage4_nav2_reverse_params.yaml"
    [[ -f "${reverse_params}" ]] || die "reverse params file does not exist: ${reverse_params}"
    cmd+=("params_file:=${reverse_params}" "allow_reverse:=true")
  fi

  cmd+=("${extra_args[@]}")
  if [[ "${capture}" == "true" ]]; then
    run_nav_with_capture "${dry_run}" "${skip_preflight}" "${run_id}" "${map_file}" "${counts}" "${capture_raw_cloud}" "${cmd[@]}"
  else
    [[ "${capture_raw_cloud}" == "false" ]] || die "--capture-raw-cloud requires companion capture"
    run_launch "${dry_run}" "${skip_preflight}" nav "${run_id}" "${map_file}" "${counts}" "${cmd[@]}"
  fi
}

main "$@"
