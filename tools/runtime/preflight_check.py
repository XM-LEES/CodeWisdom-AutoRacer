#!/usr/bin/env python3
"""Startup preflight checks for final map/nav runtime commands."""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import stat
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any


PASS = "PASS"
WARN = "WARN"
FAIL = "FAIL"


@dataclass
class CheckResult:
    name: str
    status: str
    detail: str
    hint: str = ""

    def as_dict(self) -> dict[str, str]:
        data = {"name": self.name, "status": self.status, "detail": self.detail}
        if self.hint:
            data["hint"] = self.hint
        return data


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def run_command(args: list[str], timeout: float = 4.0) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            args,
            cwd=str(repo_root()),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
            check=False,
        )
    except FileNotFoundError as exc:
        return subprocess.CompletedProcess(args=args, returncode=127, stdout="", stderr=str(exc))
    except subprocess.TimeoutExpired as exc:
        return subprocess.CompletedProcess(args=args, returncode=124, stdout=exc.stdout or "", stderr=exc.stderr or "timeout")


def parse_launch_overrides(values: list[str]) -> dict[str, str]:
    overrides: dict[str, str] = {}
    for value in values:
        if ":=" not in value:
            continue
        key, raw = value.split(":=", 1)
        if key:
            overrides[key] = raw
    return overrides


def bool_override(overrides: dict[str, str], key: str, default: bool) -> bool:
    value = overrides.get(key)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def resolve_repo_path(value: str | None, default: Path) -> Path:
    if not value:
        return default
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    return (repo_root() / path).resolve()


def read_simple_yaml_value(path: Path, key: str) -> str | None:
    if not path.is_file():
        return None
    pattern = re.compile(rf"^\s*{re.escape(key)}\s*:\s*(.+?)\s*(?:#.*)?$")
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        match = pattern.match(line)
        if not match:
            continue
        return match.group(1).strip().strip('"').strip("'")
    return None


def check_path_exists(name: str, path: Path, kind: str = "file") -> CheckResult:
    if path.is_file():
        return CheckResult(name, PASS, f"{kind} exists: {path}")
    return CheckResult(name, FAIL, f"{kind} missing: {path}", "confirm the path or rebuild/copy the required artifact")


def check_serial_device(name: str, path: Path) -> CheckResult:
    if not path.exists():
        return CheckResult(name, FAIL, f"device missing: {path}", "check USB cable, power, udev rule, and device name")
    try:
        mode = path.stat().st_mode
    except OSError as exc:
        return CheckResult(name, FAIL, f"cannot stat {path}: {exc}", "check device permissions")
    if not stat.S_ISCHR(mode):
        return CheckResult(name, WARN, f"path exists but is not a character device: {path}", "verify the configured device path")
    if not os.access(path, os.R_OK | os.W_OK):
        return CheckResult(name, FAIL, f"device exists but is not readable/writable: {path}", "add the user to dialout or fix udev permissions")
    return CheckResult(name, PASS, f"device ready: {path}")


def check_ros2_package(package: str) -> CheckResult:
    if shutil.which("ros2") is None:
        return CheckResult("ros2_package", FAIL, "ros2 command not found", "source ROS and workspace setup before running")
    result = run_command(["ros2", "pkg", "prefix", package], timeout=5.0)
    if result.returncode == 0 and result.stdout.strip():
        return CheckResult(f"package:{package}", PASS, result.stdout.strip())
    detail = (result.stderr or result.stdout).strip() or "not found"
    return CheckResult(f"package:{package}", FAIL, detail, "run colcon build and source install/setup.bash")


def check_disk_space(output_root: Path, capture_enabled: bool, raw_cloud: bool, mode: str) -> CheckResult:
    output_root.mkdir(parents=True, exist_ok=True)
    usage = shutil.disk_usage(output_root)
    free_gb = usage.free / (1024 ** 3)
    if raw_cloud:
        required_gb = 10.0
    elif capture_enabled:
        required_gb = 2.0
    elif mode == "map":
        required_gb = 1.0
    else:
        required_gb = 0.5
    if free_gb >= required_gb:
        return CheckResult("artifact_disk_space", PASS, f"{free_gb:.1f} GB free at {output_root}")
    return CheckResult(
        "artifact_disk_space",
        FAIL,
        f"{free_gb:.1f} GB free at {output_root}, need at least {required_gb:.1f} GB",
        "free disk space or disable capture only for non-diagnostic runs",
    )


def check_map_file(map_file: Path) -> list[CheckResult]:
    results = [check_path_exists("map_yaml", map_file, "map yaml")]
    if not map_file.is_file():
        return results
    image_value = read_simple_yaml_value(map_file, "image")
    if not image_value:
        results.append(CheckResult("map_image", WARN, f"map image field not found in {map_file}", "verify map yaml format"))
        return results
    image_path = Path(image_value).expanduser()
    if not image_path.is_absolute():
        image_path = (map_file.parent / image_path).resolve()
    results.append(check_path_exists("map_image", image_path, "map image"))
    return results


def check_lidar_network(params_file: Path) -> list[CheckResult]:
    results: list[CheckResult] = [check_path_exists("lidar_params", params_file, "LiDAR params")]
    device_ip = read_simple_yaml_value(params_file, "device_ip") or "192.168.1.200"
    if shutil.which("ip") is not None:
        addr = run_command(["ip", "-4", "addr"], timeout=3.0)
        if re.search(r"\b192\.168\.1\.\d+\b", addr.stdout):
            results.append(CheckResult("lidar_local_network", PASS, "local 192.168.1.x address found"))
        else:
            results.append(
                CheckResult(
                    "lidar_local_network",
                    FAIL,
                    "no local 192.168.1.x address found",
                    "configure the Jetson Ethernet interface for the LiDAR subnet",
                )
            )
    else:
        results.append(CheckResult("lidar_local_network", WARN, "ip command not found", "cannot inspect local LiDAR subnet"))

    if shutil.which("ping") is None:
        results.append(CheckResult("lidar_ping", WARN, "ping command not found", f"cannot ping LiDAR {device_ip}"))
        return results
    ping = run_command(["ping", "-c", "1", "-W", "1", device_ip], timeout=3.0)
    if ping.returncode == 0:
        results.append(CheckResult("lidar_ping", PASS, f"LiDAR responds at {device_ip}"))
    else:
        results.append(
            CheckResult(
                "lidar_ping",
                FAIL,
                f"LiDAR did not respond at {device_ip}",
                "check LiDAR power, Ethernet cable, Jetson IP, and sensor IP",
            )
        )
    return results


def package_checks(mode: str, start_imu: bool, start_chassis: bool, start_lidar: bool, start_nav2: bool) -> list[str]:
    packages = ["autoracer_bringup"]
    if start_imu:
        packages.extend(["hipnuc_imu", "imu_filter_madgwick"])
    if start_chassis:
        packages.append("turn_on_autoracer_robot")
    if start_lidar:
        packages.append("lslidar_driver")
    if mode == "map":
        packages.append("autoracer_slam_toolbox")
    if mode == "nav" and start_nav2:
        packages.extend([
            "autoracer_robot_nav2",
            "nav2_bringup",
            "nav2_collision_monitor",
            "pointcloud_to_laserscan",
        ])
    return sorted(set(packages))


def build_results(args: argparse.Namespace, overrides: dict[str, str], output_root: Path) -> list[CheckResult]:
    root = repo_root()
    results: list[CheckResult] = []
    start_imu = bool_override(overrides, "start_imu", True)
    start_chassis = bool_override(overrides, "start_chassis", True)
    start_lidar = bool_override(overrides, "start_lidar", True)
    start_nav2 = bool_override(overrides, "start_nav2", True)

    results.append(check_path_exists("source_all", root / "source_all.sh", "runtime source script"))
    results.append(check_path_exists("workspace_setup", root / "install" / "setup.bash", "workspace setup"))

    try:
        counts = float(args.counts)
    except ValueError:
        counts = -1.0
    if counts > 0.0:
        results.append(CheckResult("counts_per_meter", PASS, f"{counts:.3f}"))
    else:
        results.append(CheckResult("counts_per_meter", FAIL, f"invalid counts_per_meter: {args.counts}", "use the measured positive calibration value"))

    results.append(check_disk_space(output_root, args.capture_enabled, args.capture_raw_cloud, args.mode))

    if args.mode == "nav":
        if not args.map:
            results.append(CheckResult("map_yaml", FAIL, "nav requires --map", "provide a saved Stage-3 map yaml"))
        elif start_nav2:
            results.extend(check_map_file(resolve_repo_path(args.map, root / args.map)))

    for package in package_checks(args.mode, start_imu, start_chassis, start_lidar, start_nav2):
        results.append(check_ros2_package(package))

    if start_chassis:
        chassis_port = resolve_repo_path(overrides.get("usart_port_name"), Path("/dev/ttyACM0"))
        results.append(check_serial_device("chassis_serial", chassis_port))

    if start_imu:
        imu_config = root / "src" / "hipnuc_imu" / "config" / "hipnuc_config.yaml"
        results.append(check_path_exists("imu_config", imu_config, "IMU config"))
        imu_port_value = read_simple_yaml_value(imu_config, "serial_port") or "/dev/autoracer_imu"
        results.append(check_serial_device("imu_serial", Path(imu_port_value).expanduser()))

    if start_lidar:
        lidar_params = root / "src" / "autoracer_lidar_ros2" / "lslidar_ros" / "lslidar_driver" / "params" / "lslidar_cx.yaml"
        results.extend(check_lidar_network(lidar_params))

    params_file = overrides.get("params_file")
    if args.mode == "nav":
        default_params = root / "src" / "autoracer_robot_nav2" / "param" / "stage4_nav2_params.yaml"
        results.append(check_path_exists("nav2_params", resolve_repo_path(params_file, default_params), "Nav2 params"))

    return results


def write_report(args: argparse.Namespace, output_root: Path, overrides: dict[str, str], results: list[CheckResult]) -> Path:
    artifact_dir = output_root / args.run_id
    report_path = artifact_dir / "checks" / "startup_preflight.json"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    failed = [item for item in results if item.status == FAIL]
    report = {
        "run_id": args.run_id,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "mode": args.mode,
        "passed": not failed,
        "map": args.map,
        "counts": args.counts,
        "capture_enabled": args.capture_enabled,
        "capture_raw_cloud": args.capture_raw_cloud,
        "launch_overrides": overrides,
        "results": [item.as_dict() for item in results],
    }
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report_path


def print_results(mode: str, results: list[CheckResult], report_path: Path) -> None:
    failed = [item for item in results if item.status == FAIL]
    title = "PASS" if not failed else "FAIL"
    print(f"Startup preflight ({mode}): {title}")
    for item in results:
        print(f"{item.status:4} {item.name}: {item.detail}")
        if item.hint and item.status != PASS:
            print(f"     hint: {item.hint}")
    print(f"Preflight report: {report_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", required=True, choices=["map", "nav"])
    parser.add_argument("--map", default="")
    parser.add_argument("--counts", default="13.545")
    parser.add_argument("--run-id", default=datetime.now().strftime("%Y-%m-%d-preflight-%H%M%S"))
    parser.add_argument("--output-root", default=str(repo_root() / "docs" / "test-records" / "artifacts"))
    parser.add_argument("--capture-enabled", action="store_true")
    parser.add_argument("--capture-raw-cloud", action="store_true")
    parser.add_argument("launch_args", nargs=argparse.REMAINDER)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    launch_args = list(args.launch_args)
    if launch_args and launch_args[0] == "--":
        launch_args = launch_args[1:]
    overrides = parse_launch_overrides(launch_args)
    output_root = Path(args.output_root).expanduser().resolve()
    results = build_results(args, overrides, output_root)
    report_path = write_report(args, output_root, overrides, results)
    print_results(args.mode, results, report_path)
    return 1 if any(item.status == FAIL for item in results) else 0


if __name__ == "__main__":
    raise SystemExit(main())
