#!/usr/bin/env python3
"""Record and summarize a map-navigation session.

This tool observes an already-running Stage-4 navigation graph. It does not
start navigation, send goals, or publish motion commands.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any


DEFAULT_TOPICS = [
    "/scan",
    "/tf",
    "/tf_static",
    "/odom",
    "/wheel_odom",
    "/imu/data",
    "/amcl_pose",
    "/map",
    "/map_metadata",
    "/global_costmap/costmap",
    "/global_costmap/costmap_updates",
    "/global_costmap/published_footprint",
    "/local_costmap/costmap",
    "/local_costmap/costmap_updates",
    "/local_costmap/published_footprint",
    "/plan",
    "/received_global_plan",
    "/local_plan",
    "/lookahead_point",
    "/lookahead_collision_arc",
    "/speed_limit",
    "/cmd_vel_nav",
    "/nav2_cmd_vel",
    "/safe_nav2_cmd_vel",
    "/collision_monitor/state",
    "/twist_to_ackermann/diagnostics",
    "/ackermann_cmd",
    "/chassis_state",
    "/navigate_to_pose/_action/status",
    "/navigate_to_pose/_action/feedback",
    "/behavior_tree_log",
    "/rosout",
]

LITE_TOPICS = [
    "/scan",
    "/tf",
    "/tf_static",
    "/odom",
    "/amcl_pose",
    "/plan",
    "/cmd_vel_nav",
    "/nav2_cmd_vel",
    "/safe_nav2_cmd_vel",
    "/twist_to_ackermann/diagnostics",
    "/ackermann_cmd",
    "/chassis_state",
    "/navigate_to_pose/_action/status",
    "/rosout",
]

RAW_CLOUD_TOPIC = "/point_cloud_raw"

NODE_INFO_TARGETS = [
    "/map_server",
    "/amcl",
    "/planner_server",
    "/controller_server",
    "/velocity_smoother",
    "/collision_monitor",
    "/bt_navigator",
    "/twist_to_ackermann",
    "/ackermann_chassis_bridge",
]

ACTIVE_STATUS = {1, 2, 3}
TERMINAL_STATUS = {4, 5, 6}
STATUS_NAMES = {
    0: "unknown",
    1: "accepted",
    2: "executing",
    3: "canceling",
    4: "succeeded",
    5: "canceled",
    6: "aborted",
}

FIRST_FAILED_LAYERS = {
    "sensor_localization",
    "planner_costmap",
    "bt_navigator",
    "controller",
    "velocity_smoother",
    "collision_monitor",
    "adapter",
    "chassis",
    "unknown_insufficient_evidence",
}

ROSOUT_KEYWORDS = (
    "abort",
    "failed",
    "failure",
    "timeout",
    "exceeded",
    "progress",
    "create plan",
    "valid plan",
    "collision",
    "transform",
    "tf",
    "goal",
    "cancel",
    "recover",
)

STOP_REQUESTED = False


@dataclass
class TopicStats:
    count: int = 0
    first_sec: float | None = None
    last_sec: float | None = None


@dataclass
class BagSummary:
    topic_stats: dict[str, TopicStats] = field(default_factory=dict)
    timeline: dict[str, Any] = field(default_factory=dict)
    goal_events: list[dict[str, Any]] = field(default_factory=list)
    evidence: list[str] = field(default_factory=list)
    missing_evidence: list[str] = field(default_factory=list)
    last_rosout_key: dict[str, Any] | None = None
    key_rosout: list[dict[str, Any]] = field(default_factory=list)
    last_adapter_stop_reason: str | None = None
    last_ack_nonzero: bool = False
    chassis_moved: bool = False
    chassis_safety: dict[str, bool] = field(default_factory=dict)
    safe_suppressed: bool = False
    terminal_chassis_stall: bool = False
    last_ack_sample: dict[str, Any] | None = None
    last_chassis_sample: dict[str, Any] | None = None


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def ensure_ros_environment() -> None:
    if os.environ.get("AUTORACER_NAV_CAPTURE_SOURCED") == "1":
        return

    try:
        import rclpy  # noqa: F401
        return
    except ImportError:
        pass

    source_all = repo_root() / "source_all.sh"
    if not source_all.is_file():
        return

    env = os.environ.copy()
    env["AUTORACER_NAV_CAPTURE_SOURCED"] = "1"
    command = 'cd "$1"; shift; source ./source_all.sh; exec python3 tools/diagnostics/nav_capture.py "$@"'
    os.execvpe(
        "bash",
        ["bash", "-lc", command, "bash", str(repo_root()), *sys.argv[1:]],
        env,
    )


def run_command(args: list[str], *, cwd: Path, timeout: float = 8.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        args,
        cwd=str(cwd),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
        check=False,
    )


def parse_topic_list(output: str) -> dict[str, str | None]:
    topics: dict[str, str | None] = {}
    for raw_line in output.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if " [" in line and line.endswith("]"):
            name, msg_type = line.rsplit(" [", 1)
            topics[name.strip()] = msg_type[:-1].strip()
        else:
            topics[line] = None
    return topics


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def request_stop(signum: int, frame: Any) -> None:
    del signum, frame
    global STOP_REQUESTED
    STOP_REQUESTED = True


def build_topics(args: argparse.Namespace) -> list[str]:
    if args.all_topics:
        return []
    topics = list(LITE_TOPICS if args.lite else DEFAULT_TOPICS)
    if args.raw_cloud and RAW_CLOUD_TOPIC not in topics:
        topics.insert(0, RAW_CLOUD_TOPIC)
    return topics


def create_preflight(args: argparse.Namespace, artifact_dir: Path, topics: list[str]) -> dict[str, Any]:
    cwd = repo_root()
    topic_result = run_command(["ros2", "topic", "list", "-t"], cwd=cwd)
    topics_seen = parse_topic_list(topic_result.stdout) if topic_result.returncode == 0 else {}

    logs_dir = artifact_dir / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    (logs_dir / "topic-list.txt").write_text(topic_result.stdout + topic_result.stderr, encoding="utf-8")

    node_result = run_command(["ros2", "node", "list"], cwd=cwd)
    nodes_seen = {line.strip() for line in node_result.stdout.splitlines() if line.strip()} if node_result.returncode == 0 else set()
    (logs_dir / "node-list.txt").write_text(node_result.stdout + node_result.stderr, encoding="utf-8")

    node_info_dir = logs_dir / "node-info"
    node_info_dir.mkdir(parents=True, exist_ok=True)
    for node in NODE_INFO_TARGETS:
        if node not in nodes_seen:
            continue
        info = run_command(["ros2", "node", "info", node], cwd=cwd, timeout=5.0)
        safe_name = node.strip("/").replace("/", "_") or "root"
        (node_info_dir / f"{safe_name}.txt").write_text(info.stdout + info.stderr, encoding="utf-8")

    missing = sorted(topic for topic in topics if topic not in topics_seen)
    preflight = {
        "run_id": args.run_id,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "profile": "all-topics" if args.all_topics else ("nav-lite" if args.lite else "nav-full"),
        "capture_mode": "one-goal" if args.one_goal else ("duration" if args.duration is not None else "session"),
        "all_topics": bool(args.all_topics),
        "raw_cloud": bool(args.raw_cloud),
        "recorded_topics": topics if not args.all_topics else ["*"],
        "missing_topics": missing,
        "topic_list_returncode": topic_result.returncode,
        "node_list_returncode": node_result.returncode,
        "nodes_seen": sorted(nodes_seen),
    }
    write_json(artifact_dir / "checks" / "preflight.json", preflight)
    return preflight


def start_recording(args: argparse.Namespace, artifact_dir: Path, topics: list[str]) -> subprocess.Popen[str]:
    bag_dir = artifact_dir / "rosbags" / args.run_id
    if bag_dir.exists() and any(bag_dir.iterdir()):
        raise RuntimeError(f"bag directory already exists and is not empty: {bag_dir}")
    bag_dir.parent.mkdir(parents=True, exist_ok=True)

    cmd = ["ros2", "bag", "record", "--include-hidden-topics", "-o", str(bag_dir)]
    if args.all_topics:
        cmd.append("-a")
    else:
        cmd.extend(topics)

    if args.dry_run:
        print(" ".join(shlex.quote(part) for part in cmd))
        raise SystemExit(0)

    log_path = artifact_dir / "logs" / "rosbag-record.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        cmd,
        cwd=str(repo_root()),
        text=True,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    process._autoracer_log_file = log_file  # type: ignore[attr-defined]
    return process


def stop_recording(process: subprocess.Popen[str], timeout: float = 10.0) -> None:
    if process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()

    log_file = getattr(process, "_autoracer_log_file", None)
    if log_file is not None:
        log_file.close()


def wait_duration(seconds: float) -> None:
    end_time = time.monotonic() + seconds
    while not STOP_REQUESTED and time.monotonic() < end_time:
        time.sleep(min(0.5, end_time - time.monotonic()))


def wait_session() -> None:
    print("Recording navigation session. Stop the parent nav command or press Ctrl-C to finish.")
    while not STOP_REQUESTED:
        time.sleep(1.0)


def wait_for_nav_goal(terminal_delay: float) -> None:
    try:
        import rclpy
        from action_msgs.msg import GoalStatusArray
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError(
            "rclpy/action_msgs are required for default goal observation; use --duration instead"
        ) from exc

    class GoalObserver(Node):
        def __init__(self) -> None:
            super().__init__("autoracer_nav_capture_goal_observer")
            self.active_seen = False
            self.terminal_seen = False
            self.terminal_time: float | None = None
            self.subscription = self.create_subscription(
                GoalStatusArray,
                "/navigate_to_pose/_action/status",
                self.on_status,
                10,
            )

        def on_status(self, msg: GoalStatusArray) -> None:
            statuses = [int(status.status) for status in msg.status_list]
            if any(status in ACTIVE_STATUS for status in statuses):
                self.active_seen = True
            if self.active_seen and any(status in TERMINAL_STATUS for status in statuses):
                self.terminal_seen = True
                self.terminal_time = time.monotonic()

    rclpy.init(args=None)
    node = GoalObserver()
    try:
        print("Waiting for one /navigate_to_pose goal. Press Ctrl-C to stop recording manually.")
        while rclpy.ok() and not STOP_REQUESTED:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.terminal_seen and node.terminal_time is not None:
                if time.monotonic() - node.terminal_time >= terminal_delay:
                    return
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def msg_time_sec(msg: Any) -> float | None:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    return float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) * 1.0e-9


def twist_magnitude(msg: Any) -> float:
    linear = getattr(msg, "linear", None)
    angular = getattr(msg, "angular", None)
    vx = float(getattr(linear, "x", 0.0)) if linear is not None else 0.0
    wz = float(getattr(angular, "z", 0.0)) if angular is not None else 0.0
    return max(abs(vx), abs(wz))


def extract_diag_value(msg: Any, key: str) -> str | None:
    for status in getattr(msg, "status", []):
        for item in getattr(status, "values", []):
            if getattr(item, "key", "") == key:
                return str(getattr(item, "value", ""))
    return None


def maybe_mark_terminal_chassis_stall(summary: BagSummary, terminal_sec: float) -> None:
    ack = summary.last_ack_sample
    chassis = summary.last_chassis_sample
    if ack is None or chassis is None:
        return

    ack_recent = 0.0 <= terminal_sec - float(ack["sec"]) <= 2.0
    chassis_recent = 0.0 <= terminal_sec - float(chassis["sec"]) <= 2.0
    command_nonzero = abs(float(ack["speed_mps"])) > 0.05 and not bool(ack["brake"]) and not bool(ack["estop"])
    chassis_stopped = abs(float(chassis["actual_speed_mps"])) < 0.03 and int(chassis["hall_delta_count"]) == 0
    safety_clear = not any(
        bool(chassis[name])
        for name in ("brake_active", "rc_override_active", "command_timeout", "estop_active")
    )
    if ack_recent and chassis_recent and command_nonzero and chassis_stopped and safety_clear:
        summary.terminal_chassis_stall = True
        summary.timeline.setdefault("terminal_chassis_stall_sec", terminal_sec)
        summary.timeline.setdefault("terminal_ack_speed_mps", float(ack["speed_mps"]))
        summary.timeline.setdefault("terminal_ack_steering_angle_rad", float(ack["steering_angle_rad"]))
        summary.timeline.setdefault("terminal_chassis_actual_speed_mps", float(chassis["actual_speed_mps"]))
        summary.timeline.setdefault("terminal_chassis_status_bits", int(chassis["status_bits"]))


def format_goal_id(status: Any) -> str:
    goal_info = getattr(status, "goal_info", None)
    goal_id = getattr(goal_info, "goal_id", None)
    uuid = getattr(goal_id, "uuid", None)
    if uuid is None:
        return "unknown"
    try:
        return bytes(uuid).hex()
    except TypeError:
        return "".join(f"{int(item):02x}" for item in uuid)


def summarize_bag(bag_path: Path) -> tuple[BagSummary, str | None]:
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        return BagSummary(), f"rosbag2_py summary unavailable: {exc}"

    reader = SequentialReader()
    try:
        reader.open(
            StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
            ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
        )
    except Exception:
        reader.open(
            StorageOptions(uri=str(bag_path), storage_id=""),
            ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
        )

    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    type_cache: dict[str, Any] = {}
    summary = BagSummary()
    first_ns: int | None = None
    last_nav2_mag: float | None = None
    terminal_time: float | None = None
    last_goal_status: dict[str, int] = {}

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if first_ns is None:
            first_ns = int(timestamp_ns)
        rel_sec = (int(timestamp_ns) - first_ns) * 1.0e-9

        stats = summary.topic_stats.setdefault(topic, TopicStats())
        stats.count += 1
        if stats.first_sec is None:
            stats.first_sec = rel_sec
        stats.last_sec = rel_sec

        msg_type_name = topic_types.get(topic)
        if msg_type_name is None:
            continue
        try:
            msg_type = type_cache.setdefault(msg_type_name, get_message(msg_type_name))
            msg = deserialize_message(data, msg_type)
        except Exception:
            continue

        if topic == "/navigate_to_pose/_action/status":
            statuses = []
            for status in getattr(msg, "status_list", []):
                status_code = int(getattr(status, "status", 0))
                statuses.append(status_code)
                goal_id = format_goal_id(status)
                if last_goal_status.get(goal_id) == status_code:
                    continue
                last_goal_status[goal_id] = status_code
                summary.goal_events.append(
                    {
                        "sec": rel_sec,
                        "goal_id": goal_id,
                        "status_code": status_code,
                        "status": STATUS_NAMES.get(status_code, str(status_code)),
                    }
                )

            if any(status in ACTIVE_STATUS for status in statuses) and "goal_active_sec" not in summary.timeline:
                summary.timeline["goal_active_sec"] = rel_sec
                summary.timeline["goal_active_statuses"] = [STATUS_NAMES.get(status, str(status)) for status in statuses]
                summary.timeline["first_goal_active_sec"] = rel_sec
            if any(status in TERMINAL_STATUS for status in statuses):
                terminal_time = rel_sec
                maybe_mark_terminal_chassis_stall(summary, rel_sec)
                summary.timeline["goal_terminal_sec"] = rel_sec
                summary.timeline.setdefault("first_goal_terminal_sec", rel_sec)
                summary.timeline["goal_terminal_statuses"] = [
                    STATUS_NAMES.get(status, str(status)) for status in statuses if status in TERMINAL_STATUS
                ]

        elif topic == "/plan" and "first_plan_sec" not in summary.timeline:
            summary.timeline["first_plan_sec"] = rel_sec
            poses = getattr(msg, "poses", [])
            summary.timeline["first_plan_pose_count"] = len(poses)

        elif topic == "/cmd_vel_nav" and "first_cmd_vel_nav_sec" not in summary.timeline:
            summary.timeline["first_cmd_vel_nav_sec"] = rel_sec

        elif topic == "/nav2_cmd_vel":
            last_nav2_mag = twist_magnitude(msg)
            if last_nav2_mag > 0.02 and "first_nav2_cmd_vel_nonzero_sec" not in summary.timeline:
                summary.timeline["first_nav2_cmd_vel_nonzero_sec"] = rel_sec

        elif topic == "/safe_nav2_cmd_vel":
            safe_mag = twist_magnitude(msg)
            if safe_mag > 0.02 and "first_safe_nav2_cmd_vel_nonzero_sec" not in summary.timeline:
                summary.timeline["first_safe_nav2_cmd_vel_nonzero_sec"] = rel_sec
            if last_nav2_mag is not None and last_nav2_mag > 0.05 and safe_mag < max(0.02, last_nav2_mag * 0.5):
                summary.safe_suppressed = True
                summary.timeline.setdefault("first_safe_cmd_suppressed_sec", rel_sec)

        elif topic == "/twist_to_ackermann/diagnostics":
            reason = extract_diag_value(msg, "stop_reason")
            if reason is not None:
                summary.last_adapter_stop_reason = reason
                if reason not in ("none", "zero_command"):
                    summary.timeline.setdefault("first_adapter_stop_reason_sec", rel_sec)
                    summary.timeline.setdefault("first_adapter_stop_reason", reason)

        elif topic == "/ackermann_cmd":
            speed = abs(float(getattr(msg, "speed_mps", 0.0)))
            steering = abs(float(getattr(msg, "steering_angle_rad", 0.0)))
            summary.last_ack_sample = {
                "sec": rel_sec,
                "speed_mps": float(getattr(msg, "speed_mps", 0.0)),
                "steering_angle_rad": float(getattr(msg, "steering_angle_rad", 0.0)),
                "brake": bool(getattr(msg, "brake", False)),
                "estop": bool(getattr(msg, "emergency_stop", False)),
            }
            if max(speed, steering) > 1.0e-3:
                summary.last_ack_nonzero = True
                summary.timeline.setdefault("first_ackermann_nonzero_sec", rel_sec)

        elif topic == "/chassis_state":
            speed = abs(float(getattr(msg, "actual_speed_mps", 0.0)))
            summary.last_chassis_sample = {
                "sec": rel_sec,
                "actual_speed_mps": float(getattr(msg, "actual_speed_mps", 0.0)),
                "hall_delta_count": int(getattr(msg, "hall_delta_count", 0)),
                "brake_active": bool(getattr(msg, "brake_active", False)),
                "rc_override_active": bool(getattr(msg, "rc_override_active", False)),
                "command_timeout": bool(getattr(msg, "command_timeout", False)),
                "estop_active": bool(getattr(msg, "estop_active", False)),
                "status_bits": int(getattr(msg, "status_bits", 0)),
            }
            if speed > 0.03:
                summary.chassis_moved = True
                summary.timeline.setdefault("first_chassis_motion_sec", rel_sec)
            for field_name in ("brake_active", "rc_override_active", "estop_active", "command_timeout"):
                value = bool(getattr(msg, field_name, False))
                summary.chassis_safety[field_name] = summary.chassis_safety.get(field_name, False) or value
                if value:
                    summary.timeline.setdefault(f"first_chassis_{field_name}_sec", rel_sec)

        elif topic == "/rosout":
            message = str(getattr(msg, "msg", ""))
            lower = message.lower()
            if any(keyword in lower for keyword in ROSOUT_KEYWORDS):
                item = {
                    "sec": rel_sec,
                    "name": str(getattr(msg, "name", "")),
                    "level": int(getattr(msg, "level", 0)),
                    "message": message,
                }
                summary.key_rosout.append(item)
                summary.key_rosout = summary.key_rosout[-30:]
                summary.last_rosout_key = item

    if terminal_time is not None:
        before_terminal = [item for item in summary.key_rosout if float(item["sec"]) <= terminal_time + 0.001]
        if before_terminal:
            summary.last_rosout_key = before_terminal[-1]
    return summary, None


def classify(summary: BagSummary, preflight: dict[str, Any], summary_error: str | None) -> tuple[str, str]:
    evidence = summary.evidence
    missing = summary.missing_evidence

    if summary_error:
        missing.append(summary_error)
        return "unknown_insufficient_evidence", "bag summary could not be parsed on this machine"

    missing_topics = set(preflight.get("missing_topics", []))
    topic_counts = {topic: stats.count for topic, stats in summary.topic_stats.items()}

    def no_messages(topic: str) -> bool:
        return topic_counts.get(topic, 0) == 0

    for topic in ("/scan", "/tf", "/odom", "/amcl_pose"):
        if no_messages(topic):
            detail = "missing or unrecorded"
            if topic in missing_topics:
                detail = "missing during preflight and unrecorded"
            missing.append(f"{topic} {detail}")

    if any(item.startswith(topic) for topic in ("/scan", "/tf", "/odom", "/amcl_pose") for item in missing):
        evidence.append("navigation lacks one or more sensor/localization streams")
        return "sensor_localization", "missing /scan, TF, odom, or AMCL evidence before/inside the task"

    rosout_text = "\n".join(item.get("message", "") for item in summary.key_rosout).lower()
    if "failed to create plan" in rosout_text or "exceeded maximum iterations" in rosout_text or "no valid path" in rosout_text:
        evidence.append("planner/costmap failure appears in /rosout")
        return "planner_costmap", "planner failed before a valid path could drive the controller"

    if no_messages("/plan"):
        missing.append("/plan missing or unrecorded")
        if "goal_terminal_sec" in summary.timeline:
            evidence.append("goal reached a terminal state without recorded /plan")
            return "planner_costmap", "no recorded global plan for the terminal navigation task"

    if summary.terminal_chassis_stall:
        evidence.append("terminal window had nonzero /ackermann_cmd while /chassis_state stayed stopped")
        if "failed to make progress" in rosout_text:
            evidence.append("progress checker abort followed the chassis stall")
        return "chassis", "low-speed chassis stall: command reached STM32, but Hall/actual speed stayed zero before abort"

    if "actionserver aborting" in rosout_text or "bt navigator" in rosout_text and "abort" in rosout_text:
        evidence.append("BT navigator abort appears in /rosout")
        return "bt_navigator", "behavior tree aborted the navigation task; inspect adjacent planner/controller logs"

    if "failed to make progress" in rosout_text:
        evidence.append("progress checker failure appears in /rosout")
        return "controller", "controller/progress failure; verify odom, AMCL stability, and RPP tracking inputs"

    if not no_messages("/plan") and no_messages("/cmd_vel_nav"):
        missing.append("/cmd_vel_nav missing after /plan")
        return "controller", "global plan exists but controller output was not recorded"

    if not no_messages("/cmd_vel_nav") and no_messages("/nav2_cmd_vel"):
        missing.append("/nav2_cmd_vel missing after /cmd_vel_nav")
        return "velocity_smoother", "controller output exists but velocity smoother output was not recorded"

    if summary.safe_suppressed:
        evidence.append("/safe_nav2_cmd_vel was suppressed relative to /nav2_cmd_vel")
        return "collision_monitor", "Collision Monitor likely stopped or slowed the command stream"

    if summary.last_adapter_stop_reason and summary.last_adapter_stop_reason not in ("none", "zero_command"):
        evidence.append(f"adapter stop_reason={summary.last_adapter_stop_reason}")
        return "adapter", "twist_to_ackermann rejected, timed out, or stopped the safe Twist"

    if summary.last_ack_nonzero and not summary.chassis_moved:
        evidence.append("/ackermann_cmd became nonzero but chassis motion was not observed")
        return "chassis", "command reached the chassis interface but actual motion was not recorded"

    active_safety = [name for name, enabled in summary.chassis_safety.items() if enabled]
    if active_safety:
        evidence.append(f"chassis safety state active: {', '.join(active_safety)}")
        return "chassis", "STM32 safety or vehicle state blocked motion"

    return "unknown_insufficient_evidence", "no single first failed layer could be determined from the recorded evidence"


def format_topic_stats(summary: BagSummary) -> dict[str, dict[str, Any]]:
    return {
        topic: {
            "count": stats.count,
            "first_sec": stats.first_sec,
            "last_sec": stats.last_sec,
        }
        for topic, stats in sorted(summary.topic_stats.items())
    }


def create_post_summary(args: argparse.Namespace, artifact_dir: Path, preflight: dict[str, Any]) -> None:
    bag_path = artifact_dir / "rosbags" / args.run_id
    summary, summary_error = summarize_bag(bag_path)
    first_layer, suspected = classify(summary, preflight, summary_error)
    if first_layer not in FIRST_FAILED_LAYERS:
        first_layer = "unknown_insufficient_evidence"

    result = {
        "run_id": args.run_id,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "bag_path": str(bag_path),
        "profile": preflight.get("profile"),
        "capture_mode": preflight.get("capture_mode"),
        "first_failed_layer": first_layer,
        "suspected_upstream_cause": suspected,
        "timeline": summary.timeline,
        "goal_events": summary.goal_events,
        "last_rosout_key": summary.last_rosout_key,
        "key_rosout_tail": summary.key_rosout[-10:],
        "last_adapter_stop_reason": summary.last_adapter_stop_reason,
        "ackermann_nonzero_observed": summary.last_ack_nonzero,
        "chassis_motion_observed": summary.chassis_moved,
        "chassis_safety_observed": summary.chassis_safety,
        "safe_cmd_suppressed": summary.safe_suppressed,
        "terminal_chassis_stall": summary.terminal_chassis_stall,
        "evidence": summary.evidence,
        "missing_evidence": sorted(set(summary.missing_evidence)),
        "topic_stats": format_topic_stats(summary),
        "summary_error": summary_error,
    }
    write_json(artifact_dir / "checks" / "post_summary.json", result)


def parse_args() -> argparse.Namespace:
    default_run_id = datetime.now().strftime("%Y-%m-%d-nav-%H%M%S")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default=default_run_id, help="Run id used under docs/test-records/artifacts")
    parser.add_argument(
        "--output-root",
        default=str(repo_root() / "docs" / "test-records" / "artifacts"),
        help="Artifact root directory",
    )
    parser.add_argument("--duration", type=float, default=None, help="Record for a fixed number of seconds")
    parser.add_argument(
        "--one-goal",
        action="store_true",
        help="Stop after one /navigate_to_pose goal reaches a terminal state",
    )
    parser.add_argument(
        "--terminal-delay",
        type=float,
        default=3.0,
        help="Seconds to keep recording after goal terminal state when --one-goal is used",
    )
    parser.add_argument("--lite", action="store_true", help="Record a smaller diagnostic topic set")
    parser.add_argument("--raw-cloud", action="store_true", help="Include /point_cloud_raw in the default topic set")
    parser.add_argument("--all-topics", action="store_true", help="Record all topics instead of the curated nav-full set")
    parser.add_argument("--dry-run", action="store_true", help="Print rosbag command after writing preflight files, then exit")
    return parser.parse_args()


def main() -> int:
    ensure_ros_environment()
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    args = parse_args()
    artifact_dir = Path(args.output_root).expanduser().resolve() / args.run_id
    topics = build_topics(args)
    preflight = create_preflight(args, artifact_dir, topics)

    process = start_recording(args, artifact_dir, topics)
    try:
        if args.duration is not None:
            wait_duration(max(0.0, args.duration))
        elif args.one_goal:
            wait_for_nav_goal(max(0.0, args.terminal_delay))
        else:
            wait_session()
    except KeyboardInterrupt:
        request_stop(signal.SIGINT, None)
    finally:
        if STOP_REQUESTED:
            print("Stopping capture.")
        stop_recording(process)

    create_post_summary(args, artifact_dir, preflight)
    print(f"Artifacts written to: {artifact_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
